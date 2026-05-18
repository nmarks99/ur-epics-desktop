#pragma once
#include <atomic>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <variant>
#include <vector>

#include "cadef.h"

namespace ezec {

namespace detail {

using MonitorVariant = std::variant<std::monostate, double, int, std::string>;

template <typename T>
std::optional<T> convert(const MonitorVariant& v) {
    if (auto* val = std::get_if<T>(&v)) {
        return *val;
    }
    if constexpr (std::is_arithmetic_v<T>) {
        if (auto* d = std::get_if<double>(&v)) {
            return static_cast<T>(*d);
        }
        if (auto* i = std::get_if<int>(&v)) {
            return static_cast<T>(*i);
        }
    }
    if constexpr (std::is_same_v<T, std::string>) {
        if (auto* d = std::get_if<double>(&v)) {
            return std::to_string(*d);
        }
        if (auto* i = std::get_if<int>(&v)) {
            return std::to_string(*i);
        }
    }
    return std::nullopt;
}

struct MonitorSlotBase {
    virtual ~MonitorSlotBase() = default;
    virtual void copy_to_targets(const MonitorVariant& staged) = 0;
};

template <typename T>
struct MonitorSlot : MonitorSlotBase {
    std::vector<T*> targets;
    void copy_to_targets(const MonitorVariant& staged) override {
        if (auto val = convert<T>(staged)) {
            for (T* t : targets) {
                *t = *val;
            }
        }
    }
};

} // namespace detail

class ChannelBase {
  public:
    ChannelBase(const std::string& pv_name) : pv_name_(pv_name) {}
    virtual ~ChannelBase() = default;

    virtual bool connected() const = 0;

    template <typename T>
    void bind(T& var) {
        std::lock_guard lock(mutex_);
        for (auto& slot : slots_) {
            if (auto* typed = dynamic_cast<detail::MonitorSlot<T>*>(slot.get())) {
                typed->targets.push_back(&var);
                return;
            }
        }
        auto slot = std::make_unique<detail::MonitorSlot<T>>();
        slot->targets.push_back(&var);
        slots_.push_back(std::move(slot));
    }

    bool sync() {
        if (!new_data_.load(std::memory_order_acquire)) {
            return false;
        }
        std::lock_guard lock(mutex_);
        for (auto& slot : slots_) {
            slot->copy_to_targets(staged_value_);
        }
        new_data_.store(false, std::memory_order_relaxed);
        return true;
    }

  protected:
    std::string pv_name_;
    std::mutex mutex_;
    std::atomic<bool> new_data_{false};
    detail::MonitorVariant staged_value_;
    std::vector<std::unique_ptr<detail::MonitorSlotBase>> slots_;
};

class CAChannel : public ChannelBase {
  public:
    CAChannel(const std::string& pv_name) : ChannelBase(pv_name) {
        if (!ca_current_context()) {
            should_destroy_context_ = true;
            SEVCHK(ca_context_create(ca_enable_preemptive_callback), "ca_context_create");
        }
        SEVCHK(
            ca_create_channel(pv_name.c_str(), connection_callback, this, CA_PRIORITY_DEFAULT, &channel_id_),
            "ca_create_channel");
        SEVCHK(ca_flush_io(), "ca_flush_io");
    }

    ~CAChannel() {
        if (evt_id_) {
            ca_clear_subscription(evt_id_);
        }
        ca_clear_channel(channel_id_);
        if (should_destroy_context_) {
            ca_context_destroy();
        }
    }

    bool connected() const override { return connected_.load(std::memory_order_relaxed); }

    template <typename T>
    void put(T value) {
        if (connected()) {
            auto dbr_type = ca_field_type(channel_id_);
            assert_type_match<T>(dbr_type);
            ca_put(ca_field_type(channel_id_), channel_id_, &value);
            ca_pend_io(1.0);
        }
    }

  private:
    chid channel_id_;
    evid evt_id_ = nullptr;
    std::atomic<bool> connected_{false};
    bool should_destroy_context_ = false;

    template <typename T>
    void assert_type_match(chtype dbr_type) {
        bool ok = false;
        switch (dbr_type) {
        case DBR_LONG:
            ok = std::is_integral_v<T>;
            break;
        case DBR_SHORT:
            ok = std::is_integral_v<T>;
            break;
        case DBR_FLOAT:
            ok = std::is_floating_point_v<T>;
            break;
        case DBR_DOUBLE:
            ok = std::is_floating_point_v<T>;
            break;
        case DBR_STRING:
            ok = std::is_same_v<T, std::string> || std::is_same_v<T, char*> || std::is_same_v<T, const char*>;
            break;
        }
        if (!ok) {
            throw std::runtime_error("CA put type mismatch");
        }
    }

    void start_monitor() {
        if (evt_id_) {
            ca_clear_subscription(evt_id_);
            evt_id_ = nullptr;
        }

        // auto native = ca_field_type(channel_id_);
        // chtype dbr;
        // if (native == DBF_STRING) {
            // dbr = DBR_STRING;
        // } else if (native == DBF_ENUM) {
            // dbr = DBR_LONG;
        // } else {
            // dbr = DBR_DOUBLE;
        // }

        auto dbr = ca_field_type(channel_id_);
        SEVCHK(ca_create_subscription(dbr, 1, channel_id_, DBE_VALUE | DBE_ALARM, subscription_callback, this,
                                      &evt_id_),
               "ca_create_subscription");
        SEVCHK(ca_flush_io(), "ca_flush_io");
    }

    static void connection_callback(struct connection_handler_args args) {
        auto* self = static_cast<CAChannel*>(ca_puser(args.chid));
        if (args.op == CA_OP_CONN_UP) {
            self->connected_.store(true, std::memory_order_relaxed);
            self->start_monitor();
        } else {
            self->connected_.store(false, std::memory_order_relaxed);
        }
    }

    static void subscription_callback(struct event_handler_args evt) {
        auto* self = static_cast<CAChannel*>(evt.usr);
        if (evt.status != ECA_NORMAL) {
            return;
        }

        detail::MonitorVariant value;
        short dbr = evt.type;
        if (dbr == DBR_DOUBLE) {
            value = *static_cast<const dbr_double_t*>(evt.dbr);
        } else if (dbr == DBR_LONG) {
            value = static_cast<int>(*static_cast<const dbr_long_t*>(evt.dbr));
        } else if (dbr == DBR_STRING) {
            value = std::string(static_cast<const char*>(evt.dbr));
        } else {
            // TODO: implemented all the DBR_ types and cast correctly
            throw std::runtime_error(std::string(dbr_type_to_text(dbr)) + " monitor unimplemented");
        }

        std::lock_guard lock(self->mutex_);
        self->staged_value_ = value;
        self->new_data_.store(true, std::memory_order_release);
    }
};

class ChannelGroup {
  public:
    void add(const std::string& pv_name) {
        std::lock_guard lock(mutex_);
        if (channel_map_.count(pv_name) == 0) {
            channel_map_.emplace(pv_name, std::make_unique<CAChannel>(pv_name));
        }
    }

    bool sync() {
        std::lock_guard lock(mutex_);
        bool new_data = false;
        for (auto& [pv_name, channel] : channel_map_) {
            if (channel->sync()) {
                new_data = true;
            }
        }
        return new_data;
    }

    template <typename T>
    void bind(T& var, const std::string& pv_name) {
        std::lock_guard lock(mutex_);
        get_channel_unlocked(pv_name).bind(var);
    }

    ChannelBase& get_channel(const std::string& pv_name) {
        std::lock_guard lock(mutex_);
        return get_channel_unlocked(pv_name);
    }

  private:
    std::mutex mutex_;
    std::unordered_map<std::string, std::unique_ptr<ChannelBase>> channel_map_;

    ChannelBase& get_channel_unlocked(const std::string& pv_name) {
        auto it = channel_map_.find(pv_name);
        if (it == channel_map_.end()) {
            throw std::runtime_error(pv_name + " not registered");
        }
        return *it->second;
    }
};

} // namespace ezec
