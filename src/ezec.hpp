#pragma once
#include <atomic>
#include <cstring>
#include <iomanip>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <unordered_map>
#include <variant>
#include <vector>

#include "cadef.h"

namespace ezec {

namespace detail {

/// \brief std::variant type that holds the latest value from an EPICS subscription.
///
/// PVs that cannot be represented as one of the types in this variant are
/// unsupported.
using ValueVariant = std::variant<std::monostate, double, int, std::string>;

/// \brief Convert a ValueVariant to a target type T.
///
/// The ValueVariant holds the latest value from a CA subscription callback
/// This function attempts to convert it to the type T of the user's bound
/// variable. Returns std::nullopt if the conversion is not supported
template <typename T>
std::optional<T> convert(const ValueVariant& v, int precision = 4) {

    // Target type T is the same as the variant's type
    if (auto* val = std::get_if<T>(&v)) {
        return *val;
    }

    // Target type T and variant are different, but
    // both arithmetic types, so we cast.
    if constexpr (std::is_arithmetic_v<T>) {
        if (auto* d = std::get_if<double>(&v)) {
            return static_cast<T>(*d);
        }
        if (auto* i = std::get_if<int>(&v)) {
            return static_cast<T>(*i);
        }
    }

    // Target type T is string, and variant type is not
    // so we convert to string if we can.
    if constexpr (std::is_same_v<T, std::string>) {
        if (auto* d = std::get_if<double>(&v)) {
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(precision) << *d;
            return oss.str();
        }
        if (auto* i = std::get_if<int>(&v)) {
            return std::to_string(*i);
        }
    }

    return std::nullopt;
}

/// \brief Type-erased base class for monitor fan-out slots.
///
/// ChannelBase stores a vector of MonitorSlotBase pointers to fan out a single
/// staged ValueVariant to bound variables of potentially different types.
/// Each concrete MonitorSlot<T> handles conversion and distribution for one
/// target type. This provides type erasure so that ChannelBase doesn't need to
/// know the types of the user's bound variables.
///
/// Slots are created on demand by bind<T>():
/// - If a MonitorSlot<T> already exists, the new variable pointer is appended
///   to its targets vector.
/// - Otherwise, a new MonitorSlot<T> is created.
///
/// When sync() runs, it iterates all slots and calls copy_to_targets() on each.
/// Each slot independently converts the variant to its type T via convert<T>()
/// and writes the result to all of its target pointers.
struct MonitorSlotBase {
    virtual ~MonitorSlotBase() = default;
    virtual void copy_to_targets(const ValueVariant& staged, int precision = 4) = 0;
};

/// \brief Concrete slot that fans out a ValueVariant to bound variables of type T.
///
/// Holds raw pointers to the user's bound variables. When copy_to_targets() is
/// called, it converts the staged value via convert<T>() and writes to every
/// target. If conversion fails (e.g. string variant to double target), the
/// targets are left unchanged.
template <typename T>
struct MonitorSlot : MonitorSlotBase {
    std::vector<T*> targets;
    void copy_to_targets(const ValueVariant& staged, int precision = 4) override {
        if (auto val = convert<T>(staged, precision)) {
            for (T* t : targets) {
                *t = *val;
            }
        }
    }
};

} // namespace detail

/// \brief Abstract base class for a channel bound to a single PV.
///
/// Provides the bind/sync mechanism that decouples the network callback thread
/// from the user's polling thread. Subclasses (e.g. CAChannel) are responsible
/// for connecting to the PV and writing into staged_value_ when new data
/// arrives. The user calls sync() to copy the staged value to all bound
/// variables.
class ChannelBase {
  public:
    ChannelBase(const std::string& pv_name) : pv_name_(pv_name) {}
    virtual ~ChannelBase() = default;

    /// \brief Returns true if the channel is currently connected to the PV.
    virtual bool connected() const = 0;

    /// \brief Writes a value to the PV.
    ///
    /// The type T must be convertible to one of the supported types.
    ///
    /// \param value The value to write.
    /// \return true if the put was sent, false if the channel is disconnected
    /// or the value is not convertible to a supported type.
    template <typename T>
    bool put(const T& value) {
        return put(detail::ValueVariant(value));
    }

    /// \brief Write a string literal to the PV
    /// \overload
    bool put(const char* value) { return put(detail::ValueVariant(std::string(value))); }

    /// \brief Bind a user variable to receive monitor updates from this channel.
    ///
    /// The variable will be updated with the latest PV value each time sync() is
    /// called. Multiple variables can be bound to the same channel, including
    /// variables of different types. The bound variable must outlive the channel.
    ///
    /// \param var Reference to the user's variable. A raw pointer to this
    ///            variable is stored internally.
    template <typename T>
    void bind(T& var) {
        std::lock_guard lock(mutex_);
        // If a MonitorSlot for this T already exists,
        // store this pointer in its target vector
        for (auto& slot : slots_) {
            if (auto* typed = dynamic_cast<detail::MonitorSlot<T>*>(slot.get())) {
                typed->targets.push_back(&var);
                return;
            }
        }
        // Create a new slot if no slot for T exists already
        auto slot = std::make_unique<detail::MonitorSlot<T>>();
        slot->targets.push_back(&var);
        slots_.push_back(std::move(slot));
    }

    /// \brief Copy the latest staged value to all bound variables.
    ///
    /// Returns true if new data was available since the last call to sync().
    /// This is the user's polling point. Call sync() periodically from your
    /// application loop.
    bool sync() {
        if (!new_data_.load(std::memory_order_acquire)) {
            return false;
        }
        std::lock_guard lock(mutex_);
        for (auto& slot : slots_) {
            slot->copy_to_targets(staged_value_, precision_);
        }
        new_data_.store(false, std::memory_order_relaxed);
        return true;
    }

  private:
    std::vector<std::unique_ptr<detail::MonitorSlotBase>> slots_;

  protected:
    /// \brief CA/PVA specific put implementation. Overridden by subclasses.
    virtual bool put(const detail::ValueVariant& value) = 0;

    std::string pv_name_;
    std::mutex mutex_;
    std::atomic<bool> new_data_{false};
    detail::ValueVariant staged_value_;
    int precision_ = 4;
};

/// \brief Helper class for creation/destruction of EPICS (CA/PVA) context
class Context {
  public:
    Context() { SEVCHK(ca_context_create(ca_enable_preemptive_callback), "ca_context_create"); }
    ~Context() { ca_context_destroy(); }
    Context(const Context&) = delete;
    Context& operator=(const Context&) = delete;
};

/// \brief Channel Access implementation of ChannelBase.
///
/// Connects to a single PV via the EPICS Channel Access protocol. A CA context
/// (e.g. ezec::Context) must exist before construction. On connection, a
/// monitor subscription is created automatically and the channel begins
/// receiving value updates into the staged_value_. For floating-point PVs,
/// the record's PREC field is fetched at connection time.
///
/// For write operations, use put(), or id() for direct CA API access.
class CAChannel : public ChannelBase {
  public:
    using ChannelBase::put;

    CAChannel(const std::string& pv_name) : ChannelBase(pv_name) {
        if (!ca_current_context()) {
            throw std::runtime_error("No CA context. Call ca_context_create() before creating a CAChannel.");
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
    }

    bool connected() const override { return connected_.load(std::memory_order_relaxed); }

    /// \brief Returns the underlying CA channel identifier for direct CA API use.
    chid id() const { return channel_id_; }

  private:
    chid channel_id_;
    evid evt_id_ = nullptr;
    std::atomic<bool> connected_{false};

    /// \brief CA-specific put implementation. Sends the value via ca_put.
    bool put(const detail::ValueVariant& value) override {
        if (!connected()) {
            return false;
        }
        if (auto* v = std::get_if<double>(&value)) {
            dbr_double_t val = *v;
            ca_put(DBR_DOUBLE, channel_id_, &val);
        } else if (auto* v = std::get_if<int>(&value)) {
            dbr_long_t val = *v;
            ca_put(DBR_LONG, channel_id_, &val);
        } else if (auto* s = std::get_if<std::string>(&value)) {
            dbr_string_t v;
            strncpy(v, s->c_str(), sizeof(v) - 1);
            v[sizeof(v) - 1] = '\0';
            ca_put(DBR_STRING, channel_id_, &v);
        } else {
            return false; // monostate
        }
        ca_flush_io();
        return true;
    }

    /// \brief Creates a monitor subscription and fetches precision on connect.
    ///
    /// Called from the connection callback when the channel comes up. Clears
    /// any existing subscription before creating a new one. For DBF_FLOAT and
    /// DBF_DOUBLE PVs, also issues a one-time ca_get_callback to fetch PREC.
    void start_monitor() {
        if (evt_id_) {
            ca_clear_subscription(evt_id_);
            evt_id_ = nullptr;
        }

        auto native = ca_field_type(channel_id_);
        SEVCHK(ca_create_subscription(dbf_type_to_DBR(native), 1, channel_id_, DBE_VALUE | DBE_ALARM,
                                      subscription_callback, this, &evt_id_),
               "ca_create_subscription");

        if (native == DBF_FLOAT || native == DBF_DOUBLE) {
            ca_get_callback(DBR_CTRL_DOUBLE, channel_id_, precision_callback, this);
        }

        SEVCHK(ca_flush_io(), "ca_flush_io");
    }

    /// \brief CA connection state callback. Starts the monitor on connect.
    static void connection_callback(struct connection_handler_args args) {
        auto* self = static_cast<CAChannel*>(ca_puser(args.chid));
        if (args.op == CA_OP_CONN_UP) {
            self->connected_.store(true, std::memory_order_relaxed);
            self->start_monitor();
        } else {
            self->connected_.store(false, std::memory_order_relaxed);
        }
    }

    /// \brief One-time ca_get_callback handler that extracts PREC from DBR_CTRL_DOUBLE.
    /// Falls back to a default precision of 4 on failure.
    static void precision_callback(struct event_handler_args evt) {
        auto* self = static_cast<CAChannel*>(evt.usr);
        std::lock_guard lock(self->mutex_);
        if (evt.status == ECA_NORMAL) {
            auto* ctrl = static_cast<const struct dbr_ctrl_double*>(evt.dbr);
            self->precision_ = ctrl->precision;
        } else {
            self->precision_ = 4;
        }
    }

    /// \brief CA subscription callback. Converts the incoming value to a
    /// ValueVariant and stages it for the next sync() call.
    static void subscription_callback(struct event_handler_args evt) {
        auto* self = static_cast<CAChannel*>(evt.usr);
        if (evt.status != ECA_NORMAL) {
            return;
        }

        detail::ValueVariant value;
        switch (evt.type) {
        case DBR_DOUBLE:
            value = *static_cast<const dbr_double_t*>(evt.dbr);
            break;
        case DBR_FLOAT:
            value = static_cast<double>(*static_cast<const dbr_float_t*>(evt.dbr));
            break;
        case DBR_LONG:
            value = static_cast<int>(*static_cast<const dbr_long_t*>(evt.dbr));
            break;
        case DBR_SHORT:
            value = static_cast<int>(*static_cast<const dbr_short_t*>(evt.dbr));
            break;
        case DBR_CHAR:
            value = static_cast<int>(*static_cast<const dbr_char_t*>(evt.dbr));
            break;
        case DBR_ENUM:
            value = static_cast<int>(*static_cast<const dbr_enum_t*>(evt.dbr));
            break;
        case DBR_STRING:
            value = std::string(static_cast<const char*>(evt.dbr));
            break;
        default:
            return;
        }

        std::lock_guard lock(self->mutex_);
        self->staged_value_ = value;
        self->new_data_.store(true, std::memory_order_release);
    }
};

/// \brief PVAccess implementation of ChannelBase.
/// \warning This is not implemented yet
class PVAChannel : public ChannelBase {
  public:
    using ChannelBase::put;

    PVAChannel(const std::string& pv_name) : ChannelBase(pv_name) {}
    bool connected() const override { return false; }

  private:
    bool put(const detail::ValueVariant& value) override { return false; }
};

/// \brief Manages a collection of PV channels with a single sync() call.
///
/// ChannelGroup is the primary interface for monitoring multiple PVs. Register
/// PVs with add(), bind local variables with bind(), then call sync()
/// periodically to update all bound variables at once.
///
/// Example usage:
/// \code
///     ezec::Context ctx;
///     ezec::ChannelGroup group;
///
///     group.add("MyIOC:m1.RBV");
///     group.add("MyIOC:m1.DMOV");
///
///     double position = 0.0;
///     int done = 0;
///     group.bind(position, "MyIOC:m1.RBV");
///     group.bind(done, "MyIOC:m1.DMOV");
///
///     while (true) {
///         if (group.sync()) {
///             // At least one PV has new data
///             printf("pos=%f done=%d\n", position, done);
///         }
///         std::this_thread::sleep_for(std::chrono::milliseconds(100));
///     }
/// \endcode
class ChannelGroup {
  public:
    /// \brief Register a PV to be monitored.
    ///
    /// Creates a CAChannel and begins connecting. If the PV has already been
    /// added, this is a no-op. Must be called before bind() for the same PV.
    ///
    /// \param pv_name The PV name (e.g. "MyIOC:name.VAL").
    /// \return A reference to the ChannelBase that was added
    ChannelBase& add(const std::string& pv_name) {
        std::lock_guard lock(mutex_);
        if (channel_map_.count(pv_name) == 0) {
            channel_map_.emplace(pv_name, std::make_unique<CAChannel>(pv_name));
        }
        return this->get_channel_unlocked(pv_name);
    }

    /// \brief Bind a local variable to a registered PV.
    ///
    /// The PV must have been added with add() first, otherwise this throws
    /// std::runtime_error. The bound variable must outlive the ChannelGroup.
    /// Multiple variables (including different types) can be bound to the
    /// same PV.
    ///
    /// \param var  Reference to the local variable.
    /// \param pv_name  The PV name, must match a previous add() call.
    template <typename T>
    void bind(T& var, const std::string& pv_name) {
        std::lock_guard lock(mutex_);
        get_channel_unlocked(pv_name).bind(var);
    }

    /// \brief Sync all channels, updating every bound variable with new data.
    ///
    /// Returns true if at least one channel had new data since the last sync().
    /// Call this periodically from your application loop.
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

    /// \brief Get a reference to a registered channel.
    ///
    /// Throws std::runtime_error if the PV was not previously added.
    /// The returned reference can be used to check connection status or
    /// access the underlying channel directly.
    ///
    /// \param pv_name  The PV name, must match a previous add() call.
    ChannelBase& get_channel(const std::string& pv_name) {
        std::lock_guard lock(mutex_);
        return get_channel_unlocked(pv_name);
    }

    /// \brief Shorthand for get_channel().
    /// \overload
    ChannelBase& operator[](const std::string& pv_name) { return get_channel(pv_name); }

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
