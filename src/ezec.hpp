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
#include <pvxs/client.h>

namespace ezec {

namespace detail {

/// \brief std::variant type that holds the latest value from an EPICS subscription.
///
/// PVs that cannot be represented as one of the types in this variant are
/// unsupported.
using ValueVariant =
    std::variant<std::monostate, double, int, std::string, std::vector<double>, std::vector<int>>;

/// \brief Convert a ValueVariant to a target type T.
///
/// The ValueVariant holds the latest value from a subscription callback
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

    // TODO: Handle conversion between std::vector<double> and std::vector<int>

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

        // Inform the child class we need to create a subscription
        enable_monitor();
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

    /// \brief Returns the latest value from the monitor as the request type.
    ///
    /// Returns The latest value from the monitor as std::optional<T>.
    /// If the conversion to T fails, the returned value is std::nullopt;
    template <typename T>
    std::optional<T> peek() {
        std::lock_guard lock(mutex_);
        return detail::convert<T>(staged_value_);
    }

  private:
    std::vector<std::unique_ptr<detail::MonitorSlotBase>> slots_;

  protected:
    /// \brief CA/PVA specific put implementation.
    virtual bool put(const detail::ValueVariant& value) = 0;

    /// \brief Informs the CA/PVAChannel to create a monitor.
    virtual void enable_monitor() = 0;

    std::string pv_name_;
    std::mutex mutex_;
    std::atomic<bool> new_data_{false};
    detail::ValueVariant staged_value_;
    int precision_ = 4;
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

    void enable_monitor() override {
        if (wants_monitor_.load(std::memory_order_relaxed)) {
            // we have already created the monitor
            return;
        }
        wants_monitor_.store(true, std::memory_order_relaxed);
        start_monitor();
    }

  private:
    std::atomic<bool> connected_{false};
    std::atomic<bool> wants_monitor_{false};
    chid channel_id_;
    evid evt_id_ = nullptr;

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
            // No other puts are supported yet
            return false;
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
        if (wants_monitor_.load(std::memory_order_relaxed) && connected()) {
            if (evt_id_) {
                ca_clear_subscription(evt_id_);
                evt_id_ = nullptr;
            }

            auto native = ca_field_type(channel_id_);
            SEVCHK(ca_create_subscription(dbf_type_to_DBR(native), 0, channel_id_, DBE_VALUE | DBE_ALARM,
                                          subscription_callback, this, &evt_id_),
                   "ca_create_subscription");

            if (native == DBF_FLOAT || native == DBF_DOUBLE) {
                ca_get_callback(DBR_CTRL_DOUBLE, channel_id_, precision_callback, this);
            }

            SEVCHK(ca_flush_io(), "ca_flush_io");
        }
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

    static detail::ValueVariant get_scalar_event(struct event_handler_args evt) {
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
        }
        return value;
    }

    static detail::ValueVariant get_array_event(struct event_handler_args evt) {
        detail::ValueVariant value;
        auto count = static_cast<size_t>(evt.count);
        switch (evt.type) {
        case DBR_CHAR: {
            auto* arr = static_cast<const dbr_char_t*>(evt.dbr);
            std::vector<int> vec(arr, arr + count);
            value = std::move(vec);
            break;
        }
        case DBR_DOUBLE: {
            auto* arr = static_cast<const dbr_double_t*>(evt.dbr);
            std::vector<double> vec(arr, arr + count);
            value = std::move(vec);
            break;
        }
        case DBR_FLOAT: {
            auto* arr = static_cast<const dbr_float_t*>(evt.dbr);
            std::vector<double> vec(arr, arr + count);
            value = std::move(vec);
            break;
        }
        case DBR_LONG: {
            auto* arr = static_cast<const dbr_long_t*>(evt.dbr);
            std::vector<int> vec(arr, arr + count);
            value = std::move(vec);
            break;
        }
        case DBR_SHORT: {
            auto* arr = static_cast<const dbr_short_t*>(evt.dbr);
            std::vector<int> vec(arr, arr + count);
            value = std::move(vec);
            break;
        }
        case DBR_ENUM: {
            break;
        }
        case DBR_STRING: {
            break;
        }
        }
        return value;
    }

    /// \brief CA subscription callback. Converts the incoming value to a
    /// ValueVariant and stages it for the next sync() call.
    static void subscription_callback(struct event_handler_args evt) {
        auto* self = static_cast<CAChannel*>(evt.usr);
        if (evt.status != ECA_NORMAL) {
            return;
        }

        detail::ValueVariant value;
        if (evt.count == 1) {
            value = get_scalar_event(evt);
        } else if (evt.count > 1) {
            value = get_array_event(evt);
        }

        if (std::holds_alternative<std::monostate>(value)) {
            return;
        }

        std::lock_guard lock(self->mutex_);
        self->staged_value_ = value;
        self->new_data_.store(true, std::memory_order_release);
    }
};

/// \brief PVAccess implementation of ChannelBase.
/// \warning TODO: document this.
class PVAChannel : public ChannelBase {
  public:
    using ChannelBase::put;

    PVAChannel(pvxs::client::Context& context, const std::string& pv_name)
        : ChannelBase(pv_name), ctx_(context) {}

    ~PVAChannel() {
        if (subscription_) {
            subscription_->cancel();
        }
    }

    bool connected() const override { return connected_.load(std::memory_order_relaxed); }

    void enable_monitor() override {
        if (wants_monitor_.load(std::memory_order_relaxed)) {
            return;
        }
        wants_monitor_.store(true, std::memory_order_relaxed);
        start_monitor();
    }

  private:
    std::atomic<bool> connected_{false};
    std::atomic<bool> wants_monitor_{false};
    std::shared_ptr<pvxs::client::Subscription> subscription_;
    pvxs::client::Context& ctx_;

    void start_monitor() {
        if (!wants_monitor_.load(std::memory_order_relaxed)) {
            return;
        }
        if (subscription_) {
            return;
        }

        subscription_ = ctx_.monitor(pv_name_)
                            .maskConnected(true)
                            .maskDisconnected(true)
                            .event([this](pvxs::client::Subscription& sub) {
                                try {
                                    while (auto update = sub.pop()) {
                                        detail::ValueVariant value;
                                        auto pva_value_field = update["value"];
                                        switch (pva_value_field.type().code) {
                                        case pvxs::TypeCode::Float64:
                                            value = pva_value_field.as<double>();
                                            break;
                                        case pvxs::TypeCode::Float32:
                                            value = static_cast<double>(pva_value_field.as<float>());
                                            break;
                                        case pvxs::TypeCode::Int32:
                                            value = static_cast<int>(pva_value_field.as<int32_t>());
                                            break;
                                        case pvxs::TypeCode::Int16:
                                            value = static_cast<int>(pva_value_field.as<int16_t>());
                                            break;
                                        case pvxs::TypeCode::UInt16:
                                            value = static_cast<int>(pva_value_field.as<uint16_t>());
                                            break;
                                        case pvxs::TypeCode::Int8:
                                            value = static_cast<int>(pva_value_field.as<int8_t>());
                                            break;
                                        case pvxs::TypeCode::UInt8:
                                            value = static_cast<int>(pva_value_field.as<uint8_t>());
                                            break;
                                        case pvxs::TypeCode::String:
                                            value = pva_value_field.as<std::string>();
                                            break;
                                        default:
                                            continue;
                                        }
                                        std::lock_guard lock(mutex_);
                                        staged_value_ = value;
                                        new_data_.store(true, std::memory_order_release);
                                    }
                                } catch (pvxs::client::Connected&) {
                                    connected_.store(true, std::memory_order_relaxed);
                                } catch (pvxs::client::Finished&) {
                                    connected_.store(false, std::memory_order_relaxed);
                                } catch (pvxs::client::Disconnect&) {
                                    connected_.store(false, std::memory_order_relaxed);
                                }
                            })
                            .exec();
    }

    bool put(const detail::ValueVariant& value) override { return false; }
};

/// \brief Manages a collection of PV channels with a single sync() call.
///
/// Context is the primary interface for monitoring multiple PVs under a
/// single protocol (CA or PVA). For an application that supports both,
/// create two Context's, one for each protocol. Bind local variables with
/// bind(), then call sync() periodically to update all bound variables at
/// once. Channels are created automatically on first use.
///
/// Example usage:
/// \code
///     ezec::Context ctxt;
///
///     double position = 0.0;
///     int done = 0;
///     ctxt.bind(position, "MyIOC:m1.RBV");
///     ctxt.bind(done, "MyIOC:m1.DMOV");
///
///     while (true) {
///         if (ctxt.sync()) {
///             // At least one PV has new data
///             printf("pos=%f done=%d\n", position, done);
///         }
///         std::this_thread::sleep_for(std::chrono::milliseconds(100));
///     }
/// \endcode
class Context {
  public:
    Context(const std::string& protocol = "ca") : protocol_(protocol) {
        if (protocol_ == "ca") {
            SEVCHK(ca_context_create(ca_enable_preemptive_callback), "ca_context_create");
        } else if (protocol_ == "pva") {
            pvxs_ctxt_ = pvxs::client::Context::fromEnv();
        } else {
            throw std::runtime_error("Unknown protocol " + protocol);
        }
    }

    ~Context() {
        channel_map_.clear();
        if (protocol_ == "ca") {
            ca_context_destroy();
        } else if (protocol_ == "pva") {
        }
    }

    bool all_connected() const {
        for (auto& [_, pv] : channel_map_) {
            if (!pv->connected()) {
                return false;
            }
        }
        return true;
    }

    // TODO:
    // Context(const Context&) = delete;
    // Context& operator=(const Context&) = delete;

    /// \brief Bind a local variable to a PV.
    ///
    /// If the PV has not been seen before, a channel is created automatically.
    /// The bound variable must outlive the Context. Multiple variables
    /// (including different types) can be bound to the same PV.
    ///
    /// \param var  Reference to the local variable.
    /// \param pv_name  The PV name (e.g. "MyIOC:name.VAL").
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

    /// \brief Get a reference to a channel.
    ///
    /// If the PV has not been seen before, a channel is created automatically.
    /// The returned reference can be used to check connection status or
    /// access the underlying channel directly.
    ///
    /// \param pv_name  The PV name (e.g. "MyIOC:name.VAL").
    ChannelBase& get_channel(const std::string& pv_name) {
        std::lock_guard lock(mutex_);
        return get_channel_unlocked(pv_name);
    }

    /// \brief Shorthand for get_channel().
    /// \overload
    ChannelBase& operator[](const std::string& pv_name) { return get_channel(pv_name); }

    /// \brief Write a value to a PV.
    ///
    /// If the PV has not been seen before, a channel is created automatically.
    ///
    /// \param pv_name  The PV name (e.g. "MyIOC:name.VAL").
    /// \param value    The value to write.
    template<typename T>
    void put(const std::string& pv_name, const T& value) {
        get_channel(pv_name).put(value);
    }

    ChannelBase& add(const std::string& pv_name) {
        auto it = channel_map_.find(pv_name);
        if (it == channel_map_.end()) {
            // add the pv if if doens't exist yet
            if (protocol_ == "ca") {
                it = channel_map_.emplace(pv_name, std::make_unique<CAChannel>(pv_name)).first;
            } else if (protocol_ == "pva") {
                it = channel_map_.emplace(pv_name, std::make_unique<PVAChannel>(*pvxs_ctxt_, pv_name)).first;
            }
        }
        return *it->second;
    }

    void add(const std::vector<std::string>& pv_names) {
        for (const auto& pv_name : pv_names) {
            auto it = channel_map_.find(pv_name);
            if (it == channel_map_.end()) {
                // add the pv if if doens't exist yet
                if (protocol_ == "ca") {
                    it = channel_map_.emplace(pv_name, std::make_unique<CAChannel>(pv_name)).first;
                } else if (protocol_ == "pva") {
                    it = channel_map_.emplace(pv_name, std::make_unique<PVAChannel>(*pvxs_ctxt_, pv_name)).first;
                }
            }
        }
    }

    void add(const std::string& prefix, const std::vector<std::string>& pv_names) {
        for (std::string pv_name : pv_names) {
            pv_name = prefix + pv_name;
            auto it = channel_map_.find(pv_name);
            if (it == channel_map_.end()) {
                // add the pv if if doens't exist yet
                if (protocol_ == "ca") {
                    it = channel_map_.emplace(pv_name, std::make_unique<CAChannel>(pv_name)).first;
                } else if (protocol_ == "pva") {
                    it = channel_map_.emplace(pv_name, std::make_unique<PVAChannel>(*pvxs_ctxt_, pv_name)).first;
                }
            }
        }
    }

  private:
    const std::string protocol_ = "ca";
    std::mutex mutex_;
    std::unordered_map<std::string, std::unique_ptr<ChannelBase>> channel_map_;
    std::optional<pvxs::client::Context> pvxs_ctxt_;

    ChannelBase& get_channel_unlocked(const std::string& pv_name) {
        auto it = channel_map_.find(pv_name);
        if (it == channel_map_.end()) {
            // add the pv if if doens't exist yet
            if (protocol_ == "ca") {
                it = channel_map_.emplace(pv_name, std::make_unique<CAChannel>(pv_name)).first;
            } else if (protocol_ == "pva") {
                it = channel_map_.emplace(pv_name, std::make_unique<PVAChannel>(*pvxs_ctxt_, pv_name)).first;
            }
        }
        return *it->second;
    }
};

} // namespace ezec
