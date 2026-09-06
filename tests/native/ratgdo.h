#pragma once

#include "callbacks.h"
#include "protocol.h"
#include "runtime.h"
#include "scheduler_ids.inc"

namespace esphome::ratgdo {
inline void log_once_callbacks_overflow(uint8_t) { throw std::runtime_error("callback overflow"); }
inline void log_multiple_subscribers() { throw std::runtime_error("multiple subscribers"); }
inline void log_observer_overflow() { throw std::runtime_error("observer overflow"); }

// The production command methods and callback-registration template are
// compiled into this lightweight component. Only publishing and peripherals
// are replaced, so commands still traverse the component and protocol layers.
class RATGDOComponent {
public:
    Scheduler* scheduler;
    protocol::Protocol* protocol_ = nullptr;
    single_observable<DoorState> door_state { DoorState::UNKNOWN };
    single_observable<float> opening_duration { 0 };
    single_observable<float> closing_duration { 0 };
    OnceCallbacks<void(DoorState)> on_door_state_;
    struct {
        bool obstruction_sensor_detected = true;
    } flags_;
    bool sync_failed = false;
    DoorCommandStats stats { };
    template <typename F>
    void set_timeout(uint32_t id, uint32_t delay, F callback)
    {
        scheduler->set_timeout(this, id, delay, callback);
    }
    template <typename F>
    void set_timeout(uint32_t delay, F callback)
    {
        set_timeout(10000, delay, callback);
    }
    void cancel_timeout(uint32_t id) { scheduler->cancel_timeout(this, id); }
    template <typename T>
    void received(T) { }
    void received(DoorCommandStats value) { stats = value; }
    void received(DoorState value)
    {
        if (*door_state != value) {
            door_state = value;
            on_door_state_.trigger(value);
        }
    }
    void query_status() { protocol_->call(protocol::QueryStatus { }); }
    void door_open();
    void door_close();
    void door_stop();
    void door_toggle();
    void door_action(DoorAction action);
    void set_door_state_expiry();
    void cancel_door_state_expiry();
#include "register_callback.inc"
};
}
