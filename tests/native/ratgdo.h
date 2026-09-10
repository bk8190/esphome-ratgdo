#pragma once

#include "callbacks.h"
#include "protocol.h"
#include "runtime.h"
#include "scheduler_ids.inc"
#include <cmath>

namespace esphome::ratgdo {
inline constexpr float DOOR_POSITION_UNKNOWN = -1.0f;
inline constexpr float DOOR_DELTA_UNKNOWN = -2.0f;
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
    single_observable<float> door_position { DOOR_POSITION_UNKNOWN };
    MotorState motor_state = MotorState::OFF;
    uint32_t door_start_moving = 0;
    float door_start_position = DOOR_POSITION_UNKNOWN;
    float door_move_delta = DOOR_DELTA_UNKNOWN;
    int start_opening = 0;
    int start_closing = 0;
    uint16_t position_sync_remaining_ = 0;
    optional<float> position_target_;
    DoorState position_target_direction_ = DoorState::UNKNOWN;
    bool position_target_tracking_ = false;
    void cancel_position_target_();
    void update_position_target_(DoorState state);
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
    template <typename F>
    void set_interval(uint32_t id, uint32_t period, F callback)
    {
        scheduler->set_interval(this, id, period, callback);
    }
    void cancel_interval(uint32_t id) { scheduler->intervals.erase({ this, id }); }
    template <typename T>
    void received(T) { }
    void received(DoorCommandStats value) { stats = value; }
    void received(DoorState value, uint32_t observed_at = 0);
    void door_move_to_position(float position);
    void schedule_door_position_sync(float update_period = 500);
    void door_position_update(uint32_t observed_at = 0);
    void cancel_position_sync_callbacks();
    void set_opening_duration(float value) { opening_duration = value; }
    void set_closing_duration(float value) { closing_duration = value; }
    void query_openings() { }
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
