#pragma once

#ifdef PROTOCOL_SECPLUSV1

#include <queue>
#include <type_traits>
#include <utility>

#include "esphome/core/hal.h"
#include "esphome/core/optional.h"
#include "ratgdo_uart.h"

#include "callbacks.h"
#include "observable.h"
#include "protocol.h"
#include "ratgdo_state.h"

namespace esphome {

class Scheduler;
class InternalGPIOPin;

} // namespace esphome

namespace esphome::ratgdo {
namespace secplus1 {

    using namespace esphome::ratgdo::protocol;

    static const uint8_t RX_LENGTH = 2;
    typedef uint8_t RxPacket[RX_LENGTH];

    static const uint8_t TX_LENGTH = 2;
    typedef uint8_t TxPacket[TX_LENGTH];

    static const TxPacket toggle_door = { 0x30, 0x31 };
    static const TxPacket toggle_light = { 0x32, 0x33 };
    static const TxPacket toggle_lock = { 0x34, 0x35 };

    static const uint8_t secplus1_states[] PROGMEM = { 0x35, 0x35, 0x35, 0x35, 0x33, 0x33, 0x53, 0x53, 0x38, 0x3A, 0x3A, 0x3A, 0x39, 0x38, 0x3A, 0x38, 0x3A, 0x39, 0x3A };

    ENUM_SPARSE(CommandType, uint8_t,
        (TOGGLE_DOOR_PRESS, 0x30),
        (TOGGLE_DOOR_RELEASE, 0x31),
        (TOGGLE_LIGHT_PRESS, 0x32),
        (TOGGLE_LIGHT_RELEASE, 0x33),
        (TOGGLE_LOCK_PRESS, 0x34),
        (TOGGLE_LOCK_RELEASE, 0x35),
        (QUERY_DOOR_STATUS_0x37, 0x37),
        (QUERY_DOOR_STATUS, 0x38),
        (OBSTRUCTION, 0x39),
        (QUERY_OTHER_STATUS, 0x3A),
        (UNKNOWN, 0xFF), )

    struct RxCommand {
        CommandType req;
        uint8_t resp;

        RxCommand()
            : req(CommandType::UNKNOWN)
            , resp(0)
        {
        }
        RxCommand(CommandType req_)
            : req(req_)
            , resp(0)
        {
        }
        RxCommand(CommandType req_, uint8_t resp_ = 0)
            : req(req_)
            , resp(resp_)
        {
        }
    };

    struct TxCommand {
        CommandType request;
        // Cancel these queries together with their still-queued door press.
        bool door_warmup { false };
        uint32_t time;
    };

    struct FirstToSend {
        bool operator()(const TxCommand l, const TxCommand r) const { return l.time > r.time; }
    };

    enum class WallPanelEmulationState {
        WAITING,
        RUNNING,
    };

    class Secplus1 : public Protocol {
    public:
        void setup(RATGDOComponent* ratgdo, Scheduler* scheduler, InternalGPIOPin* rx_pin, InternalGPIOPin* tx_pin);
        void loop();
        void dump_config();
        void on_shutdown() override;

        void sync();

        void light_action(LightAction action);
        void lock_action(LockAction action);
        void door_action(DoorAction action);
        void set_door_state_expiry();
        void cancel_door_state_expiry();

        // See RATGDOComponent::on_door_state() in ratgdo.h for detailed
        // explanation of why the callback runs before the expiry check.
        template <typename F>
        void on_door_state(F&& callback)
        {
            using Cb = std::decay_t<F>;
            this->on_door_state_([this, cb = Cb(std::forward<F>(callback))](DoorState s) {
                cb(s);
                if (!this->on_door_state_.count()) {
                    this->cancel_door_state_expiry();
                }
            });
            this->set_door_state_expiry();
        }

        Result call(Args args);

        const Traits& traits() const { return this->traits_; }

        // methods not used by secplus1
        void set_open_limit(bool state) { }
        void set_close_limit(bool state) { }
        void set_discrete_open_pin(InternalGPIOPin* pin) { }
        void set_discrete_close_pin(InternalGPIOPin* pin) { }

    protected:
        void wall_panel_emulation(size_t index = 0);
        // Active door-status acquisition for panels that never poll door status:
        // periodically inject QUERY_DOOR_STATUS instead of only waiting to
        // overhear a passive poll that may never come.
        void door_status_probe_();
        void schedule_door_status_probe_(uint32_t delay);

        optional<RxCommand> read_command();
        void handle_command(const RxCommand& cmd);

        void print_rx_packet(const RxPacket& packet) const;
        void print_tx_packet(const TxPacket& packet) const;
        optional<RxCommand> decode_packet(const RxPacket& packet) const;

        void enqueue_transmit(CommandType cmd, uint32_t time = 0, bool door_warmup = false);
        void cancel_pending_door_presses_();
        optional<CommandType> pending_tx();
        optional<CommandType> pop_pending_tx();
        bool do_transmit_if_pending();
        void enqueue_command_pair(CommandType cmd);
        void transmit_byte(uint32_t value);

        void toggle_light();
        void toggle_lock();
        void toggle_door();
        void query_status();
        void clear_deferred_door_action_();
        void defer_door_action_(DoorAction action);
        void maybe_run_deferred_door_action_();
        bool door_action_satisfied_(DoorAction action, DoorState state) const;
        bool arm_door_command_watch_(DoorAction action, DoorState baseline_state);
        void mark_door_command_transmitted_();
        void schedule_door_command_watch_timeout_();
        // Late authoritative re-query after a door command, timed past travel, so
        // a transient/garbled mid-command read can't stick as the wrong end state.
        void schedule_post_command_verify_();
        // Must only be fed states that passed the two-confirmation gate; raw
        // single reads are noisy enough to fake a "confirmed" verdict.
        void observe_door_command_status_(DoorState state);
        void clear_door_command_watch_();
        void log_door_command_stats_(const char* outcome) const;
        void publish_door_command_stats_();

        // Pointers first (4-byte aligned)
        InternalGPIOPin* tx_pin_;
        InternalGPIOPin* rx_pin_;
        RATGDOComponent* ratgdo_;
        Scheduler* scheduler_;

        // 4-byte members
        uint32_t wall_panel_emulation_start_ { 0 };
        uint32_t last_rx_ { 0 };
        uint32_t last_tx_ { 0 };
        uint32_t last_status_query_ { 0 };
        // millis() of the last passive door-status poll observed on the bus (a
        // capable wall panel polling door position). 0 = never seen. Used to
        // suppress active probing while something else is supplying door status.
        uint32_t last_passive_door_status_ { 0 };
        uint32_t pending_door_action_since_ { 0 };
        uint32_t door_command_sent_at_ { 0 };
        uint32_t door_command_attempts_ { 0 };
        uint32_t door_command_misses_ { 0 };
        uint32_t door_command_retries_ { 0 };
        uint32_t door_command_retry_failures_ { 0 };
        uint32_t door_command_unverified_ { 0 };
        uint32_t door_command_duplicates_suppressed_ { 0 };

        // Larger structures
        std::priority_queue<TxCommand, std::vector<TxCommand>, FirstToSend> pending_tx_;
        RatgdoUART uart_;
        OnceCallbacks<void(DoorState)> on_door_state_;
        Traits traits_;

        // rx parsing state
        RxPacket rx_packet_;
        uint16_t rx_byte_count_ { 0 };

        // Group all 1-byte members at the end to minimize padding
        LightState light_state { LightState::UNKNOWN };
        LockState lock_state { LockState::UNKNOWN };
        DoorState door_state { DoorState::UNKNOWN };
        LightState maybe_light_state { LightState::UNKNOWN };
        LockState maybe_lock_state { LockState::UNKNOWN };
        DoorState maybe_door_state { DoorState::UNKNOWN };
        DoorAction pending_door_action_ { DoorAction::UNKNOWN };
        DoorAction watched_door_action_ { DoorAction::UNKNOWN };
        DoorState watched_door_baseline_ { DoorState::UNKNOWN };
        WallPanelEmulationState wall_panel_emulation_state_ { WallPanelEmulationState::WAITING };
        struct {
            uint8_t door_moving : 1;
            uint8_t wall_panel_starting : 1;
            uint8_t is_0x37_panel : 1;
            uint8_t rx_reading_msg : 1;
            uint8_t door_command_retried : 1;
            uint8_t door_command_baseline_reads : 2;
            uint8_t reserved : 1; // Reserved for future use
        } flags_ { 0 };
    };

} // namespace secplus1
} // namespace esphome::ratgdo

#endif // PROTOCOL_SECPLUSV1
