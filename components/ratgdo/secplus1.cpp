
#ifdef PROTOCOL_SECPLUSV1

#include "secplus1.h"
#include "ratgdo.h"

#include <cinttypes>

#include "esphome/core/gpio.h"
#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"
#include "esphome/core/scheduler.h"

// Opt-in packet/state/queue tracing; disabled builds omit its argument evaluation.
#ifndef RATGDO_SECPLUS1_DIAGNOSTICS
#define RATGDO_SECPLUS1_DIAGNOSTICS 0
#endif

// Opt-in: send status queries with the checked bit-level sender in
// secplus1_query_tx.h, which cannot put a panel command on the bus.
#ifndef RATGDO_SECPLUS1_CHECKED_QUERY_TX
#define RATGDO_SECPLUS1_CHECKED_QUERY_TX 0
#endif
#if RATGDO_SECPLUS1_CHECKED_QUERY_TX && defined(USE_ESP32)
#error "RATGDO_SECPLUS1_CHECKED_QUERY_TX needs the bit-banged ESP8266 UART"
#endif
// How often a settled door is re-checked when no panel supplies its status.
#ifndef RATGDO_SECPLUS1_IDLE_PROBE_MS
#define RATGDO_SECPLUS1_IDLE_PROBE_MS 300000
#endif
// Supervised hardware test only: the Query status button sends a burst of
// deliberately abandoned queries, to confirm the opener ignores them.
#ifndef RATGDO_SECPLUS1_ABORT_TEST
#define RATGDO_SECPLUS1_ABORT_TEST 0
#endif
// A real wall panel is known to be on the bus: never start emulating one,
// even if the opener stays silent after boot. Emulating beside a real panel
// puts two masters on the bus.
#ifndef RATGDO_SECPLUS1_NO_PANEL_EMULATION
#define RATGDO_SECPLUS1_NO_PANEL_EMULATION 0
#endif
#if RATGDO_SECPLUS1_ABORT_TEST && !RATGDO_SECPLUS1_CHECKED_QUERY_TX
#error "RATGDO_SECPLUS1_ABORT_TEST needs RATGDO_SECPLUS1_CHECKED_QUERY_TX"
#endif

namespace esphome::ratgdo {
namespace secplus1 {

    using namespace scheduler_ids;
    static const char* const TAG = "ratgdo_secplus1";
    // Basic panels need two actively requested replies to confirm a transition.
    static constexpr uint32_t DOOR_STATE_CALLBACK_TIMEOUT = 12000;
    static constexpr uint32_t DOOR_STATUS_FAST_INTERVAL = 2500;
    static constexpr uint32_t DOOR_ACTION_DEFER_TIMEOUT = 22000;
    static constexpr uint32_t DOOR_ACTION_POST_TOGGLE_QUERY_DELAY = 1200;
    // Pre-toggle bus warm-up: wake the opener's receiver with a short
    // QUERY_DOOR_STATUS burst before the button press so a single press lands
    // instead of being dropped into an idle bus.
    static constexpr uint8_t DOOR_PRESS_WARMUP_QUERIES = 3;
    static constexpr uint32_t DOOR_PRESS_WARMUP_INTERVAL = 200; // ms between warm-up queries and before the press
    // The opener periodically stops answering for a few seconds, and a press
    // sent then is ignored: field logs showed 29/33 first presses missed after
    // 2+ unanswered warm-up queries, versus 0/36 otherwise. Hold the press until
    // this many consecutive valid replies arrive, re-querying while silent, and
    // press anyway at the deadline (below the 12s watch/callback timeouts).
    static constexpr uint8_t DOOR_PRESS_GATE_GOOD_REPLIES = 2;
    static constexpr uint32_t DOOR_PRESS_GATE_RECHECK = 250;
    static constexpr uint32_t DOOR_PRESS_GATE_DEADLINE = 10000;
    static constexpr uint8_t DOOR_COMMAND_BASELINE_CONFIRMATIONS = 2;
    static constexpr uint32_t DOOR_COMMAND_RETRY_MIN_DELAY = 3000;
    // Watch verdicts consume only two-read-confirmed states. Noise that decodes
    // as a different valid state resets the gate (costing extra reads at the 2.5s probe
    // cadence), so the window must fit 2 confirmed baseline reads plus one
    // noise-flip recovery after the ~0.6s warm-up burst.
    static constexpr uint32_t DOOR_COMMAND_WATCH_TIMEOUT = 12000;
    // After a door toggle transmits, re-query the opener once the door should
    // have finished travelling and authoritatively re-confirm the settled end
    // state. The post-toggle queries at ~1.2s/3.7s only catch the door mid-
    // travel; without a late verify a transient/garbled read committed during a
    // messy command (e.g. a mid-motion reversal) can stick as the wrong end
    // state. Delay covers typical residential door travel with margin; the
    // verify enqueues two queries so the two-confirmation gate commits truth.
    static constexpr uint32_t DOOR_COMMAND_VERIFY_DELAY = 22000;
    static constexpr uint32_t DOOR_COMMAND_VERIFY_SECOND_QUERY = 2500;
    // If no passive door-status poll has been observed for this long, assume no
    // capable panel is supplying door status and resume active probing. 3x the
    // 0x37 panel's ~10s poll interval, so an occasional missed poll won't trip it.
    static constexpr uint32_t DOOR_STATUS_PASSIVE_TIMEOUT = 30000;
    // Every query is a byte on a shared bus that a collision or timing glitch
    // can turn into another command: field logs show about 1 in 40,000-100,000
    // 0x38 queries reading back as 0x30, a door press, with the door then
    // moving. Exposure scales with the query count, so a settled door is only
    // re-checked this often (it was 10s). This bounds how long an operation
    // ratgdo cannot see (RF remote, mechanical) stays unreported. The checked
    // query sender removes that exposure and can use a short interval again.
    static constexpr uint32_t DOOR_STATUS_IDLE_INTERVAL = RATGDO_SECPLUS1_IDLE_PROBE_MS;
    // An aborted checked query is sent again once, this much later. The bus
    // carries interfering pulses about every 102 ms, often several periods
    // running, so the retry aims for the middle of a period (2.5 periods).
    static constexpr uint32_t QUERY_RETRY_DELAY = 256;
    static constexpr uint8_t ABORT_TEST_FRAMES = 30;
    static constexpr uint32_t ABORT_TEST_INTERVAL = 300;
    // After a hint that the door may be about to move (a wall-panel press, or
    // one of our queries read back as a press), probe fast for this long so
    // the movement is reported promptly despite the slow idle rate.
    static constexpr uint32_t DOOR_STATUS_SUSPECT_WINDOW = 15000;
    // Our own byte is read back within a few ms of sending it.
    static constexpr uint32_t TX_ECHO_TIMEOUT = 100;

    void Secplus1::setup(RATGDOComponent* ratgdo, Scheduler* scheduler, InternalGPIOPin* rx_pin, InternalGPIOPin* tx_pin)
    {
        this->ratgdo_ = ratgdo;
        this->scheduler_ = scheduler;
        this->tx_pin_ = tx_pin;
        this->rx_pin_ = rx_pin;

        this->uart_.begin(1200, RATGDO_UART_8E1, rx_pin->get_pin(), tx_pin->get_pin(), true);

        this->traits_.set_features(HAS_DOOR_STATUS | HAS_LIGHT_TOGGLE | HAS_LOCK_TOGGLE);
    }

    void Secplus1::loop()
    {
        auto rx_cmd = this->read_command();
        if (rx_cmd) {
            this->handle_command(rx_cmd.value());
        }
        auto tx_cmd = this->pending_tx();
        if (
            (millis() - this->last_tx_) > 200 && // don't send twice in a period
            (millis() - this->last_rx_) > 50 && // time to send it
            tx_cmd && // have pending command
            !(this->flags_.is_0x37_panel && tx_cmd.value() == CommandType::TOGGLE_LOCK_PRESS) && this->wall_panel_emulation_state_ != WallPanelEmulationState::RUNNING) {
            this->do_transmit_if_pending();
        }
    }

    void Secplus1::dump_config()
    {
        ESP_LOGCONFIG(TAG, "  Protocol: SEC+ v1");
#if RATGDO_SECPLUS1_DIAGNOSTICS
        ESP_LOGCONFIG(TAG, "  Packet diagnostics: enabled");
#else
        ESP_LOGCONFIG(TAG, "  Packet diagnostics: disabled");
#endif
    }

    void Secplus1::on_shutdown()
    {
        this->uart_.on_shutdown();
    }

    void Secplus1::sync()
    {
        this->wall_panel_emulation_state_ = WallPanelEmulationState::WAITING;
        this->wall_panel_emulation_start_ = millis();
        this->flags_.wall_panel_starting = false;
        this->cancel_pending_door_presses_();
        this->on_door_state_.clear();
        this->cancel_door_state_expiry();
        this->clear_deferred_door_action_();
        this->clear_door_command_watch_();
        this->door_state = DoorState::UNKNOWN;
        this->ratgdo_->cancel_position_target_();
        this->maybe_door_state = DoorState::UNKNOWN;
        this->maybe_door_state_since_ = 0;
        this->light_state = LightState::UNKNOWN;
        this->scheduler_->cancel_timeout(this->ratgdo_, TIMEOUT_WALL_PANEL_EMULATION);
        this->scheduler_->cancel_timeout(this->ratgdo_, TIMEOUT_DOOR_COMMAND_VERIFY);
        this->wall_panel_emulation();

        this->ratgdo_->set_timeout(45000, [this] {
            if (this->door_state == DoorState::UNKNOWN) {
                ESP_LOGW(TAG, "Triggering sync failed actions.");
                this->ratgdo_->sync_failed = true;
            }
        });

        // Kick off active door-status probing while we don't yet know the door
        // position. Non-0x37 wall panels never poll door status, so passively
        // waiting can leave door_state UNKNOWN forever after a reboot. Short
        // delay lets boot-time bus traffic settle before we transmit.
        this->schedule_door_status_probe_(5000);
    }

    void Secplus1::schedule_door_status_probe_(uint32_t delay)
    {
        this->scheduler_->set_timeout(this->ratgdo_, TIMEOUT_DOOR_PROBE, delay, [this] {
            this->door_status_probe_();
        });
    }

    void Secplus1::door_status_probe_()
    {
        // Probe only when nothing else is supplying door status. If a capable
        // panel has polled door position recently (last_passive_door_status_),
        // ratgdo already has a live source of truth and stays dormant. This is
        // determined entirely from observed bus traffic — no configuration — and
        // self-corrects: if passive polls stop arriving, probing resumes.
        const bool passive_source_present = this->last_passive_door_status_ != 0
            && (millis() - this->last_passive_door_status_) < DOOR_STATUS_PASSIVE_TIMEOUT;

        // Basic wall panels never poll door status, so ratgdo has no passive
        // source: such an install is otherwise dead-reckoning from its own
        // commands and drifts stale on reboots and out-of-band operations
        // (wall-panel button, RF remote, mechanical). Actively query the opener to
        // keep state grounded in reality. enqueue_transmit + loop() handle bus-gap
        // timing the same way toggle_door and the 0x37 path already do.
        if (!passive_source_present
            && this->wall_panel_emulation_state_ != WallPanelEmulationState::RUNNING
            && this->pending_tx_.empty()) {
            ESP_LOGD(TAG, "Probing door status, door state: %s", LOG_STR_ARG(DoorState_to_string(this->door_state)));
            this->enqueue_transmit(CommandType::QUERY_DOOR_STATUS);
        }
        // Probe fast while we don't know the state, the door is moving, or a
        // correction is pending — an unconfirmed candidate (maybe_door_state)
        // disagrees with the currently held state, so a second confirming read
        // lands quickly and overturns a stale/wrong commit in ~2.5s instead of a
        // full background cycle (quick post-reboot heal and quick settle). Then
        // drop to a low background rate to bound staleness. The loop is kept
        // alive even when a passive source is present (cheap timer + compare) so
        // probing can resume if it disappears. Self-sustaining single timer
        // (same scheduler key replaces the prior one).
        const bool correction_pending = this->maybe_door_state != this->door_state;
        const bool watching_command = this->watched_door_action_ != DoorAction::UNKNOWN;
        if (this->door_suspect_ && millis() - this->door_suspect_since_ >= DOOR_STATUS_SUSPECT_WINDOW) {
            this->door_suspect_ = false;
        }
        uint32_t next_ms = (this->door_state == DoorState::UNKNOWN || this->flags_.door_moving || correction_pending || watching_command || this->door_suspect_)
            ? DOOR_STATUS_FAST_INTERVAL
            : DOOR_STATUS_IDLE_INTERVAL;
        this->schedule_door_status_probe_(next_ms);
    }

    void Secplus1::note_door_suspect_()
    {
        this->door_suspect_ = true;
        this->door_suspect_since_ = millis();
        this->schedule_door_status_probe_(DOOR_STATUS_FAST_INTERVAL);
    }

    void Secplus1::wall_panel_emulation(size_t index)
    {
        if (this->flags_.wall_panel_starting) {
            this->wall_panel_emulation_state_ = WallPanelEmulationState::WAITING;
        } else if (this->wall_panel_emulation_state_ == WallPanelEmulationState::WAITING) {
            ESP_LOGD(TAG, "Looking for security+ 1.0 wall panel...");

            if (this->door_state != DoorState::UNKNOWN || this->light_state != LightState::UNKNOWN) {
                ESP_LOG1(TAG, "Wall panel detected");
                return;
            }
            if (!RATGDO_SECPLUS1_NO_PANEL_EMULATION && millis() - this->wall_panel_emulation_start_ > 35000 && !this->flags_.wall_panel_starting) {
                ESP_LOGD(TAG, "No wall panel detected. Switching to emulation mode.");
                this->wall_panel_emulation_state_ = WallPanelEmulationState::RUNNING;
            }
            this->scheduler_->set_timeout(this->ratgdo_, TIMEOUT_WALL_PANEL_EMULATION, 2000, [this] {
                this->wall_panel_emulation();
            });
            return;
        } else if (this->wall_panel_emulation_state_ == WallPanelEmulationState::RUNNING) {
#ifdef USE_ESP8266
            // ESP_LOG2(TAG, "[Wall panel emulation] Sending byte: [%02X]", progmem_read_byte(&secplus1_states[index]));
#else
            // ESP_LOG2(TAG, "[Wall panel emulation] Sending byte: [%02X]", secplus1_states[index]);
#endif

            if (index < 15 || !this->do_transmit_if_pending()) {
#ifdef USE_ESP8266
                this->transmit_byte(progmem_read_byte(&secplus1_states[index]));
#else
                this->transmit_byte(secplus1_states[index]);
#endif
                // gdo response simulation for testing
#ifdef USE_ESP8266
                // auto resp = progmem_read_byte(&secplus1_states[index]) == 0x39 ? 0x00 :
                //             progmem_read_byte(&secplus1_states[index]) == 0x3A ? 0x5C :
                //             progmem_read_byte(&secplus1_states[index]) == 0x38 ? 0x52 : 0xFF;
#else
                // auto resp = secplus1_states[index] == 0x39 ? 0x00 :
                //             secplus1_states[index] == 0x3A ? 0x5C :
                //             secplus1_states[index] == 0x38 ? 0x52 : 0xFF;
#endif
                // if (resp != 0xFF) {
                //     this->transmit_byte(resp, true);
                // }

                index += 1;
                if (index == 18) {
                    index = 15;
                }
            }
            this->scheduler_->set_timeout(this->ratgdo_, TIMEOUT_WALL_PANEL_EMULATION, 250, [this, index] {
                this->wall_panel_emulation(index);
            });
        }
    }

    void Secplus1::light_action(LightAction action)
    {
        ESP_LOG1(TAG, "Light action: %s", LOG_STR_ARG(LightAction_to_string(action)));
        if (action == LightAction::UNKNOWN) {
            return;
        }
        if (
            action == LightAction::TOGGLE || (action == LightAction::ON && this->light_state == LightState::OFF) || (action == LightAction::OFF && this->light_state == LightState::ON)) {
            this->toggle_light();
        }
    }

    void Secplus1::lock_action(LockAction action)
    {
        ESP_LOG1(TAG, "Lock action: %s", LOG_STR_ARG(LockAction_to_string(action)));
        if (action == LockAction::UNKNOWN) {
            return;
        }
        if (
            action == LockAction::TOGGLE || (action == LockAction::LOCK && this->lock_state == LockState::UNLOCKED) || (action == LockAction::UNLOCK && this->lock_state == LockState::LOCKED)) {
            this->toggle_lock();
        }
    }

    void Secplus1::set_door_state_expiry()
    {
        this->scheduler_->set_timeout(this->ratgdo_, TIMEOUT_SECPLUS1_DOOR_STATE_EXPIRY, DOOR_STATE_CALLBACK_TIMEOUT, [this]() {
            ESP_LOGW(TAG, "Door state callback expired, clearing");
            this->on_door_state_.clear();
            this->cancel_pending_door_presses_();
        });
    }

    void Secplus1::cancel_door_state_expiry()
    {
        this->scheduler_->cancel_timeout(this->ratgdo_, TIMEOUT_SECPLUS1_DOOR_STATE_EXPIRY);
    }

    void Secplus1::clear_deferred_door_action_()
    {
        this->scheduler_->cancel_timeout(this->ratgdo_, TIMEOUT_DEFERRED_DOOR_ACTION);
        this->pending_door_action_ = DoorAction::UNKNOWN;
        this->pending_door_action_since_ = 0;
    }

    bool Secplus1::door_action_satisfied_(DoorAction action, DoorState state) const
    {
        if (action == DoorAction::OPEN) {
            return state == DoorState::OPEN || state == DoorState::OPENING;
        }
        if (action == DoorAction::CLOSE) {
            return state == DoorState::CLOSED || state == DoorState::CLOSING;
        }
        return false;
    }

    void Secplus1::log_door_command_stats_(const char* outcome) const
    {
        ESP_LOGI(TAG,
            "Door command recovery %s: attempts=%" PRIu32 " misses=%" PRIu32 " retries=%" PRIu32 " retry_failures=%" PRIu32 " unverified=%" PRIu32 " duplicates_suppressed=%" PRIu32,
            outcome,
            this->door_command_attempts_,
            this->door_command_misses_,
            this->door_command_retries_,
            this->door_command_retry_failures_,
            this->door_command_unverified_,
            this->door_command_duplicates_suppressed_);
    }

    void Secplus1::publish_door_command_stats_()
    {
        this->ratgdo_->received(DoorCommandStats {
            this->door_command_retries_,
            this->door_command_retry_failures_,
            this->door_command_unverified_,
            this->door_command_duplicates_suppressed_,
            this->tx_echo_errors_,
            this->tx_query_aborts_ });
    }

    void Secplus1::clear_door_command_watch_()
    {
        this->scheduler_->cancel_timeout(this->ratgdo_, TIMEOUT_DOOR_COMMAND_WATCH);
        this->watched_door_action_ = DoorAction::UNKNOWN;
        this->watched_door_baseline_ = DoorState::UNKNOWN;
        this->door_command_sent_at_ = 0;
        this->flags_.door_command_retried = false;
        this->flags_.door_command_baseline_reads = 0;
    }

    bool Secplus1::arm_door_command_watch_(DoorAction action, DoorState baseline_state)
    {
        // Recovery is only safe for an idempotent directional request from a
        // known end stop. A raw TOGGLE or a command while stopped/moving has no
        // safe automatic retry direction.
        if (this->flags_.is_0x37_panel
            || !((action == DoorAction::OPEN && baseline_state == DoorState::CLOSED)
                || (action == DoorAction::CLOSE && baseline_state == DoorState::OPEN))) {
            return true;
        }

        if (this->watched_door_action_ == action) {
            this->door_command_duplicates_suppressed_++;
            this->log_door_command_stats_("duplicate suppressed");
            this->publish_door_command_stats_();
            return false;
        }

        this->clear_door_command_watch_();
        this->watched_door_action_ = action;
        this->watched_door_baseline_ = baseline_state;
        this->door_command_attempts_++;
        // Failsafe scheduled at arm time, not transmit time: a press that never
        // makes it onto the bus would otherwise leave the watch armed forever,
        // duplicate-suppressing every later command in the same direction.
        // mark_door_command_transmitted_ re-schedules from actual transmit.
        this->schedule_door_command_watch_timeout_();
        this->schedule_door_status_probe_(DOOR_STATUS_FAST_INTERVAL);
        return true;
    }

    void Secplus1::schedule_post_command_verify_()
    {
        // Fire once, after expected travel, regardless of what state the command
        // sequence committed in the meantime. Two queries a couple seconds apart
        // feed the two-confirmation gate so a genuine settled end state commits
        // (and overturns any transient/garbled mid-command commit). Self-
        // replacing timer: repeated presses (retry, STOP direction sequences)
        // just push the single verify out to after the last press.
        this->scheduler_->set_timeout(this->ratgdo_, TIMEOUT_DOOR_COMMAND_VERIFY, DOOR_COMMAND_VERIFY_DELAY, [this]() {
            ESP_LOGD(TAG, "Post-command verify: re-querying door end state (held %s)", LOG_STR_ARG(DoorState_to_string(this->door_state)));
            auto now = millis();
            this->enqueue_transmit(CommandType::QUERY_DOOR_STATUS, now);
            this->enqueue_transmit(CommandType::QUERY_DOOR_STATUS, now + DOOR_COMMAND_VERIFY_SECOND_QUERY);
        });
    }

    void Secplus1::schedule_door_command_watch_timeout_()
    {
        this->scheduler_->set_timeout(this->ratgdo_, TIMEOUT_DOOR_COMMAND_WATCH, DOOR_COMMAND_WATCH_TIMEOUT, [this]() {
            if (this->watched_door_action_ == DoorAction::UNKNOWN) {
                return;
            }
            this->door_command_unverified_++;
            this->log_door_command_stats_("inconclusive");
            this->cancel_pending_door_presses_();
            this->clear_door_command_watch_();
            this->publish_door_command_stats_();
        });
    }

    void Secplus1::mark_door_command_transmitted_()
    {
        if (this->watched_door_action_ == DoorAction::UNKNOWN || this->door_command_sent_at_ != 0) {
            return;
        }

        this->door_command_sent_at_ = millis();
        this->flags_.door_command_baseline_reads = 0;
        this->schedule_door_command_watch_timeout_();
        this->schedule_door_status_probe_(DOOR_STATUS_FAST_INTERVAL);
    }

    void Secplus1::observe_door_command_status_(DoorState state)
    {
        if (this->watched_door_action_ == DoorAction::UNKNOWN || state == DoorState::UNKNOWN) {
            return;
        }

        if (state != this->watched_door_baseline_) {
            this->log_door_command_stats_(this->flags_.door_command_retried ? "recovered" : "confirmed");
            // A manual command or delayed actuation can start motion during
            // warm-up. Cancel the queued press before it can stop that motion.
            this->cancel_pending_door_presses_();
            this->clear_door_command_watch_();
            return;
        }

        if (this->door_command_sent_at_ == 0) {
            return; // Warm-up baseline replies cannot establish a missed press.
        }

        if (this->flags_.door_command_baseline_reads < DOOR_COMMAND_BASELINE_CONFIRMATIONS) {
            this->flags_.door_command_baseline_reads++;
        }
        if (this->flags_.door_command_baseline_reads < DOOR_COMMAND_BASELINE_CONFIRMATIONS
            || (millis() - this->door_command_sent_at_) < DOOR_COMMAND_RETRY_MIN_DELAY) {
            return;
        }

        if (this->flags_.door_command_retried) {
            this->door_command_retry_failures_++;
            this->log_door_command_stats_("retry failed");
            this->clear_door_command_watch_();
            this->publish_door_command_stats_();
            return;
        }

        // Never stack recovery traffic on another queued action. Without two
        // fresh, valid baseline replies and an empty queue, the result is
        // inconclusive rather than permission to transmit another toggle.
        if (!this->pending_tx_.empty() || this->wall_panel_emulation_state_ == WallPanelEmulationState::RUNNING) {
            this->door_command_unverified_++;
            this->log_door_command_stats_("retry blocked");
            this->clear_door_command_watch_();
            this->publish_door_command_stats_();
            return;
        }

        this->door_command_misses_++;
        this->door_command_retries_++;
        this->publish_door_command_stats_();
        this->flags_.door_command_retried = true;
        this->door_command_sent_at_ = 0;
        this->flags_.door_command_baseline_reads = 0;
        this->log_door_command_stats_("miss confirmed; retrying once");
        this->toggle_door();
    }

    void Secplus1::defer_door_action_(DoorAction action)
    {
        this->pending_door_action_ = action;
        this->pending_door_action_since_ = millis();

        ESP_LOGI(TAG, "Door state unknown, deferring action %s until status is confirmed", LOG_STR_ARG(DoorAction_to_string(action)));

        this->scheduler_->set_timeout(this->ratgdo_, TIMEOUT_DEFERRED_DOOR_ACTION, DOOR_ACTION_DEFER_TIMEOUT, [this]() {
            if (this->pending_door_action_ == DoorAction::UNKNOWN) {
                return;
            }
            ESP_LOGW(TAG, "Deferred door action %s expired before door status was confirmed", LOG_STR_ARG(DoorAction_to_string(this->pending_door_action_)));
            this->pending_door_action_ = DoorAction::UNKNOWN;
            this->pending_door_action_since_ = 0;
        });

        if (this->pending_tx_.empty() && this->wall_panel_emulation_state_ != WallPanelEmulationState::RUNNING) {
            this->enqueue_transmit(CommandType::QUERY_DOOR_STATUS);
        }
    }

    void Secplus1::maybe_run_deferred_door_action_()
    {
        if (this->pending_door_action_ == DoorAction::UNKNOWN) {
            return;
        }

        const uint32_t pending_age = millis() - this->pending_door_action_since_;
        const auto action = this->pending_door_action_;
        if (pending_age >= DOOR_ACTION_DEFER_TIMEOUT) {
            ESP_LOGW(TAG, "Deferred door action %s expired before door status was confirmed", LOG_STR_ARG(DoorAction_to_string(action)));
            this->clear_deferred_door_action_();
            return;
        }

        if (this->door_state == DoorState::UNKNOWN) {
            return;
        }

        if (this->door_action_satisfied_(action, this->door_state)) {
            ESP_LOG1(TAG, "Deferred door action %s already satisfied by door state %s", LOG_STR_ARG(DoorAction_to_string(action)), LOG_STR_ARG(DoorState_to_string(this->door_state)));
            this->clear_deferred_door_action_();
            return;
        }

        const bool can_run_action = (action == DoorAction::OPEN
                                        && (this->door_state == DoorState::CLOSED || this->door_state == DoorState::CLOSING || this->door_state == DoorState::STOPPED))
            || (action == DoorAction::CLOSE
                && (this->door_state == DoorState::OPEN || this->door_state == DoorState::OPENING || this->door_state == DoorState::STOPPED));
        if (!can_run_action) {
            ESP_LOGW(TAG, "Dropping deferred door action %s, door state is %s", LOG_STR_ARG(DoorAction_to_string(action)), LOG_STR_ARG(DoorState_to_string(this->door_state)));
            this->clear_deferred_door_action_();
            return;
        }

        ESP_LOGI(TAG, "Running deferred door action %s, door state is %s", LOG_STR_ARG(DoorAction_to_string(action)), LOG_STR_ARG(DoorState_to_string(this->door_state)));
        this->clear_deferred_door_action_();
        this->door_action(action);
    }

    void Secplus1::cancel_door_action()
    {
        this->cancel_pending_door_presses_();
        this->clear_door_command_watch_();
        this->clear_deferred_door_action_();
        this->on_door_state_.clear();
        this->cancel_door_state_expiry();
    }

    void Secplus1::door_action(DoorAction action)
    {
#if RATGDO_SECPLUS1_DIAGNOSTICS
        ESP_LOGD(TAG, "[%" PRIu32 "] Door request=%s held=%s candidate=%s watch=%s queued=%u",
            millis(), LOG_STR_ARG(DoorAction_to_string(action)), LOG_STR_ARG(DoorState_to_string(this->door_state)),
            LOG_STR_ARG(DoorState_to_string(this->maybe_door_state)), LOG_STR_ARG(DoorAction_to_string(this->watched_door_action_)),
            static_cast<unsigned>(this->pending_tx_.size()));
#else
        ESP_LOG1(TAG, "Door action: %s, door state: %s", LOG_STR_ARG(DoorAction_to_string(action)), LOG_STR_ARG(DoorState_to_string(this->door_state)));
#endif
        if (action == DoorAction::UNKNOWN) {
            return;
        }

        if (action == this->watched_door_action_) {
            this->door_command_duplicates_suppressed_++;
            this->log_door_command_stats_("duplicate suppressed");
            this->publish_door_command_stats_();
            return;
        }

        // A new intent supersedes an earlier queued press, retry, deferred
        // action, or direction sequence, even when already satisfied by state.
        // Preserve release bytes: a press already on the bus must be released.
        this->cancel_door_action();

        if ((action == DoorAction::OPEN || action == DoorAction::CLOSE) && this->door_state == DoorState::UNKNOWN) {
            this->defer_door_action_(action);
            return;
        }

        if (action == DoorAction::TOGGLE) {
            this->toggle_door();
        } else if (action == DoorAction::OPEN) {
            if (this->door_state == DoorState::CLOSED || this->door_state == DoorState::CLOSING) {
                if (!this->arm_door_command_watch_(action, this->door_state)) {
                    return;
                }
                this->toggle_door();
            } else if (this->door_state == DoorState::STOPPED) {
                this->toggle_door(); // this starts closing door
                // When the outer callback fires and registers an inner callback,
                // on_door_state() replaces the outer's expiry with a new one for
                // the inner callback (same timeout ID = replace, not add).
                this->on_door_state([this](DoorState s) {
                    if (s == DoorState::CLOSING) {
                        // this changes direction of the door on some openers, on others it stops it
                        this->toggle_door();
                        this->on_door_state([this](DoorState s) {
                            if (s == DoorState::STOPPED) {
                                this->toggle_door();
                            }
                        });
                    }
                });
            }
        } else if (action == DoorAction::CLOSE) {
            if (this->door_state == DoorState::OPEN) {
                if (!this->arm_door_command_watch_(action, this->door_state)) {
                    return;
                }
                this->toggle_door();
            } else if (this->door_state == DoorState::OPENING) {
                this->toggle_door(); // this switches to stopped
                // another toggle needed to close
                this->on_door_state([this](DoorState s) {
                    if (s == DoorState::STOPPED) {
                        this->toggle_door();
                    }
                });
            } else if (this->door_state == DoorState::STOPPED) {
#if RATGDO_SECPLUS1_DIAGNOSTICS
                ESP_LOGD(TAG, "[%" PRIu32 "] CLOSE from STOPPED: single press; recovery watch not armed", millis());
#endif
                this->toggle_door();
            }
        } else if (action == DoorAction::STOP) {
            if (this->door_state == DoorState::OPENING) {
                this->toggle_door();
            } else if (this->door_state == DoorState::CLOSING) {
                this->toggle_door(); // this switches to opening

                // another toggle needed to stop
                this->on_door_state([this](DoorState s) {
                    if (s == DoorState::OPENING) {
                        this->toggle_door();
                    }
                });
            }
        }
    }

    void Secplus1::toggle_light()
    {
        this->enqueue_transmit(CommandType::TOGGLE_LIGHT_PRESS);
    }

    void Secplus1::toggle_lock()
    {
        this->enqueue_transmit(CommandType::TOGGLE_LOCK_PRESS);
    }

    void Secplus1::toggle_door()
    {
        // Pre-toggle bus warm-up: emit a short QUERY_DOOR_STATUS burst to wake the
        // opener's receiver, then the single button press. On a basic panel the bus
        // is otherwise mostly idle and a press into silence is sometimes dropped by
        // the opener. The press is scheduled after the burst; the priority queue
        // orders by time so the warm-up queries always transmit first. Release +
        // post-toggle queries are scheduled by enqueue_command_pair when the press
        // actually transmits.
        //
        // Skip the warm-up when the door is already moving (OPENING/CLOSING): the
        // bus is already warm from motion traffic, and the corrective toggles of
        // the Sec+1 single-button direction sequence (see door_action STOP /
        // OPEN-from-STOPPED) must land promptly to minimize overshoot.
        //
        // The warm-up replies also show whether the opener is listening. The
        // press waits behind the press gate until they are healthy.
        auto now = millis();
        // A newer toggle supersedes a press still waiting at the gate.
        this->clear_door_press_gate_();
        if (this->door_state != DoorState::OPENING && this->door_state != DoorState::CLOSING) {
            for (uint8_t i = 0; i < DOOR_PRESS_WARMUP_QUERIES; i++) {
                this->enqueue_transmit(CommandType::QUERY_DOOR_STATUS, now + i * DOOR_PRESS_WARMUP_INTERVAL, true);
            }
            this->start_door_press_gate_(now + DOOR_PRESS_WARMUP_QUERIES * DOOR_PRESS_WARMUP_INTERVAL);
        } else {
            this->enqueue_transmit(CommandType::TOGGLE_DOOR_PRESS, now);
        }
        if (this->door_state == DoorState::STOPPED || this->door_state == DoorState::OPEN || this->door_state == DoorState::CLOSED) {
            this->flags_.door_moving = true;
        }
    }

    void Secplus1::start_door_press_gate_(uint32_t first_check)
    {
        this->door_press_gate_active_ = true;
        this->door_press_gate_awaiting_ = false;
        this->door_press_gate_good_ = 0;
        this->door_press_gate_started_ = millis();
        // The press can now wait up to the gate deadline. Restart a watched
        // command's failsafe (including a retry's) so it outlasts the gate.
        if (this->watched_door_action_ != DoorAction::UNKNOWN) {
            this->schedule_door_command_watch_timeout_();
        }
        this->scheduler_->set_timeout(this->ratgdo_, TIMEOUT_DOOR_PRESS_GATE, first_check - millis(), [this]() {
            this->check_door_press_gate_();
        });
    }

    void Secplus1::check_door_press_gate_()
    {
        if (!this->door_press_gate_active_) {
            return;
        }
        if (this->cancel_unsafe_retry_press_()) {
            return;
        }
        const auto waited = millis() - this->door_press_gate_started_;
        const bool healthy = this->door_press_gate_good_ >= DOOR_PRESS_GATE_GOOD_REPLIES;
        if (healthy || waited >= DOOR_PRESS_GATE_DEADLINE) {
            if (!healthy) {
                ESP_LOGW(TAG, "Opener not answering status queries after %" PRIu32 "ms; pressing anyway", waited);
            } else if (waited > DOOR_PRESS_WARMUP_QUERIES * DOOR_PRESS_WARMUP_INTERVAL + DOOR_PRESS_GATE_RECHECK) {
                ESP_LOGI(TAG, "Door press held %" PRIu32 "ms until the opener answered", waited);
            }
            this->clear_door_press_gate_();
            this->enqueue_transmit(CommandType::TOGGLE_DOOR_PRESS);
            // A direction sequence waits for the result of this press. Time its
            // callback from the press, not from the original request.
            if (this->on_door_state_.count()) {
                this->set_door_state_expiry();
            }
            return;
        }
        // Keep asking while the opener is silent; loop() spaces transmissions.
        if (this->pending_tx_.empty()) {
            this->enqueue_transmit(CommandType::QUERY_DOOR_STATUS, 0, true);
        }
        this->scheduler_->set_timeout(this->ratgdo_, TIMEOUT_DOOR_PRESS_GATE, DOOR_PRESS_GATE_RECHECK, [this]() {
            this->check_door_press_gate_();
        });
    }

    void Secplus1::clear_door_press_gate_()
    {
        this->scheduler_->cancel_timeout(this->ratgdo_, TIMEOUT_DOOR_PRESS_GATE);
        this->door_press_gate_active_ = false;
        this->door_press_gate_awaiting_ = false;
        this->door_press_gate_good_ = 0;
    }

    void Secplus1::note_door_press_gate_reply_(bool valid)
    {
        if (!this->door_press_gate_active_) {
            return;
        }
        this->door_press_gate_awaiting_ = false;
        if (!valid) {
            this->door_press_gate_good_ = 0;
        } else if (this->door_press_gate_good_ < UINT8_MAX) {
            this->door_press_gate_good_++;
        }
    }

    bool Secplus1::cancel_unsafe_retry_press_()
    {
        // Even an unconfirmed change since the miss makes a retry unsafe. Do
        // not send it and do not claim successful recovery.
        if (!this->flags_.door_command_retried
            || (this->door_state == this->watched_door_baseline_ && this->maybe_door_state == this->watched_door_baseline_)) {
            return false;
        }
        this->door_command_unverified_++;
        this->log_door_command_stats_("retry cancelled before transmit");
        this->cancel_pending_door_presses_();
        this->clear_door_command_watch_();
        this->publish_door_command_stats_();
        return true;
    }

    Result Secplus1::call(Args args)
    {
        if (args.tag == Args::Tag::query_status) {
#if RATGDO_SECPLUS1_ABORT_TEST
            this->abort_test_remaining_ = ABORT_TEST_FRAMES;
            this->abort_test_step_();
            return { };
#endif
            this->enqueue_transmit(CommandType::QUERY_DOOR_STATUS);
            // Second QUERY_DOOR_STATUS 2.5s later so the two-confirmation gate
            // can commit state from a single manual query-status press.
            this->enqueue_transmit(CommandType::QUERY_DOOR_STATUS, millis() + 2500);
            this->enqueue_transmit(CommandType::QUERY_OTHER_STATUS, millis() + 300);
            this->enqueue_transmit(CommandType::OBSTRUCTION, millis() + 600);
        }
        return { };
    }

    void Secplus1::check_tx_echo_()
    {
        if (!this->uart_.available()) {
            if (millis() - this->last_tx_ > TX_ECHO_TIMEOUT) {
                this->tx_echo_pending_ = false;
                ESP_LOGD(TAG, "[%" PRIu32 "] No echo of sent byte [%02X]", millis(), this->tx_echo_expected_);
            }
            return;
        }
        // Queries are sent with RX enabled, so the first byte read afterwards
        // is our own. It is never a command from a wall panel, whatever it
        // decodes as.
        uint8_t echo = this->uart_.read();
        this->last_rx_ = millis();
        this->tx_echo_pending_ = false;
        // Anything half-read before we transmitted is stale.
        this->flags_.rx_reading_msg = false;
        this->rx_byte_count_ = 0;
        if (echo == this->tx_echo_expected_) {
            {
                char hex[format_hex_pretty_size(1)];
                ESP_LOG2(TAG, "[%" PRIu32 "] Received byte: [%s]", millis(), format_hex_pretty_to(hex, &echo, 1));
            }
            this->rx_packet_[this->rx_byte_count_++] = echo;
            this->flags_.rx_reading_msg = true;
            return;
        }

        // The bus did not carry what we sent (another transmitter overlapped
        // it, or a bit edge went out late). The opener received the same
        // corrupted byte, so no usable reply follows.
#ifdef USE_ESP8266
        const char* parity = this->uart_.readParity() == RatgdoUART::parityEven(echo) ? "ok" : "bad";
#else
        const char* parity = "n/a";
#endif
        if (echo == 0x37 || (echo >= 0x30 && echo <= 0x35)) {
            // A one-byte panel command. Read back as 0x30 this is a door
            // press, and the opener may act on it: do not report it as a
            // wall-panel button, count it, and watch the door.
            ++this->tx_echo_errors_;
            ESP_LOGW(TAG, "[%" PRIu32 "] Sent [%02X] but the bus read back [%02X] (parity %s, rx_gap=%" PRIu32 "); the opener may act on it. Count=%" PRIu32,
                millis(), this->tx_echo_expected_, echo, parity, this->tx_echo_rx_gap_, this->tx_echo_errors_);
            this->publish_door_command_stats_();
            this->note_door_suspect_();
        } else {
            ESP_LOGD(TAG, "[%" PRIu32 "] Sent [%02X] but the bus read back [%02X] (parity %s, rx_gap=%" PRIu32 ")",
                millis(), this->tx_echo_expected_, echo, parity, this->tx_echo_rx_gap_);
        }
        this->note_door_press_gate_reply_(false);
    }

    optional<RxCommand> Secplus1::read_command()
    {
        if (this->tx_echo_pending_) {
            this->check_tx_echo_();
        }
        if (!this->flags_.rx_reading_msg) {
            while (this->uart_.available()) {
                uint8_t ser_byte = this->uart_.read();
                this->last_rx_ = millis();

                if (ser_byte < 0x30 || ser_byte > 0x3A) {
                    char hex[format_hex_pretty_size(1)];
#if RATGDO_SECPLUS1_DIAGNOSTICS
                    ESP_LOGD(TAG, "[%" PRIu32 "] RX ignored: [%s] tx_age=%" PRIu32, millis(), format_hex_pretty_to(hex, &ser_byte, 1), millis() - this->last_tx_);
#else
                    ESP_LOG2(TAG, "[%" PRIu32 "] Ignoring byte [%s], baud: %d", millis(), format_hex_pretty_to(hex, &ser_byte, 1), this->uart_.baudRate());
#endif
                    this->rx_byte_count_ = 0;
                    continue;
                }
                this->rx_packet_[this->rx_byte_count_++] = ser_byte;
                {
                    char hex[format_hex_pretty_size(1)];
                    ESP_LOG2(TAG, "[%" PRIu32 "] Received byte: [%s]", millis(), format_hex_pretty_to(hex, &ser_byte, 1));
                }
                this->flags_.rx_reading_msg = true;

                if (ser_byte == 0x37 || (ser_byte >= 0x30 && ser_byte <= 0x35)) {
                    this->rx_packet_[this->rx_byte_count_++] = 0;
                    this->flags_.rx_reading_msg = false;
                    this->rx_byte_count_ = 0;
                    {
                        char hex[format_hex_pretty_size(1)];
#if RATGDO_SECPLUS1_DIAGNOSTICS
                        ESP_LOGD(TAG, "[%" PRIu32 "] RX command: [%s]", millis(), format_hex_pretty_to(hex, &this->rx_packet_[0], 1));
#else
                        ESP_LOG2(TAG, "[%" PRIu32 "] Received command: [%s]", millis(), format_hex_pretty_to(hex, &this->rx_packet_[0], 1));
#endif
                    }
                    return this->decode_packet(this->rx_packet_);
                }

                break;
            }
        }
        if (this->flags_.rx_reading_msg) {
            while (this->uart_.available()) {
                uint8_t ser_byte = this->uart_.read();
                this->last_rx_ = millis();
                this->rx_packet_[this->rx_byte_count_++] = ser_byte;
                {
                    char hex[format_hex_pretty_size(1)];
                    ESP_LOG2(TAG, "[%" PRIu32 "] Received byte: [%s]", millis(), format_hex_pretty_to(hex, &ser_byte, 1));
                }

                if (this->rx_byte_count_ == RX_LENGTH) {
                    this->flags_.rx_reading_msg = false;
                    this->rx_byte_count_ = 0;
                    this->print_rx_packet(this->rx_packet_);
                    return this->decode_packet(this->rx_packet_);
                }
            }

            if (millis() - this->last_rx_ > 100) {
                // if we have a partial packet and it's been over 100ms since last byte was read,
                // the rest is not coming (a full packet should be received in ~20ms),
                // discard it so we can read the following packet correctly
                {
                    char hex[format_hex_pretty_size(1)];
#if RATGDO_SECPLUS1_DIAGNOSTICS
                    ESP_LOGW(TAG, "[%" PRIu32 "] Discard incomplete packet: [%s ...] length=%u rx_age=%" PRIu32 " tx_age=%" PRIu32 " queued=%u",
                        millis(), format_hex_pretty_to(hex, &this->rx_packet_[0], 1), this->rx_byte_count_,
                        millis() - this->last_rx_, millis() - this->last_tx_, static_cast<unsigned>(this->pending_tx_.size()));
#else
                    ESP_LOGW(TAG, "[%" PRIu32 "] Discard incomplete packet: [%s ...]", millis(), format_hex_pretty_to(hex, &this->rx_packet_[0], 1));
#endif
                }
                this->flags_.rx_reading_msg = false;
                this->rx_byte_count_ = 0;
                // Usually our own query echo with no reply from the opener.
                this->note_door_press_gate_reply_(false);
            }
        }

        return { };
    }

    void Secplus1::print_rx_packet(const RxPacket& packet) const
    {
        constexpr size_t hex_size = format_hex_pretty_size(RX_LENGTH);
        char hex[hex_size];
#if RATGDO_SECPLUS1_DIAGNOSTICS
        ESP_LOGD(TAG, "[%" PRIu32 "] RX packet: [%s] tx_age=%" PRIu32, millis(), format_hex_pretty_to(hex, packet, RX_LENGTH), millis() - this->last_tx_);
#else
        ESP_LOG2(TAG, "[%" PRIu32 "] Received packet: [%s]", millis(), format_hex_pretty_to(hex, packet, RX_LENGTH));
#endif
    }

    void Secplus1::print_tx_packet(const TxPacket& packet) const
    {
        constexpr size_t hex_size = format_hex_pretty_size(TX_LENGTH);
        char hex[hex_size];
        ESP_LOG2(TAG, "[%" PRIu32 "] Sending packet: [%s]", millis(), format_hex_pretty_to(hex, packet, TX_LENGTH));
    }

    optional<RxCommand> Secplus1::decode_packet(const RxPacket& packet) const
    {
        CommandType cmd_type = to_CommandType(packet[0], CommandType::UNKNOWN);
        return RxCommand { cmd_type, packet[1] };
    }

    // unknown meaning of observed command-responses:
    // 40 00 and 40 80
    // 53 01
    // C0 3F
    // F8 3F
    // FE 3F

    void Secplus1::handle_command(const RxCommand& cmd)
    {
        if (cmd.req == CommandType::TOGGLE_DOOR_RELEASE || cmd.resp == 0x31) {
            if (this->wall_panel_emulation_state_ == WallPanelEmulationState::WAITING) {
                ESP_LOGD(TAG, "wall panel is starting");
                this->flags_.wall_panel_starting = true;
            }
        } else if (cmd.req == CommandType::QUERY_DOOR_STATUS) {
            DoorState door_state;
            auto val = cmd.resp & 0x7;
            // 000 0x0 stopped
            // 001 0x1 opening
            // 010 0x2 open
            // 100 0x4 closing
            // 101 0x5 closed
            // 110 0x6 stopped

            if (val == 0x2) {
                door_state = DoorState::OPEN;
            } else if (val == 0x5) {
                door_state = DoorState::CLOSED;
            } else if (val == 0x0 || val == 0x6) {
                door_state = DoorState::STOPPED;
            } else if (val == 0x1) {
                door_state = DoorState::OPENING;
            } else if (val == 0x4) {
                door_state = DoorState::CLOSING;
            } else {
                door_state = DoorState::UNKNOWN;
            }
            this->note_door_press_gate_reply_(door_state != DoorState::UNKNOWN);

            // A single UNKNOWN (garbled/noise) read must not disturb a real
            // state we already hold or a real candidate we are confirming.
            // Otherwise a lone noise read between two good reads resets the
            // two-confirmation gate every cycle, which can wedge a stale wrong
            // state indefinitely because a correcting read never gets to land
            // twice in a row (observed 2026-07-15: ~34min stuck CLOSED while
            // physically OPEN). Ignore it and wait for the next real read.
            if (door_state == DoorState::UNKNOWN && this->door_state != DoorState::UNKNOWN) {
                ESP_LOGD(TAG, "Ignoring UNKNOWN door status read (noise), keeping %s (pending %s)",
                    LOG_STR_ARG(DoorState_to_string(this->door_state)), LOG_STR_ARG(DoorState_to_string(this->maybe_door_state)));
                return;
            }

            if (!this->flags_.is_0x37_panel && door_state != this->maybe_door_state) {
                if (this->maybe_door_state == this->door_state) {
                    // The background timer was chosen before this response.
                    // Pull the next probe forward when a correction starts.
                    this->schedule_door_status_probe_(DOOR_STATUS_FAST_INTERVAL);
                }
                this->maybe_door_state = door_state;
                this->maybe_door_state_since_ = millis();
#if RATGDO_SECPLUS1_DIAGNOSTICS
                ESP_LOGD(TAG, "[%" PRIu32 "] Door candidate=%s reply=%02X held=%s; waiting for confirmation",
                    millis(), LOG_STR_ARG(DoorState_to_string(door_state)), cmd.resp, LOG_STR_ARG(DoorState_to_string(this->door_state)));
#else
                ESP_LOG1(TAG, "Door maybe %s, waiting for 2nd status message to confirm", LOG_STR_ARG(DoorState_to_string(door_state)));
#endif
            } else {
                const bool state_changed = this->door_state != door_state;
#if RATGDO_SECPLUS1_DIAGNOSTICS
                ESP_LOGD(TAG, "[%" PRIu32 "] Door confirmed=%s reply=%02X changed=%u candidate_age=%" PRIu32,
                    millis(), LOG_STR_ARG(DoorState_to_string(door_state)), cmd.resp, state_changed, millis() - this->maybe_door_state_since_);
#endif
                this->maybe_door_state = door_state;
                this->door_state = door_state;
                this->flags_.door_moving = door_state == DoorState::OPENING || door_state == DoorState::CLOSING;
                // The command watch only trusts state that survived the
                // two-confirmation gate above: a single garbled read must not
                // clear the watch as "confirmed" (door moved) any more than it
                // may commit state to HA. Runs before the deferred-action
                // dispatch so the old watch settles before a new one can arm.
                this->observe_door_command_status_(door_state);
                if (state_changed) {
                    // Direction-changing presses require confirmed movement,
                    // and must dispatch using the newly committed state.
                    this->on_door_state_.trigger(door_state);
                }
                // Only use the candidate timestamp after it passed confirmation.
                // This accounts for the two-read delay without trusting a lone reply.
                const auto now = millis();
                const auto observed_at = !this->flags_.is_0x37_panel && now - this->maybe_door_state_since_ <= 10000
                    ? this->maybe_door_state_since_
                    : now; // Do not extrapolate across a long gap without valid replies.
                this->ratgdo_->received(door_state, observed_at);
                this->maybe_run_deferred_door_action_();
            }
        } else if (cmd.req == CommandType::QUERY_DOOR_STATUS_0x37) {
            this->flags_.is_0x37_panel = true;
            // A capable panel is actively polling door status, so ratgdo has a
            // live passive source of truth and need not probe. Stamp the time so
            // the probe stays dormant while polls keep arriving, and resumes on
            // its own if they stop (panel removed/failed).
            this->last_passive_door_status_ = millis();
            auto cmd = this->pending_tx();
            if (cmd && cmd.value() == CommandType::TOGGLE_LOCK_PRESS) {
                this->do_transmit_if_pending();
            } else {
                // inject door status request
                if (flags_.door_moving || (millis() - this->last_status_query_ > 10000)) {
                    this->transmit_byte(static_cast<uint8_t>(CommandType::QUERY_DOOR_STATUS));
                    this->last_status_query_ = millis();
                }
            }
        } else if (cmd.req == CommandType::QUERY_OTHER_STATUS) {
            LightState light_state = to_LightState((cmd.resp >> 2) & 1, LightState::UNKNOWN);

            if (!this->flags_.is_0x37_panel && light_state != this->maybe_light_state) {
                this->maybe_light_state = light_state;
            } else {
                this->light_state = light_state;
                this->ratgdo_->received(light_state);
            }

            LockState lock_state = to_LockState((~cmd.resp >> 3) & 1, LockState::UNKNOWN);
            if (!this->flags_.is_0x37_panel && lock_state != this->maybe_lock_state) {
                this->maybe_lock_state = lock_state;
            } else {
                this->lock_state = lock_state;
                this->ratgdo_->received(lock_state);
            }
        } else if (cmd.req == CommandType::OBSTRUCTION) {
            ObstructionState obstruction_state = cmd.resp == 0 ? ObstructionState::CLEAR : ObstructionState::OBSTRUCTED;
            this->ratgdo_->received(obstruction_state);
        } else if (cmd.req == CommandType::TOGGLE_LIGHT_PRESS) {
            // motion was detected, or the light toggle button was pressed
            // either way it's ok to trigger motion detection
            if (this->light_state == LightState::OFF) {
                this->ratgdo_->received(MotionState::DETECTED);
            }
        } else if (cmd.req == CommandType::TOGGLE_DOOR_PRESS) {
            this->ratgdo_->received(ButtonState::PRESSED);
            // A wall-panel press moves the door; report it without waiting
            // for the next idle probe.
            this->note_door_suspect_();
        } else if (cmd.req == CommandType::TOGGLE_DOOR_RELEASE) {
            this->ratgdo_->received(ButtonState::RELEASED);
        }
    }

    bool Secplus1::do_transmit_if_pending()
    {
        auto cmd = this->pop_pending_tx();
        if (cmd) {
            if (cmd.value() == CommandType::TOGGLE_DOOR_PRESS && this->cancel_unsafe_retry_press_()) {
                return true;
            }
            if (cmd.value() == CommandType::QUERY_DOOR_STATUS && this->door_press_gate_active_) {
                // An earlier gated query that never got an answer breaks the run.
                if (this->door_press_gate_awaiting_) {
                    this->door_press_gate_good_ = 0;
                }
                this->door_press_gate_awaiting_ = true;
            }
            this->enqueue_command_pair(cmd.value());
            this->transmit_byte(static_cast<uint32_t>(cmd.value()));
        }
        return cmd.has_value();
    }

    void Secplus1::enqueue_command_pair(CommandType cmd)
    {
        auto now = millis();
        if (cmd == CommandType::TOGGLE_DOOR_PRESS) {
            this->flags_.door_moving = true;
            this->mark_door_command_transmitted_();
            this->enqueue_transmit(CommandType::TOGGLE_DOOR_RELEASE, now + 500);
            this->enqueue_transmit(CommandType::QUERY_DOOR_STATUS, now + DOOR_ACTION_POST_TOGGLE_QUERY_DELAY);
            this->enqueue_transmit(CommandType::QUERY_DOOR_STATUS, now + DOOR_ACTION_POST_TOGGLE_QUERY_DELAY + 2500);
            this->schedule_post_command_verify_();
        } else if (cmd == CommandType::TOGGLE_LIGHT_PRESS) {
            this->enqueue_transmit(CommandType::TOGGLE_LIGHT_RELEASE, now + 500);
        } else if (cmd == CommandType::TOGGLE_LOCK_PRESS) {
            this->enqueue_transmit(CommandType::TOGGLE_LOCK_RELEASE, now + 3500);
        };
    }

    void Secplus1::enqueue_transmit(CommandType cmd, uint32_t time, bool door_warmup)
    {
        if (time == 0) {
            time = millis();
        }
        this->pending_tx_.push(TxCommand { cmd, door_warmup, time });
#if RATGDO_SECPLUS1_DIAGNOSTICS
        ESP_LOGD(TAG, "[%" PRIu32 "] Queue byte: [%02X] due=%" PRIu32 " warmup=%u depth=%u",
            millis(), static_cast<unsigned>(cmd), time, door_warmup, static_cast<unsigned>(this->pending_tx_.size()));
#endif
    }

    void Secplus1::cancel_pending_door_presses_()
    {
        // A press waiting at the gate is pending too.
        this->clear_door_press_gate_();
        decltype(this->pending_tx_) remaining;
        while (!this->pending_tx_.empty()) {
            auto cmd = this->pending_tx_.top();
            this->pending_tx_.pop();
            if (cmd.request != CommandType::TOGGLE_DOOR_PRESS && !cmd.door_warmup) {
                remaining.push(cmd);
            }
        }
        this->pending_tx_ = std::move(remaining);
    }

    optional<CommandType> Secplus1::pending_tx()
    {
        if (this->pending_tx_.empty()) {
            return { };
        }
        auto cmd = this->pending_tx_.top();
        if (cmd.time > millis()) {
            return { };
        }
        return cmd.request;
    }

    optional<CommandType> Secplus1::pop_pending_tx()
    {
        auto cmd = this->pending_tx();
        if (cmd) {
            this->pending_tx_.pop();
        }
        return cmd;
    }

    void Secplus1::transmit_byte(uint32_t value)
    {
#if RATGDO_SECPLUS1_DIAGNOSTICS
        const auto previous_tx = this->last_tx_;
#endif
        const auto previous_rx = this->last_rx_;
        bool enable_rx = (value == 0x38) || (value == 0x39) || (value == 0x3A);
#if RATGDO_SECPLUS1_CHECKED_QUERY_TX
        if (enable_rx) {
            this->transmit_query_checked_(static_cast<uint8_t>(value));
            return;
        }
#endif
        if (!enable_rx) {
            this->uart_.enableIntTx(false);
        }
        this->uart_.write(value);
        this->last_tx_ = millis();
        if (!enable_rx) {
            this->uart_.enableIntTx(true);
        } else {
            this->tx_echo_pending_ = true;
            this->tx_echo_expected_ = static_cast<uint8_t>(value);
            this->tx_echo_rx_gap_ = this->last_tx_ - previous_rx;
        }
        {
            uint8_t byte_val = static_cast<uint8_t>(value);
            char hex[format_hex_pretty_size(1)];
#if RATGDO_SECPLUS1_DIAGNOSTICS
            ESP_LOGD(TAG, "[%" PRIu32 "] Sent byte: [%s] tx_gap=%" PRIu32 " rx_gap=%" PRIu32 " rx_partial=%u queued=%u",
                millis(), format_hex_pretty_to(hex, &byte_val, 1), this->last_tx_ - previous_tx,
                this->last_tx_ - previous_rx, this->flags_.rx_reading_msg, static_cast<unsigned>(this->pending_tx_.size()));
#else
            ESP_LOGD(TAG, "[%" PRIu32 "] Sent byte: [%s]", millis(), format_hex_pretty_to(hex, &byte_val, 1));
#endif
        }
    }

#if RATGDO_SECPLUS1_CHECKED_QUERY_TX
    void Secplus1::transmit_query_checked_(uint8_t value, int8_t abort_at_data_bit)
    {
        const auto previous_tx = this->last_tx_;
        const auto previous_rx = this->last_rx_;
        QueryTxInfo info;
        const auto result = this->uart_.send_query_checked(value, &info, abort_at_data_bit);
        this->last_tx_ = millis();
        // Interrupts were off for the frame, so none of it was received.
        // Anything half-read before it is stale.
        this->flags_.rx_reading_msg = false;
        this->rx_byte_count_ = 0;
        if (result == QueryTxResult::SENT) {
            // The bus was verified bit by bit: stand in for the echo, so the
            // opener's reply completes the packet as before.
            this->rx_packet_[this->rx_byte_count_++] = value;
            this->flags_.rx_reading_msg = true;
            this->last_rx_ = this->last_tx_;
            this->tx_query_retried_ = false;
#if RATGDO_SECPLUS1_DIAGNOSTICS
            ESP_LOGD(TAG, "[%" PRIu32 "] Sent byte: [%02X] tx_gap=%" PRIu32 " rx_gap=%" PRIu32 " fall=%uus rise=%uus late=%uus queued=%u",
                millis(), value, this->last_tx_ - previous_tx, this->last_tx_ - previous_rx,
                info.fall_us, info.rise_us, info.late_us, static_cast<unsigned>(this->pending_tx_.size()));
#else
            ESP_LOGD(TAG, "[%" PRIu32 "] Sent byte: [%02X]", millis(), value);
#endif
            return;
        }

        ++this->tx_query_aborts_;
        ESP_LOGD(TAG, "[%" PRIu32 "] Query [%02X] %s (bit %d, low=%uus rise=%uus late=%uus, tx_gap=%" PRIu32 " rx_gap=%" PRIu32 "). Count=%" PRIu32,
            millis(), value, result == QueryTxResult::BUSY ? "not sent, bus busy" : (info.no_drive ? "aborted, line did not go low" : "aborted"), info.aborted_at,
            info.low_us, info.rise_us, info.late_us, this->last_tx_ - previous_tx, this->last_tx_ - previous_rx, this->tx_query_aborts_);
        this->publish_door_command_stats_();
        this->note_door_press_gate_reply_(false);
        if (abort_at_data_bit >= 0) {
            return; // deliberate, from the hardware test
        }
        // Send it again once. A second failure waits for the next probe, so a
        // line that is stuck cannot turn this into a stream of frames.
        if (this->tx_query_retried_) {
            this->tx_query_retried_ = false;
        } else {
            this->tx_query_retried_ = true;
            this->enqueue_transmit(to_CommandType(value, CommandType::QUERY_DOOR_STATUS), millis() + QUERY_RETRY_DELAY);
        }
    }
#endif

#if RATGDO_SECPLUS1_ABORT_TEST
    void Secplus1::abort_test_step_()
    {
        if (this->abort_test_remaining_ == 0) {
            ESP_LOGI(TAG, "Abort test finished; querying door status");
            this->enqueue_transmit(CommandType::QUERY_DOOR_STATUS, millis() + 500);
            this->enqueue_transmit(CommandType::QUERY_DOOR_STATUS, millis() + 3000);
            return;
        }
        if (this->pending_tx_.empty() && millis() - this->last_tx_ > 200 && millis() - this->last_rx_ > 50) {
            // A 0x38 abandoned at data bit 3 is the worst case the sender can
            // leave on the bus: the start bit and three low data bits.
            --this->abort_test_remaining_;
            ESP_LOGI(TAG, "Abort test: 0x38 abandoned at data bit 3, %u left", this->abort_test_remaining_);
            this->transmit_query_checked_(static_cast<uint8_t>(CommandType::QUERY_DOOR_STATUS), 3);
        }
        this->scheduler_->set_timeout(this->ratgdo_, TIMEOUT_ABORT_TEST, ABORT_TEST_INTERVAL, [this] {
            this->abort_test_step_();
        });
    }
#endif

} // namespace secplus1
} // namespace esphome::ratgdo

#endif // PROTOCOL_SECPLUSV1
