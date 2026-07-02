
#ifdef PROTOCOL_SECPLUSV1

#include "secplus1.h"
#include "ratgdo.h"

#include "esphome/core/gpio.h"
#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"
#include "esphome/core/scheduler.h"

namespace esphome::ratgdo {
namespace secplus1 {

    using namespace scheduler_ids;
    static const char* const TAG = "ratgdo_secplus1";
    static constexpr uint32_t DOOR_STATE_CALLBACK_TIMEOUT = 2000;
    static constexpr uint32_t DOOR_ACTION_DEFER_TIMEOUT = 22000;
    static constexpr uint32_t DOOR_ACTION_POST_TOGGLE_QUERY_DELAY = 1200;
    // Pre-toggle bus warm-up: wake the opener's receiver with a short
    // QUERY_DOOR_STATUS burst before the button press so a single press lands
    // instead of being dropped into an idle bus.
    static constexpr uint8_t DOOR_PRESS_WARMUP_QUERIES = 3;
    static constexpr uint32_t DOOR_PRESS_WARMUP_INTERVAL = 200; // ms between warm-up queries and before the press
    // If no passive door-status poll has been observed for this long, assume no
    // capable panel is supplying door status and resume active probing. 3x the
    // 0x37 panel's ~10s poll interval, so an occasional missed poll won't trip it.
    static constexpr uint32_t DOOR_STATUS_PASSIVE_TIMEOUT = 30000;

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
        this->clear_deferred_door_action_();
        this->door_state = DoorState::UNKNOWN;
        this->light_state = LightState::UNKNOWN;
        this->scheduler_->cancel_timeout(this->ratgdo_, TIMEOUT_WALL_PANEL_EMULATION);
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
        this->scheduler_->set_timeout(this->ratgdo_, TIMEOUT_DOOR_PROBE, 5000, [this] {
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
        // Probe fast while we don't know the state or the door is moving (quick
        // post-reboot heal and quick settle), then drop to a low background rate
        // to bound staleness. The loop is kept alive even when a passive source is
        // present (cheap timer + compare) so probing can resume if it disappears.
        // Self-sustaining single timer (same scheduler key replaces the prior one).
        uint32_t next_ms = (this->door_state == DoorState::UNKNOWN || this->flags_.door_moving)
            ? 2500 : 10000;
        this->scheduler_->set_timeout(this->ratgdo_, TIMEOUT_DOOR_PROBE, next_ms, [this] {
            this->door_status_probe_();
        });
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
            if (millis() - this->wall_panel_emulation_start_ > 35000 && !this->flags_.wall_panel_starting) {
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
        this->scheduler_->set_timeout(this->ratgdo_, TIMEOUT_DOOR_STATE_EXPIRY, DOOR_STATE_CALLBACK_TIMEOUT, [this]() {
            ESP_LOGW(TAG, "Door state callback expired, clearing");
            this->on_door_state_.clear();
        });
    }

    void Secplus1::cancel_door_state_expiry()
    {
        this->scheduler_->cancel_timeout(this->ratgdo_, TIMEOUT_DOOR_STATE_EXPIRY);
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

    void Secplus1::door_action(DoorAction action)
    {
        ESP_LOG1(TAG, "Door action: %s, door state: %s", LOG_STR_ARG(DoorAction_to_string(action)), LOG_STR_ARG(DoorState_to_string(this->door_state)));
        if (action == DoorAction::UNKNOWN) {
            return;
        }

        if ((action == DoorAction::OPEN || action == DoorAction::CLOSE) && this->door_state == DoorState::UNKNOWN) {
            this->defer_door_action_(action);
            return;
        }

        const uint32_t double_toggle_delay = 1000;
        if (action == DoorAction::TOGGLE) {
            this->toggle_door();
        } else if (action == DoorAction::OPEN) {
            if (this->door_state == DoorState::CLOSED || this->door_state == DoorState::CLOSING) {
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
        auto now = millis();
        uint32_t press_delay = 0;
        if (this->door_state != DoorState::OPENING && this->door_state != DoorState::CLOSING) {
            for (uint8_t i = 0; i < DOOR_PRESS_WARMUP_QUERIES; i++) {
                this->enqueue_transmit(CommandType::QUERY_DOOR_STATUS, now + i * DOOR_PRESS_WARMUP_INTERVAL);
            }
            press_delay = DOOR_PRESS_WARMUP_QUERIES * DOOR_PRESS_WARMUP_INTERVAL;
        }
        this->enqueue_transmit(CommandType::TOGGLE_DOOR_PRESS, now + press_delay);
        if (this->door_state == DoorState::STOPPED || this->door_state == DoorState::OPEN || this->door_state == DoorState::CLOSED) {
            this->flags_.door_moving = true;
        }
    }

    Result Secplus1::call(Args args)
    {
        if (args.tag == Args::Tag::query_status) {
            this->enqueue_transmit(CommandType::QUERY_DOOR_STATUS);
            // Second QUERY_DOOR_STATUS 2.5s later so the two-confirmation gate
            // can commit state from a single manual query-status press.
            this->enqueue_transmit(CommandType::QUERY_DOOR_STATUS, millis() + 2500);
            this->enqueue_transmit(CommandType::QUERY_OTHER_STATUS, millis() + 300);
            this->enqueue_transmit(CommandType::OBSTRUCTION, millis() + 600);
        }
        return { };
    }

    optional<RxCommand> Secplus1::read_command()
    {
        if (!this->flags_.rx_reading_msg) {
            while (this->uart_.available()) {
                uint8_t ser_byte = this->uart_.read();
                this->last_rx_ = millis();

                if (ser_byte < 0x30 || ser_byte > 0x3A) {
                    char hex[format_hex_pretty_size(1)];
                    ESP_LOG2(TAG, "[%d] Ignoring byte [%s], baud: %d", millis(), format_hex_pretty_to(hex, &ser_byte, 1), this->uart_.baudRate());
                    this->rx_byte_count_ = 0;
                    continue;
                }
                this->rx_packet_[this->rx_byte_count_++] = ser_byte;
                {
                    char hex[format_hex_pretty_size(1)];
                    ESP_LOG2(TAG, "[%d] Received byte: [%s]", millis(), format_hex_pretty_to(hex, &ser_byte, 1));
                }
                this->flags_.rx_reading_msg = true;

                if (ser_byte == 0x37 || (ser_byte >= 0x30 && ser_byte <= 0x35)) {
                    this->rx_packet_[this->rx_byte_count_++] = 0;
                    this->flags_.rx_reading_msg = false;
                    this->rx_byte_count_ = 0;
                    {
                        char hex[format_hex_pretty_size(1)];
                        ESP_LOG2(TAG, "[%d] Received command: [%s]", millis(), format_hex_pretty_to(hex, &this->rx_packet_[0], 1));
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
                    ESP_LOG2(TAG, "[%d] Received byte: [%s]", millis(), format_hex_pretty_to(hex, &ser_byte, 1));
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
                    ESP_LOGW(TAG, "[%d] Discard incomplete packet: [%s ...]", millis(), format_hex_pretty_to(hex, &this->rx_packet_[0], 1));
                }
                this->flags_.rx_reading_msg = false;
                this->rx_byte_count_ = 0;
            }
        }

        return { };
    }

    void Secplus1::print_rx_packet(const RxPacket& packet) const
    {
        constexpr size_t hex_size = format_hex_pretty_size(RX_LENGTH);
        char hex[hex_size];
        ESP_LOG2(TAG, "[%d] Received packet: [%s]", millis(), format_hex_pretty_to(hex, packet, RX_LENGTH));
    }

    void Secplus1::print_tx_packet(const TxPacket& packet) const
    {
        constexpr size_t hex_size = format_hex_pretty_size(TX_LENGTH);
        char hex[hex_size];
        ESP_LOG2(TAG, "[%d] Sending packet: [%s]", millis(), format_hex_pretty_to(hex, packet, TX_LENGTH));
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

            if (this->maybe_door_state != door_state) {
                this->on_door_state_.trigger(door_state);
            }

            if (!this->flags_.is_0x37_panel && door_state != this->maybe_door_state) {
                this->maybe_door_state = door_state;
                ESP_LOG1(TAG, "Door maybe %s, waiting for 2nd status message to confirm", LOG_STR_ARG(DoorState_to_string(door_state)));
            } else {
                // Two consecutive identical reads confirm state — but if both reads were
                // UNKNOWN (e.g. resp=0xFF bus noise) and we already hold a real physical
                // state, discard: noise should not erase confirmed truth.
                if (door_state == DoorState::UNKNOWN && this->door_state != DoorState::UNKNOWN) {
                    ESP_LOGD(TAG, "Discarding two UNKNOWN door status reads, keeping %s", LOG_STR_ARG(DoorState_to_string(this->door_state)));
                    this->maybe_door_state = this->door_state;
                    return;
                }
                this->maybe_door_state = door_state;
                this->door_state = door_state;
                if (this->door_state == DoorState::STOPPED || this->door_state == DoorState::OPEN || this->door_state == DoorState::CLOSED) {
                    this->flags_.door_moving = false;
                }
                this->ratgdo_->received(door_state);
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
        } else if (cmd.req == CommandType::TOGGLE_DOOR_RELEASE) {
            this->ratgdo_->received(ButtonState::RELEASED);
        }
    }

    bool Secplus1::do_transmit_if_pending()
    {
        auto cmd = this->pop_pending_tx();
        if (cmd) {
            this->enqueue_command_pair(cmd.value());
            this->transmit_byte(static_cast<uint32_t>(cmd.value()));
        }
        return cmd.has_value();
    }

    void Secplus1::enqueue_command_pair(CommandType cmd)
    {
        auto now = millis();
        if (cmd == CommandType::TOGGLE_DOOR_PRESS) {
            this->enqueue_transmit(CommandType::TOGGLE_DOOR_RELEASE, now + 500);
            this->enqueue_transmit(CommandType::QUERY_DOOR_STATUS, now + DOOR_ACTION_POST_TOGGLE_QUERY_DELAY);
            this->enqueue_transmit(CommandType::QUERY_DOOR_STATUS, now + DOOR_ACTION_POST_TOGGLE_QUERY_DELAY + 2500);
        } else if (cmd == CommandType::TOGGLE_LIGHT_PRESS) {
            this->enqueue_transmit(CommandType::TOGGLE_LIGHT_RELEASE, now + 500);
        } else if (cmd == CommandType::TOGGLE_LOCK_PRESS) {
            this->enqueue_transmit(CommandType::TOGGLE_LOCK_RELEASE, now + 3500);
        };
    }

    void Secplus1::enqueue_transmit(CommandType cmd, uint32_t time)
    {
        if (time == 0) {
            time = millis();
        }
        this->pending_tx_.push(TxCommand { cmd, time });
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
        bool enable_rx = (value == 0x38) || (value == 0x39) || (value == 0x3A);
        if (!enable_rx) {
            this->uart_.enableIntTx(false);
        }
        this->uart_.write(value);
        this->last_tx_ = millis();
        if (!enable_rx) {
            this->uart_.enableIntTx(true);
        }
        {
            uint8_t byte_val = static_cast<uint8_t>(value);
            char hex[format_hex_pretty_size(1)];
            ESP_LOGD(TAG, "[%d] Sent byte: [%s]", millis(), format_hex_pretty_to(hex, &byte_val, 1));
        }
    }

} // namespace secplus1
} // namespace esphome::ratgdo

#endif // PROTOCOL_SECPLUSV1
