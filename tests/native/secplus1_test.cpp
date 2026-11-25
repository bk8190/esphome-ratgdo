#include "cover_stub.h"
#include "ratgdo.h"
#include "secplus1.h"

#include <iostream>
#include <string>

using namespace esphome;
using namespace esphome::ratgdo;
using namespace esphome::ratgdo::secplus1;
using namespace esphome::ratgdo::scheduler_ids;

#define CHECK(condition)                                                         \
    do {                                                                         \
        if (!(condition))                                                        \
            throw std::runtime_error(std::string(__func__) + ": " + #condition); \
    } while (false)

class TestProtocol : public Secplus1 {
public:
    using Secplus1::door_command_retries_;
    using Secplus1::door_state;
    using Secplus1::door_status_probe_;
    using Secplus1::flags_;
    using Secplus1::handle_command;
    using Secplus1::maybe_door_state;
    using Secplus1::pending_door_action_;
    using Secplus1::pending_tx_;
    using Secplus1::uart_;
    using Secplus1::wall_panel_emulation_state_;
    using Secplus1::watched_door_action_;
};

struct Rig {
    Scheduler scheduler;
    RATGDOComponent component;
    RATGDOCover cover { &component };
    TestProtocol protocol;
    InternalGPIOPin pin;
    DoorState physical;
    bool actuate = false;
    bool noise_once = false;
    bool drop_queries = false;
    // Answer until the first heard press, then let the test feed replies.
    bool drop_after_press = false;
    bool reverse_by_stopping = false;
    // Field-observed failure: until this time the opener ignores presses and
    // does not answer queries (only the ratgdo's own echo comes back).
    uint32_t deaf_until = 0;
    // Until this time door-status replies are garbled (decode as UNKNOWN).
    uint32_t garble_until = 0;
    int presses = 0;
    int ignored_presses = 0;
    uint32_t first_press_at = 0;

    explicit Rig(DoorState initial = DoorState::CLOSED)
        : physical(initial)
    {
        test_millis = 1000;
        component.scheduler = &scheduler;
        component.protocol_ = &protocol;
        protocol.setup(&component, &scheduler, &pin, &pin);
        component.received(initial);
        protocol.door_state = initial;
        protocol.maybe_door_state = initial;
        protocol.uart_.respond = [this](uint8_t command) -> optional<uint8_t> {
            if (command == 0x30) {
                if (millis() < deaf_until) {
                    ++ignored_presses;
                    return { };
                }
                if (presses == 0)
                    first_press_at = millis();
                ++presses;
                if (actuate) {
                    if (physical == DoorState::CLOSED)
                        physical = DoorState::OPENING;
                    else if (physical == DoorState::OPEN || physical == DoorState::STOPPED)
                        physical = DoorState::CLOSING;
                    else if (physical == DoorState::OPENING)
                        physical = DoorState::STOPPED;
                    else if (physical == DoorState::CLOSING)
                        physical = reverse_by_stopping ? DoorState::STOPPED : DoorState::OPENING;
                    if (reverse_by_stopping && presses == 3)
                        physical = DoorState::OPENING;
                }
            }
            if (command != 0x38 || drop_queries || (drop_after_press && presses > 0) || millis() < deaf_until)
                return { };
            if (millis() < garble_until)
                return 0xff;
            if (noise_once && presses == 1) {
                noise_once = false;
                return 0x52; // One corrupt but valid OPEN during a missed OPEN.
            }
            return encode(physical);
        };
    }
    static uint8_t encode(DoorState state)
    {
        switch (state) {
        case DoorState::CLOSED:
            return 0x55;
        case DoorState::OPEN:
            return 0x52;
        case DoorState::OPENING:
            return 0x51;
        case DoorState::CLOSING:
            return 0x54;
        case DoorState::STOPPED:
            return 0x50;
        default:
            return 0xff;
        }
    }
    void reply(DoorState state) { protocol.handle_command({ CommandType::QUERY_DOOR_STATUS, encode(state) }); }
    void tick()
    {
        ++test_millis;
        scheduler.run();
        protocol.loop();
        cover.on_door_state(*component.door_state, *component.door_position);
    }
    void run(uint32_t duration)
    {
        const auto end = millis() + duration;
        while (millis() < end)
            tick();
    }
    template <typename F>
    void until(F condition, uint32_t timeout = 15000)
    {
        const auto end = millis() + timeout;
        while (!condition() && millis() < end)
            tick();
        CHECK(condition());
    }
    int sent(uint8_t byte) const
    {
        return std::count_if(protocol.uart_.tx.begin(), protocol.uart_.tx.end(), [byte](auto item) { return item.second == byte; });
    }
    void start_retry()
    {
        component.door_open();
        until([this] { return protocol.door_command_retries_ == 1; });
        CHECK(presses == 1);
    }
};

void miss_with_noise()
{
    Rig rig;
    rig.noise_once = true;
    rig.protocol.door_status_probe_();
    rig.component.door_open();
    rig.until([&] { return rig.presses == 2; });
    CHECK(rig.component.stats.retries == 1);
    CHECK(rig.component.stats.unverified == 0);
}
void retry_stays_bounded()
{
    Rig rig;
    rig.component.door_open();
    rig.run(30000);
    CHECK(rig.presses == 2);
    CHECK(rig.component.stats.retries == 1);
    CHECK(rig.component.stats.retry_failures == 1);
}
void movement_cancels_retry()
{
    Rig rig;
    rig.start_retry();
    rig.physical = DoorState::OPENING;
    rig.reply(DoorState::OPENING);
    rig.reply(DoorState::OPENING);
    rig.run(3000);
    CHECK(rig.presses == 1);
    CHECK(rig.protocol.watched_door_action_ == DoorAction::UNKNOWN);
}
void candidate_blocks_retry()
{
    Rig rig;
    rig.start_retry();
    rig.drop_queries = true;
    rig.reply(DoorState::OPENING);
    rig.run(2000);
    CHECK(rig.presses == 1);
    CHECK(rig.component.stats.unverified == 1);
}
void new_close_cancels_open()
{
    for (bool already_sent : { false, true }) {
        Rig rig;
        rig.component.door_open();
        if (already_sent)
            rig.until([&] { return rig.presses == 1; });
        rig.component.door_close();
        rig.run(15000);
        CHECK(rig.presses == (already_sent ? 1 : 0));
        CHECK(rig.component.stats.retries == 0);
    }
}
void stop_preserves_release()
{
    Rig rig;
    rig.component.door_open();
    rig.until([&] { return rig.presses == 1; });
    rig.component.door_stop();
    rig.run(15000);
    CHECK(rig.presses == 1);
    CHECK(rig.sent(0x31) == 1);
    CHECK(rig.component.stats.retries == 0);
}
void duplicate_suppressed()
{
    Rig rig;
    rig.component.door_open();
    rig.component.door_open();
    rig.until([&] { return rig.presses == 1; });
    rig.component.door_open();
    CHECK(rig.component.stats.duplicates_suppressed == 2);
    rig.physical = DoorState::OPENING;
    rig.run(5000);
    CHECK(rig.presses == 1);
}
void watch_timeout()
{
    Rig rig;
    rig.protocol.wall_panel_emulation_state_ = WallPanelEmulationState::RUNNING;
    rig.component.door_open();
    rig.run(13000);
    CHECK(rig.component.stats.unverified == 1);
    rig.protocol.wall_panel_emulation_state_ = WallPanelEmulationState::WAITING;
    rig.run(3000);
    CHECK(rig.presses == 0);
}
void correction_polls_fast()
{
    Rig rig;
    rig.protocol.door_status_probe_();
    rig.physical = DoorState::OPEN;
    rig.run(3000);
    CHECK(*rig.component.door_state == DoorState::OPEN);
    CHECK(rig.sent(0x38) >= 2);
}
void motion_polls_fast()
{
    Rig rig;
    rig.component.door_open();
    rig.until([&] { return rig.presses == 1; });
    rig.physical = DoorState::OPENING;
    rig.run(5000);
    CHECK(rig.protocol.flags_.door_moving);
    const int before = rig.sent(0x38);
    rig.run(6000);
    CHECK(rig.sent(0x38) >= before + 2);
}
void close_while_opening()
{
    Rig rig(DoorState::OPENING);
    rig.actuate = true;
    rig.component.door_close();
    rig.run(10000);
    CHECK(rig.presses == 2);
    CHECK(*rig.component.door_state == DoorState::CLOSING);
}
void stop_while_closing()
{
    Rig rig(DoorState::CLOSING);
    rig.actuate = true;
    rig.component.door_stop();
    rig.run(10000);
    CHECK(rig.presses == 2);
    CHECK(*rig.component.door_state == DoorState::STOPPED);
}
void open_from_stopped()
{
    for (bool stops_first : { false, true }) {
        Rig rig(DoorState::STOPPED);
        rig.actuate = true;
        rig.reverse_by_stopping = stops_first;
        rig.component.door_open();
        rig.run(18000);
        CHECK(rig.presses == (stops_first ? 3 : 2));
        CHECK(*rig.component.door_state == DoorState::OPENING);
    }
}
void supersede_direction_sequence()
{
    Rig rig(DoorState::OPENING);
    rig.component.door_close();
    rig.until([&] { return rig.presses == 1; });
    rig.component.door_open(); // Still reported OPENING: cancel pending CLOSE.
    rig.physical = DoorState::STOPPED;
    rig.run(15000);
    CHECK(rig.presses == 1);
}
void deferred_stop()
{
    Rig rig(DoorState::UNKNOWN);
    rig.component.door_open();
    rig.component.door_stop();
    rig.physical = DoorState::CLOSED;
    rig.reply(DoorState::CLOSED);
    rig.reply(DoorState::CLOSED);
    rig.run(3000);
    CHECK(rig.presses == 0);
    CHECK(rig.protocol.pending_door_action_ == DoorAction::UNKNOWN);
}
void passive_panel()
{
    Rig rig;
    rig.protocol.handle_command({ CommandType::QUERY_DOOR_STATUS_0x37, 0 });
    const auto before = rig.sent(0x38);
    rig.protocol.door_status_probe_();
    rig.run(15000);
    CHECK(rig.sent(0x38) == before);
    rig.component.door_open();
    rig.run(15000);
    CHECK(rig.presses == 1);
    CHECK(rig.component.stats.retries == 0);
}

void warmup_queue_bounded()
{
    Rig rig;
    rig.component.query_status(); // Preserve independent diagnostic queries.
    for (int i = 0; i < 20; ++i)
        rig.component.door_toggle();
    CHECK(rig.protocol.pending_tx_.size() == 7); // 4 diagnostics + 3 warm-ups; press waits at the gate.
    rig.run(5000);
    CHECK(rig.presses == 1);
    CHECK(rig.sent(0x39) == 1);
    CHECK(rig.sent(0x3a) == 1);
}

void boot_and_deferred_actions()
{
    for (DoorState state : { DoorState::CLOSED, DoorState::OPEN }) {
        Rig rig(DoorState::UNKNOWN);
        rig.physical = state;
        rig.protocol.sync();
        rig.run(11000);
        CHECK(*rig.component.door_state == state);
        CHECK(rig.presses == 0);
    }
    for (DoorState state : { DoorState::CLOSED, DoorState::OPEN }) {
        Rig rig(DoorState::UNKNOWN);
        rig.physical = state;
        rig.actuate = true;
        rig.protocol.sync();
        if (state == DoorState::CLOSED)
            rig.component.door_open();
        else
            rig.component.door_close();
        rig.run(11000);
        CHECK(rig.presses == 1);
        CHECK(*rig.component.door_state == (state == DoorState::CLOSED ? DoorState::OPENING : DoorState::CLOSING));
    }
}

void direction_waits_for_confirmation()
{
    Rig rig(DoorState::OPENING);
    rig.component.door_close();
    rig.until([&] { return rig.presses == 1; });
    rig.reply(DoorState::STOPPED); // A lone corrupt read must not trigger CLOSE.
    rig.run(300);
    CHECK(rig.presses == 1);
    rig.component.door_stop(); // Supersede and clear the old continuation.
    rig.physical = DoorState::STOPPED;
    rig.run(15000);
    CHECK(rig.presses == 2); // Only the operator's STOP, no old CLOSE.
}

void timeout_owners_independent()
{
    Rig rig(DoorState::OPENING);
    rig.component.door_close();
    rig.component.on_door_state([](DoorState) { });
    CHECK(rig.scheduler.timers.count({ &rig.component, TIMEOUT_DOOR_STATE_EXPIRY }) == 1);
    CHECK(rig.scheduler.timers.count({ &rig.component, TIMEOUT_SECPLUS1_DOOR_STATE_EXPIRY }) == 1);
    rig.component.cancel_door_state_expiry();
    CHECK(rig.scheduler.timers.count({ &rig.component, TIMEOUT_SECPLUS1_DOOR_STATE_EXPIRY }) == 1);
}

void confirm(Rig& rig, DoorState state)
{
    rig.physical = state;
    rig.reply(state);
    rig.reply(state);
    rig.cover.on_door_state(*rig.component.door_state, *rig.component.door_position);
}

void opening_at_estimated_open()
{
    Rig rig(DoorState::OPEN);
    rig.component.opening_duration = 13.6f;
    rig.component.closing_duration = 13.3f;
    confirm(rig, DoorState::OPENING);
    rig.run(14000);
    CHECK(rig.cover.current_operation == COVER_OPERATION_OPENING);
    CHECK(*rig.component.door_position == COVER_OPEN);
    CHECK(rig.cover.position == 0.99f);
    confirm(rig, DoorState::STOPPED);
    CHECK(rig.cover.current_operation == COVER_OPERATION_IDLE);
    CHECK(rig.cover.position == 0.99f);
    CHECK(rig.presses == 0);
}

void slider_direction_mismatch()
{
    for (bool opening : { true, false }) {
        Rig rig(DoorState::STOPPED);
        rig.component.opening_duration = 13.6f;
        rig.component.closing_duration = 13.3f;
        const float start = opening ? 0.8f : 0.2f;
        rig.component.door_position = start;
        rig.cover.control({ opening ? 0.2f : 0.8f });
        // Isolate the recorded input sequence; this does not claim the opener
        // necessarily reverses on this command. Replies traverse the real gate.
        confirm(rig, opening ? DoorState::OPENING : DoorState::CLOSING);
        rig.run(5000);
        CHECK(rig.cover.current_operation == (opening ? COVER_OPERATION_OPENING : COVER_OPERATION_CLOSING));
        CHECK(opening ? rig.cover.position >= start : rig.cover.position <= start);
        confirm(rig, DoorState::STOPPED);
        CHECK(opening ? rig.cover.position >= start : rig.cover.position <= start);
    }
}

void slider_pending_target_does_not_change_estimator()
{
    Rig rig(DoorState::OPEN);
    rig.component.closing_duration = 13.3f;
    const auto delta = rig.component.door_move_delta;
    rig.cover.control({ 0.8f });
    rig.cover.control({ 0.6f });
    CHECK(rig.component.door_move_delta == delta);
    CHECK(rig.presses == 0); // Warm-up/press has not reached the bus yet.
}

void position_reversal_and_stop()
{
    Rig rig;
    rig.component.opening_duration = 10;
    rig.component.closing_duration = 20;
    confirm(rig, DoorState::OPENING);
    rig.run(4000);
    CHECK(std::abs(rig.cover.position - 0.4f) < 0.001f);
    confirm(rig, DoorState::CLOSING);
    rig.run(2000);
    CHECK(std::abs(rig.cover.position - 0.3f) < 0.001f);
    confirm(rig, DoorState::STOPPED);
    const auto stopped = rig.cover.position;
    rig.run(5000);
    CHECK(rig.cover.position == stopped);
    confirm(rig, DoorState::CLOSED);
    CHECK(rig.cover.position == COVER_CLOSED);
}

void partial_open_stops_between_endpoints()
{
    Rig rig;
    rig.actuate = true;
    rig.component.opening_duration = 13.6f;
    rig.component.closing_duration = 13.3f;
    rig.cover.control({ 0.5f });
    rig.run(20000);
    CHECK(rig.presses == 2); // One start and one timed STOP, including releases.
    CHECK(rig.sent(0x31) == 2);
    CHECK(*rig.component.door_state == DoorState::STOPPED);
    CHECK(rig.cover.current_operation == COVER_OPERATION_IDLE);
    CHECK(rig.cover.position > COVER_CLOSED && rig.cover.position < COVER_OPEN);
}

// Regression coverage for request-time STOP and false closed endpoint reporting.
void partial_stop_before_confirmation()
{
    Rig rig;
    rig.actuate = true;
    rig.component.opening_duration = 13.6f;
    rig.cover.control({ 0.02f });
    rig.run(2000);
    CHECK(rig.presses == 1); // Warm-up survives; no STOP before confirmation.
}

void partial_timer_survives_new_command()
{
    Rig rig;
    rig.actuate = true;
    rig.component.opening_duration = 13.6f;
    rig.component.closing_duration = 13.3f;
    rig.cover.control({ 0.8f });
    rig.component.door_open();
    rig.run(12000);
    CHECK(rig.presses == 1); // Old slider STOP must not interrupt the newer OPEN.
}

void stopped_at_estimated_closed()
{
    Rig rig(DoorState::OPEN);
    rig.component.closing_duration = 13.3f;
    confirm(rig, DoorState::CLOSING);
    rig.run(14000);
    confirm(rig, DoorState::STOPPED);
    CHECK(rig.cover.position > COVER_CLOSED); // No CLOSED end-state was received.
    CHECK(!rig.cover.saved);
    confirm(rig, DoorState::CLOSED);
    CHECK(rig.cover.position == COVER_CLOSED);
    CHECK(rig.cover.saved);
}

void partial_accounts_for_confirmation_delay()
{
    Rig rig;
    rig.drop_after_press = true;
    rig.component.opening_duration = 10;
    rig.component.closing_duration = 10;
    rig.cover.control({ 0.5f });
    rig.run(1000);
    CHECK(rig.presses == 1);
    CHECK(rig.scheduler.timers.count({ &rig.component, TIMEOUT_MOVE_TO_POSITION }) == 0);
    auto first_reply = millis();
    rig.reply(DoorState::OPENING);
    rig.run(2500);
    CHECK(rig.scheduler.timers.count({ &rig.component, TIMEOUT_MOVE_TO_POSITION }) == 0);
    rig.reply(DoorState::OPENING);
    CHECK(rig.component.door_start_moving == first_reply);
    CHECK(std::abs(*rig.component.door_position - 0.25f) < 0.001f);
    CHECK(rig.scheduler.timers.at({ &rig.component, TIMEOUT_MOVE_TO_POSITION }).at == first_reply + 5000);
    rig.run(2499);
    CHECK(rig.presses == 1);
    rig.run(400);
    CHECK(rig.presses == 2);
}

void partial_waits_through_direction_sequence()
{
    Rig rig(DoorState::STOPPED);
    rig.drop_queries = true;
    rig.component.door_position = 0.5f;
    rig.component.opening_duration = 10;
    rig.component.closing_duration = 10;
    rig.cover.control({ 0.8f });
    rig.run(1000);
    confirm(rig, DoorState::CLOSING);
    rig.run(3000);
    CHECK(rig.component.position_target_.has_value());
    CHECK(rig.scheduler.timers.count({ &rig.component, TIMEOUT_MOVE_TO_POSITION }) == 0);
    confirm(rig, DoorState::STOPPED);
    CHECK(rig.component.position_target_.has_value());
    rig.run(600);
    confirm(rig, DoorState::OPENING);
    CHECK(rig.component.position_target_tracking_);
    CHECK(rig.scheduler.timers.count({ &rig.component, TIMEOUT_MOVE_TO_POSITION }) == 1);
    confirm(rig, DoorState::STOPPED); // Early physical stop cancels this target.
    CHECK(!rig.component.position_target_.has_value());
    auto presses = rig.presses;
    rig.run(15000);
    CHECK(rig.presses == presses);
}

void partial_latest_target_while_moving()
{
    Rig rig(DoorState::OPENING);
    rig.component.door_position = 0.4f;
    rig.component.opening_duration = 10;
    rig.component.closing_duration = 10;
    rig.cover.control({ 0.2f });
    rig.cover.control({ 0.7f });
    confirm(rig, DoorState::STOPPED);
    CHECK(rig.component.position_target_ == 0.7f);
    CHECK(rig.component.position_target_direction_ == DoorState::OPENING);
}

void partial_unverified_expires()
{
    Rig rig;
    rig.drop_queries = true;
    rig.component.opening_duration = 10;
    rig.cover.control({ 0.5f });
    rig.run(46000);
    CHECK(!rig.component.position_target_.has_value());
    CHECK(rig.presses == 1);
    CHECK(rig.scheduler.timers.count({ &rig.component, TIMEOUT_MOVE_TO_POSITION }) == 0);
}

void partial_new_command_cancels_active_timer()
{
    Rig rig;
    rig.actuate = true;
    rig.component.opening_duration = 10;
    rig.cover.control({ 0.8f });
    rig.until([&] { return rig.component.position_target_tracking_; });
    CHECK(rig.scheduler.timers.count({ &rig.component, TIMEOUT_MOVE_TO_POSITION }) == 1);
    rig.cover.control({ 1.0f });
    rig.run(15000);
    CHECK(rig.presses == 1);
    CHECK(!rig.component.position_target_.has_value());
    CHECK(*rig.component.door_state == DoorState::OPENING);
}

void endpoints_require_status()
{
    for (bool opening : { true, false }) {
        Rig rig(opening ? DoorState::CLOSED : DoorState::OPEN);
        rig.component.opening_duration = 10;
        rig.component.closing_duration = 10;
        rig.drop_queries = true;
        rig.cover.control({ opening ? 1.0f : 0.0f });
        confirm(rig, opening ? DoorState::OPENING : DoorState::CLOSING);
        rig.run(16000);
        CHECK(*rig.component.door_state == (opening ? DoorState::OPENING : DoorState::CLOSING));
        CHECK(rig.cover.position > 0 && rig.cover.position < 1);
    }
}

void partial_rejects_untrusted_start()
{
    for (float position : { 0.0f, 1.0f, DOOR_POSITION_UNKNOWN }) {
        Rig rig(DoorState::STOPPED);
        rig.component.door_position = position;
        rig.component.opening_duration = 10;
        rig.component.closing_duration = 10;
        rig.cover.control({ 0.5f });
        rig.run(2000);
        CHECK(rig.presses == 0);
        CHECK(!rig.component.position_target_.has_value());
    }
}

void partial_satisfied_cancels_queued_start()
{
    Rig rig(DoorState::STOPPED);
    rig.component.door_position = 0.4f;
    rig.component.opening_duration = 10;
    rig.cover.control({ 0.8f });
    rig.cover.control({ 0.4f });
    rig.run(15000);
    CHECK(rig.presses == 0);
    CHECK(!rig.component.position_target_.has_value());
}

void partial_abort_preserves_release()
{
    Rig rig;
    rig.drop_queries = true;
    rig.component.opening_duration = 10;
    rig.cover.control({ 0.5f });
    rig.until([&] { return rig.presses == 1; });
    CHECK(rig.sent(0x31) == 0);
    confirm(rig, DoorState::OPENING);
    confirm(rig, DoorState::STOPPED);
    rig.run(2000);
    CHECK(rig.presses == 1);
    CHECK(rig.sent(0x31) == 1);
}

void stale_candidate_does_not_backdate_position()
{
    Rig rig;
    rig.drop_queries = true;
    rig.component.opening_duration = 10;
    rig.reply(DoorState::OPENING);
    rig.run(11000);
    rig.reply(DoorState::OPENING);
    CHECK(rig.component.door_start_moving == millis());
    rig.component.door_position_update();
    CHECK(*rig.component.door_position == 0.0f);
}

// Press gate: hold a stationary-door press until the opener answers again.
void gate_keeps_healthy_timing()
{
    Rig rig;
    rig.actuate = true;
    rig.protocol.uart_.echo_unanswered = true;
    const auto start = millis();
    rig.component.door_open();
    rig.until([&] { return rig.presses == 1; });
    CHECK(rig.first_press_at - start <= 700); // Same as the fixed 3-query warm-up.
    CHECK(rig.sent(0x38) == 3);
}

void gate_waits_out_deaf_window()
{
    Rig rig;
    rig.actuate = true;
    rig.protocol.uart_.echo_unanswered = true;
    rig.deaf_until = millis() + 4000;
    rig.component.door_open();
    rig.run(15000);
    CHECK(rig.ignored_presses == 0);
    CHECK(rig.presses == 1);
    CHECK(rig.first_press_at >= rig.deaf_until);
    CHECK(rig.first_press_at <= rig.deaf_until + 1000);
    CHECK(*rig.component.door_state == DoorState::OPENING);
    CHECK(rig.component.stats.retries == 0);
    CHECK(rig.component.stats.unverified == 0);
}

void gate_needs_consecutive_valid_replies()
{
    Rig rig;
    rig.actuate = true;
    rig.garble_until = millis() + 2000;
    rig.component.door_open();
    rig.run(10000);
    CHECK(rig.presses == 1);
    CHECK(rig.first_press_at >= rig.garble_until + 200); // Two clean replies after the garble.
    CHECK(rig.component.stats.unverified == 0);
}

void gate_deadline_presses_anyway()
{
    Rig rig;
    rig.actuate = true;
    rig.protocol.uart_.echo_unanswered = true;
    const auto start = millis();
    rig.deaf_until = start + 60000;
    rig.component.door_open();
    rig.run(10000 - 1);
    CHECK(rig.ignored_presses == 0);
    rig.run(1000);
    CHECK(rig.ignored_presses == 1); // Pressed at the deadline.
    rig.run(20000);
    CHECK(rig.ignored_presses == 1); // No retry without valid baseline reads.
    CHECK(rig.component.stats.unverified == 1);
}

void gated_retry_outlasts_watch()
{
    Rig rig;
    rig.protocol.uart_.echo_unanswered = true;
    rig.start_retry(); // First press heard but ignored; retry starts its gate.
    rig.actuate = true;
    rig.deaf_until = millis() + 9000;
    rig.run(20000);
    CHECK(rig.presses == 2);
    CHECK(*rig.component.door_state == DoorState::OPENING);
    CHECK(rig.component.stats.retries == 1);
    CHECK(rig.component.stats.retry_failures == 0);
    CHECK(rig.component.stats.unverified == 0);
}

void gated_direction_sequence_keeps_callback()
{
    Rig rig(DoorState::STOPPED);
    rig.actuate = true;
    rig.protocol.uart_.echo_unanswered = true;
    rig.deaf_until = millis() + 9000;
    rig.component.door_open(); // Press, confirm CLOSING, press again to reverse.
    rig.run(25000);
    CHECK(rig.presses == 2);
    CHECK(*rig.component.door_state == DoorState::OPENING);
}

void cancel_drops_gated_press()
{
    Rig rig;
    rig.protocol.uart_.echo_unanswered = true;
    rig.deaf_until = millis() + 3000;
    rig.component.door_open();
    rig.run(1000);
    rig.component.door_stop(); // Stationary STOP supersedes the waiting OPEN.
    rig.run(15000);
    CHECK(rig.presses == 0);
    CHECK(rig.ignored_presses == 0);
}

// A query that reaches the bus as 0x30 is a door press to the opener. It must
// not be reported as a wall-panel button, and the door is watched closely.
void echo_read_as_press()
{
    Rig rig;
    rig.protocol.uart_.corrupt_next_echo = 0x30;
    rig.protocol.door_status_probe_();
    rig.run(50);
    CHECK(rig.sent(0x38) == 1);
    CHECK(rig.component.stats.tx_echo_errors == 1);
    CHECK(rig.component.button_presses == 0);
    CHECK(*rig.component.door_state == DoorState::CLOSED);
    rig.physical = DoorState::OPENING; // the opener acted on the corrupted byte
    rig.run(6000);
    CHECK(*rig.component.door_state == DoorState::OPENING);
    CHECK(rig.presses == 0);
}

// Garbage that is not a panel command is dropped without raising the alarm,
// and the next query is parsed normally.
void echo_garbage_dropped()
{
    Rig rig;
    rig.protocol.uart_.corrupt_next_echo = 0x70;
    rig.protocol.door_status_probe_();
    rig.run(50);
    CHECK(rig.component.stats.tx_echo_errors == 0);
    CHECK(rig.component.button_presses == 0);
    rig.physical = DoorState::OPEN;
    rig.protocol.door_status_probe_();
    rig.run(6000);
    CHECK(*rig.component.door_state == DoorState::OPEN);
}

// A reply byte that happens to follow a lost echo is not taken for a command.
void echo_lost_then_reply()
{
    Rig rig;
    rig.protocol.uart_.corrupt_next_echo = 0x30;
    rig.protocol.door_status_probe_();
    rig.protocol.uart_.rx.push_back(0x55);
    rig.run(200);
    CHECK(rig.component.stats.tx_echo_errors == 1);
    CHECK(rig.component.button_presses == 0);
    CHECK(*rig.component.door_state == DoorState::CLOSED);
}

// A settled door is re-checked every 5 minutes, not every 10 seconds.
void idle_probe_is_slow()
{
    Rig rig;
    rig.protocol.door_status_probe_();
    rig.run(299000);
    CHECK(rig.sent(0x38) == 1);
    rig.run(2000);
    CHECK(rig.sent(0x38) == 2);
    CHECK(*rig.component.door_state == DoorState::CLOSED);
}

// A real wall-panel press is still reported, and the movement it causes is
// picked up within seconds despite the slow idle rate.
void wall_press_probes_fast()
{
    Rig rig;
    rig.protocol.door_status_probe_();
    rig.run(1000);
    rig.protocol.uart_.rx.push_back(0x30);
    rig.physical = DoorState::OPENING;
    rig.run(6000);
    CHECK(rig.component.button_presses == 1);
    CHECK(rig.component.stats.tx_echo_errors == 0);
    CHECK(*rig.component.door_state == DoorState::OPENING);
    rig.physical = DoorState::OPEN;
    rig.run(6000);
    CHECK(*rig.component.door_state == DoorState::OPEN);
    const auto queries = rig.sent(0x38);
    rig.run(60000);
    CHECK(rig.sent(0x38) - queries <= 6); // fast window ends, back to idle
}

int main(int argc, char** argv)
{
    const std::map<std::string, void (*)()> scenarios = {
        { "miss_with_noise", miss_with_noise },
        { "retry_stays_bounded", retry_stays_bounded },
        { "movement_cancels_retry", movement_cancels_retry },
        { "candidate_blocks_retry", candidate_blocks_retry },
        { "new_close_cancels_open", new_close_cancels_open },
        { "stop_preserves_release", stop_preserves_release },
        { "duplicate_suppressed", duplicate_suppressed },
        { "watch_timeout", watch_timeout },
        { "correction_polls_fast", correction_polls_fast },
        { "motion_polls_fast", motion_polls_fast },
        { "close_while_opening", close_while_opening },
        { "stop_while_closing", stop_while_closing },
        { "open_from_stopped", open_from_stopped },
        { "supersede_direction_sequence", supersede_direction_sequence },
        { "deferred_stop", deferred_stop },
        { "passive_panel", passive_panel },
        { "warmup_queue_bounded", warmup_queue_bounded },
        { "boot_and_deferred_actions", boot_and_deferred_actions },
        { "direction_waits_for_confirmation", direction_waits_for_confirmation },
        { "timeout_owners_independent", timeout_owners_independent },
        { "opening_at_estimated_open", opening_at_estimated_open },
        { "slider_direction_mismatch", slider_direction_mismatch },
        { "slider_pending_target_does_not_change_estimator", slider_pending_target_does_not_change_estimator },
        { "position_reversal_and_stop", position_reversal_and_stop },
        { "partial_open_stops_between_endpoints", partial_open_stops_between_endpoints },
        { "partial_stop_before_confirmation", partial_stop_before_confirmation },
        { "partial_timer_survives_new_command", partial_timer_survives_new_command },
        { "stopped_at_estimated_closed", stopped_at_estimated_closed },
        { "partial_accounts_for_confirmation_delay", partial_accounts_for_confirmation_delay },
        { "partial_waits_through_direction_sequence", partial_waits_through_direction_sequence },
        { "partial_latest_target_while_moving", partial_latest_target_while_moving },
        { "partial_unverified_expires", partial_unverified_expires },
        { "partial_new_command_cancels_active_timer", partial_new_command_cancels_active_timer },
        { "endpoints_require_status", endpoints_require_status },
        { "partial_rejects_untrusted_start", partial_rejects_untrusted_start },
        { "partial_satisfied_cancels_queued_start", partial_satisfied_cancels_queued_start },
        { "partial_abort_preserves_release", partial_abort_preserves_release },
        { "stale_candidate_does_not_backdate_position", stale_candidate_does_not_backdate_position },
        { "gate_keeps_healthy_timing", gate_keeps_healthy_timing },
        { "gate_waits_out_deaf_window", gate_waits_out_deaf_window },
        { "gate_needs_consecutive_valid_replies", gate_needs_consecutive_valid_replies },
        { "gate_deadline_presses_anyway", gate_deadline_presses_anyway },
        { "gated_retry_outlasts_watch", gated_retry_outlasts_watch },
        { "gated_direction_sequence_keeps_callback", gated_direction_sequence_keeps_callback },
        { "cancel_drops_gated_press", cancel_drops_gated_press },
        { "echo_read_as_press", echo_read_as_press },
        { "echo_garbage_dropped", echo_garbage_dropped },
        { "echo_lost_then_reply", echo_lost_then_reply },
        { "idle_probe_is_slow", idle_probe_is_slow },
        { "wall_press_probes_fast", wall_press_probes_fast },
    };
    try {
        CHECK(argc == 2);
        scenarios.at(argv[1])();
        std::cout << argv[1] << " passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
