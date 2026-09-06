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
    TestProtocol protocol;
    InternalGPIOPin pin;
    DoorState physical;
    bool actuate = false;
    bool noise_once = false;
    bool drop_queries = false;
    bool reverse_by_stopping = false;
    int presses = 0;

    explicit Rig(DoorState initial = DoorState::CLOSED)
        : physical(initial)
    {
        test_millis = 1000;
        component.scheduler = &scheduler;
        component.protocol_ = &protocol;
        protocol.setup(&component, &scheduler, &pin, &pin);
        component.door_state = initial;
        protocol.door_state = initial;
        protocol.maybe_door_state = initial;
        protocol.uart_.respond = [this](uint8_t command) -> optional<uint8_t> {
            if (command == 0x30) {
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
            if (command != 0x38 || drop_queries)
                return { };
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
    CHECK(rig.protocol.pending_tx_.size() == 8); // 4 diagnostics + 3 warm-ups + press.
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
