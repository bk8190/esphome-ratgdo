// Host tests for the checked Sec+1 query sender: a simulated wired-AND line
// with interference, and a simulated opener UART that decodes whatever the
// line carried. The property under test: no interference turns a query into a
// one-byte panel command.

#include "secplus1_query_tx.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

using namespace esphome::ratgdo::secplus1;

#define CHECK(condition)                                                         \
    do {                                                                         \
        if (!(condition))                                                        \
            throw std::runtime_error(std::string(__func__) + ": " + #condition); \
    } while (false)

static constexpr double BIT = 10000.0 / 12.0;

struct Interval {
    int64_t from;
    int64_t to; // exclusive
};

// The line reads low while we drive it, for `rise` microseconds after we
// release it, and during an interfering pulse.
struct SimBus {
    int64_t now;
    int64_t step = 5;
    int64_t rise = 0;
    Interval pulse { 0, 0 };
    // A stall models a non-maskable interrupt: time jumps while we do nothing.
    int64_t stall_at = INT64_MAX;
    int64_t stall_len = 0;
    bool driving = false;
    int64_t released_at = INT64_MIN / 2;
    int64_t drive_start = 0;
    std::vector<Interval> driven;

    explicit SimBus(int64_t start)
        : now(start)
    {
    }
    uint32_t micros()
    {
        now += step;
        if (now >= stall_at) {
            now += stall_len;
            stall_at = INT64_MAX;
        }
        return static_cast<uint32_t>(now);
    }
    void drive(bool low)
    {
        if (low && !driving) {
            drive_start = now;
        } else if (!low && driving) {
            driven.push_back({ drive_start, now + rise });
            released_at = now;
        }
        driving = low;
    }
    bool low() const
    {
        return driving || now < released_at + rise || (now >= pulse.from && now < pulse.to);
    }
    std::vector<Interval> line() const
    {
        auto all = driven;
        if (pulse.to > pulse.from) {
            all.push_back(pulse);
        }
        std::sort(all.begin(), all.end(), [](const Interval& a, const Interval& b) { return a.from < b.from; });
        std::vector<Interval> merged;
        for (const auto& item : all) {
            if (!merged.empty() && item.from <= merged.back().to) {
                merged.back().to = std::max(merged.back().to, item.to);
            } else {
                merged.push_back(item);
            }
        }
        return merged;
    }
};

// Reference sender with no checks: absolute deadlines, drive and hope.
template <typename Bus>
void send_unchecked(Bus& bus, uint8_t byte)
{
    const uint16_t frame = query_tx::frame_bits(byte);
    const uint32_t t0 = bus.micros();
    for (uint8_t bit = 0; bit < query_tx::FRAME_BITS; ++bit) {
        bus.drive(!((frame >> bit) & 1));
        while (bus.micros() - t0 < query_tx::bit_us(bit + 1)) {
        }
    }
}

struct Opener {
    double sample = 0.5; // where in each bit it samples
    double skew = 1.0; // its bit clock relative to ours
    bool check_stop = true;
    bool level_start = false; // a low line starts a frame without an edge
};

static bool line_low(const std::vector<Interval>& line, double t)
{
    for (const auto& item : line) {
        if (t >= item.from && t < item.to) {
            return true;
        }
    }
    return false;
}

// Every byte the opener would decode from the line. Parity is not checked:
// the field events show the opener acting on a byte with a bad parity bit.
static std::vector<uint8_t> decode(const std::vector<Interval>& line, const Opener& opener)
{
    std::vector<uint8_t> bytes;
    const double bit = BIT * opener.skew;
    double from = -1e18;
    size_t index = 0;
    while (true) {
        double start;
        if (opener.level_start && line_low(line, from)) {
            start = from;
        } else {
            while (index < line.size() && line[index].from < from) {
                ++index;
            }
            if (index == line.size()) {
                break;
            }
            start = static_cast<double>(line[index].from);
        }
        if (!line_low(line, start + opener.sample * bit)) {
            from = start + 1; // glitch shorter than the start-bit check
            continue;
        }
        uint8_t byte = 0;
        for (int data = 0; data < 8; ++data) {
            if (!line_low(line, start + (1 + data + opener.sample) * bit)) {
                byte |= static_cast<uint8_t>(1U << data);
            }
        }
        const bool stop_ok = !line_low(line, start + (10 + opener.sample) * bit);
        if (stop_ok || !opener.check_stop) {
            bytes.push_back(byte);
        }
        from = start + (10 + opener.sample) * bit;
    }
    return bytes;
}

static bool is_panel_command(uint8_t byte) { return byte == 0x37 || (byte >= 0x30 && byte <= 0x35); }

static std::vector<Opener> openers()
{
    std::vector<Opener> all;
    for (double sample : { 0.42, 0.5, 0.6 }) {
        for (double skew : { 0.98, 1.0, 1.02 }) {
            for (bool check_stop : { true, false }) {
                for (bool level_start : { false, true }) {
                    all.push_back({ sample, skew, check_stop, level_start });
                }
            }
        }
    }
    return all;
}

static bool any_command(const SimBus& bus)
{
    const auto line = bus.line();
    for (const auto& opener : openers()) {
        for (uint8_t byte : decode(line, opener)) {
            if (is_panel_command(byte)) {
                return true;
            }
        }
    }
    return false;
}

static constexpr int64_t START = 1000000;
// Where the frame begins once the listen window has passed.
static int64_t frame_start() { return START + query_tx::LISTEN_US + 10; }

void clean_frame_is_delivered()
{
    for (uint8_t byte : { 0x38, 0x39, 0x3A }) {
        for (int64_t rise : { 0, 100, 200 }) {
            SimBus bus(START);
            bus.rise = rise;
            QueryTxInfo info;
            CHECK(send_query_checked(bus, byte, &info) == QueryTxResult::SENT);
            CHECK(info.spoiled_at == -1);
            CHECK(info.rise_us >= rise && info.rise_us <= rise + 2 * bus.step);
            for (const auto& opener : openers()) {
                // A slow line with an early-sampling, fast-clocked opener can
                // lose the stop bit after a low parity bit; that is the
                // line's margin, not the sender's.
                if (rise > 100 && opener.skew != 1.0) {
                    continue;
                }
                const auto bytes = decode(bus.line(), opener);
                if (!(bytes.size() == 1 && bytes[0] == byte)) {
                    std::cerr << "byte " << int(byte) << " rise " << rise << " sample " << opener.sample << " skew " << opener.skew
                              << " stop " << opener.check_stop << " level " << opener.level_start << " decoded " << bytes.size() << '\n';
                }
                CHECK(bytes.size() == 1 && bytes[0] == byte);
            }
        }
    }
}

void timer_wraparound()
{
    SimBus bus(0xFFFFF000LL);
    CHECK(send_query_checked(bus, 0x38) == QueryTxResult::SENT);
    const auto bytes = decode(bus.line(), Opener { });
    CHECK(bytes.size() == 1 && bytes[0] == 0x38);
}

void busy_line_is_left_alone()
{
    SimBus bus(START);
    bus.pulse = { START + 200, START + 600 };
    CHECK(send_query_checked(bus, 0x38) == QueryTxResult::BUSY);
    CHECK(bus.driven.empty());
}

// The failure in the field: a short low pulse on bit 3 of 0x38 is a press.
void unchecked_sender_can_emit_press()
{
    SimBus bus(frame_start());
    const int64_t bit3 = bus.now + static_cast<int64_t>(4 * BIT);
    bus.pulse = { bit3 + 100, bit3 + 700 };
    send_unchecked(bus, 0x38);
    const auto bytes = decode(bus.line(), Opener { });
    CHECK(bytes.size() == 1 && bytes[0] == 0x30);

    // Same result from a late edge: a stall across the bit 2 -> 3 boundary.
    SimBus late(frame_start());
    late.stall_at = late.now + static_cast<int64_t>(4 * BIT) - 20;
    late.stall_len = 600;
    send_unchecked(late, 0x38);
    const auto late_bytes = decode(late.line(), Opener { });
    CHECK(late_bytes.size() == 1 && late_bytes[0] == 0x30);
}

void pulse_never_makes_a_command()
{
    int spoiled = 0, sent = 0, busy = 0;
    for (uint8_t byte : { 0x38, 0x39, 0x3A }) {
        for (int64_t rise : { 0, 100, 200 }) {
            for (int64_t width : { 30, 100, 200, 417, 600, 833, 1000, 1700, 2600 }) {
                int variant = 0;
                for (int64_t at = -2600; at < 12500; at += 17) {
                    SimBus bus(START + (variant++ % 5));
                    bus.rise = rise;
                    bus.pulse = { frame_start() + at, frame_start() + at + width + rise };
                    const auto result = send_query_checked(bus, byte);
                    spoiled += result == QueryTxResult::SPOILED;
                    sent += result == QueryTxResult::SENT;
                    busy += result == QueryTxResult::BUSY;
                    CHECK(!any_command(bus));
                }
            }
        }
    }
    // The sweep must exercise every outcome to mean anything.
    CHECK(spoiled > 1000 && sent > 1000 && busy > 1000);
}

void stall_never_makes_a_command()
{
    int spoiled = 0;
    for (uint8_t byte : { 0x38, 0x39, 0x3A }) {
        for (int64_t length : { 150, 300, 450, 600, 900, 1300, 2500 }) {
            for (int64_t at = -900; at < 9500; at += 13) {
                SimBus bus(START);
                bus.stall_at = frame_start() + at;
                bus.stall_len = length;
                spoiled += send_query_checked(bus, byte) == QueryTxResult::SPOILED;
                CHECK(!any_command(bus));
            }
        }
    }
    CHECK(spoiled > 1000);
}

void stall_with_pulse_never_makes_a_command()
{
    for (int64_t length : { 150, 600 }) {
        for (int64_t stall = 2800; stall < 5200; stall += 61) {
            for (int64_t at = 2600; at < 5400; at += 37) {
                SimBus bus(START);
                bus.stall_at = frame_start() + stall;
                bus.stall_len = length;
                bus.pulse = { frame_start() + at, frame_start() + at + 500 };
                send_query_checked(bus, 0x38);
                CHECK(!any_command(bus));
            }
        }
    }
}

// The supervised hardware test deliberately spoils frames at each data bit.
void forced_spoil_is_not_a_command()
{
    for (int8_t data_bit = 0; data_bit < 8; ++data_bit) {
        SimBus bus(START);
        QueryTxInfo info;
        CHECK(send_query_checked(bus, 0x38, &info, data_bit) == QueryTxResult::SPOILED);
        CHECK(info.spoiled_at == data_bit + 1);
        CHECK(!any_command(bus));
        // An opener that checks the stop bit sees nothing at all.
        CHECK(decode(bus.line(), Opener { }).empty());
    }
}

void slow_line_is_spoiled()
{
    SimBus bus(START);
    bus.rise = 400; // slower than RISE_DEADLINE_US
    QueryTxInfo info;
    CHECK(send_query_checked(bus, 0x38, &info) == QueryTxResult::SPOILED);
    CHECK(info.spoiled_at == 4);
}

int main(int argc, char** argv)
{
    const std::map<std::string, void (*)()> scenarios {
        { "clean_frame_is_delivered", clean_frame_is_delivered },
        { "timer_wraparound", timer_wraparound },
        { "busy_line_is_left_alone", busy_line_is_left_alone },
        { "unchecked_sender_can_emit_press", unchecked_sender_can_emit_press },
        { "pulse_never_makes_a_command", pulse_never_makes_a_command },
        { "stall_never_makes_a_command", stall_never_makes_a_command },
        { "stall_with_pulse_never_makes_a_command", stall_with_pulse_never_makes_a_command },
        { "forced_spoil_is_not_a_command", forced_spoil_is_not_a_command },
        { "slow_line_is_spoiled", slow_line_is_spoiled },
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
