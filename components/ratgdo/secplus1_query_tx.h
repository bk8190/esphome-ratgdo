#pragma once

// Security+ 1.0 query sender that cannot put a wall-panel command on the bus.
//
// A status query (0x38) differs from a door press (0x30) in one bit, and the
// bus is wired-AND: anything that holds the line low while we send bit 3 turns
// the query into a press. Field logs show short low pulses on the bus about
// every 102 ms doing exactly that roughly once in 40,000-100,000 queries.
//
// This sender bit-bangs the frame against absolute deadlines and reads the bus
// back while it is releasing a data bit. If the bus is low when it should be
// high, or an edge went out late, it abandons the frame: it releases the line
// and drives nothing more. Every panel command has bits 6 and 7 clear, and
// those are the last bits a query drives low, so an abandoned query reads
// with bits 6 and 7 set unless something else pulls them low separately. The
// line then carries a low pulse of a few bit times, which the bus already
// carries naturally.
//
// When to start. With nothing talking to it, the opener's bus carries a low
// pulse of about 200 us every 12.24 ms. An answered query stops the pulses for
// about 3.9 s. A 9.2 ms frame fits between two pulses only if it starts right
// after one, so:
//  - if a pulse is seen, the frame starts START_AFTER_PULSE_US after it ends;
//  - if the line stays idle for longer than a pulse period, the bus is in its
//    quiet state and the frame starts at once;
//  - a low longer than PULSE_MAX_US is something else, and nothing is sent.
// Starting only after a low shorter than a bit, or after more than a frame of
// idle, also rules out our start bit landing as data in a frame the opener
// began on a longer low (a 5-bit low, 2 idle bits, then a start bit reads as
// 0x30 to a receiver that does not check the stop bit).
//
// The bus is polled every few microseconds; a glitch shorter than that can be
// missed.
//
// Assumption: the opener samples each bit no earlier than RISE_DEADLINE_US
// into it (a UART samples near the middle).
//
// The algorithm is a template over the bus so the host tests can run it
// against a simulated line and opener. It is forced inline so that on the
// ESP8266 the whole of it lands in the caller, which is placed in IRAM: code
// run from flash stalls for hundreds of microseconds on a cache miss, enough
// to shorten a bit.

#include <cstdint>

#define RATGDO_QUERY_TX_INLINE inline __attribute__((always_inline))

namespace esphome::ratgdo::secplus1 {

enum class QueryTxResult : uint8_t {
    SENT, // The bus carried the byte.
    BUSY, // The bus was not idle; nothing was sent.
    ABORTED, // A mismatch was seen; the rest of the frame was not sent.
};

struct QueryTxInfo {
    // Longest time the bus took to read idle after we released it.
    uint16_t rise_us { 0 };
    // Largest delay between a bit deadline and the edge having been driven.
    uint16_t late_us { 0 };
    // How long the line took to read low after we first pulled it.
    uint16_t fall_us { 0 };
    // The line never read low while we pulled it: nothing reached the bus.
    bool no_drive { false };
    // Frame bit (0 = start, 1-8 = data, 9 = parity) at which it was aborted.
    int8_t aborted_at { -1 };
    // How long the line then stayed low, up to LOW_MEASURE_CAP_US. Also set
    // for BUSY. Zero when the abort was for timing.
    uint16_t low_us { 0 };
    // Width of the periodic pulse the frame was started after; zero when the
    // bus was quiet.
    uint16_t pulse_us { 0 };
};

namespace query_tx {
    // 1200 baud.
    constexpr uint32_t bit_us(uint8_t bit) { return (static_cast<uint32_t>(bit) * 10000U) / 12U; }
    static constexpr uint8_t FRAME_BITS = 11; // start, 8 data, even parity, stop
    // Wait this long for a periodic pulse before treating the bus as quiet.
    // Longer than the pulse period (12.24 ms) and than a frame.
    static constexpr uint32_t PULSE_WAIT_US = 13500;
    // A low longer than this is not the periodic pulse.
    static constexpr uint32_t PULSE_MAX_US = 700;
    // The frame starts this long after a pulse ends, give or take
    // START_SLACK_US. More than half a bit, so a receiver that took the pulse
    // for a start bit has rejected it before ours begins.
    static constexpr uint32_t START_AFTER_PULSE_US = 600;
    static constexpr uint32_t START_SLACK_US = 150;
    // On a quiet bus, a last look inside the critical section.
    static constexpr uint32_t LISTEN_US = bit_us(1) / 2;
    // An edge this late is treated as a corrupted frame.
    static constexpr uint32_t LATE_US = bit_us(1) / 8;
    // While watching a released bit, a gap this long between two looks at the
    // bus (a non-maskable interrupt) means a pulse could have been missed.
    static constexpr uint32_t BLIND_US = bit_us(1) / 8;
    // After the start bit is driven the bus must read low within this time.
    static constexpr uint32_t FALL_DEADLINE_US = 250;
    // After a release the bus must read idle within this time.
    static constexpr uint32_t RISE_DEADLINE_US = 250;
    // After an abort, time the low that caused it for the log, up to this.
    static constexpr uint32_t LOW_MEASURE_CAP_US = 2000;

    constexpr uint16_t frame_bits(uint8_t byte)
    {
        uint8_t parity = byte;
        parity ^= parity >> 4;
        parity ^= parity >> 2;
        parity ^= parity >> 1;
        // LSB first; a set bit means the line is released (idle level).
        return static_cast<uint16_t>((static_cast<uint16_t>(byte) << 1) | (static_cast<uint16_t>(parity & 1) << 9) | (1U << 10));
    }
}

// Bus must provide:
//   uint32_t micros();     monotonic microseconds
//   void drive(bool low);  true pulls the line low, false releases it
//   bool low();            true while the line reads low
//   void lock();           enter the timing-critical section (interrupts off)
//   void unlock();
//
// abort_at_data_bit is a test hook: abandon the frame at that data bit (0-7)
// as if the bus had been found low there.
template <typename Bus>
RATGDO_QUERY_TX_INLINE QueryTxResult send_query_checked(Bus& bus, uint8_t byte, QueryTxInfo* info = nullptr, int8_t abort_at_data_bit = -1)
{
    using namespace query_tx;
    QueryTxInfo local;
    QueryTxInfo& out = info ? *info : local;
    out = QueryTxInfo { };

    const auto time_low = [&]() __attribute__((always_inline)) {
        const uint32_t from = bus.micros();
        uint32_t held = 0;
        while (bus.low() && held < LOW_MEASURE_CAP_US) {
            held = bus.micros() - from;
        }
        out.low_us = static_cast<uint16_t>(held);
    };

    // Find the slot. Interrupts stay on until the pulse has ended.
    uint32_t waited_from = bus.micros();
    uint32_t pulse_rise = 0;
    bool synced = false;
    if (bus.low()) {
        // A low already in progress: its start was not seen, so let it end
        // and start the wait again.
        const uint32_t from = waited_from;
        while (bus.low()) {
            if (bus.micros() - from > PULSE_MAX_US) {
                time_low();
                return QueryTxResult::BUSY;
            }
        }
        waited_from = bus.micros();
    }
    while (bus.micros() - waited_from < PULSE_WAIT_US) {
        if (bus.low()) {
            const uint32_t fell = bus.micros();
            uint32_t width = 0;
            while (bus.low()) {
                width = bus.micros() - fell;
                if (width > PULSE_MAX_US) {
                    time_low();
                    return QueryTxResult::BUSY;
                }
            }
            pulse_rise = bus.micros();
            out.pulse_us = static_cast<uint16_t>(pulse_rise - fell);
            synced = true;
            break;
        }
    }

    struct Critical {
        Bus& bus;
        RATGDO_QUERY_TX_INLINE explicit Critical(Bus& b)
            : bus(b)
        {
            bus.lock();
        }
        RATGDO_QUERY_TX_INLINE ~Critical() { bus.unlock(); }
    } critical(bus);

    if (synced) {
        // Hold off to a fixed point after the pulse. If something delayed us
        // past it, the slot is gone.
        uint32_t since = bus.micros() - pulse_rise;
        if (since > START_AFTER_PULSE_US + START_SLACK_US) {
            return QueryTxResult::BUSY;
        }
        while (since < START_AFTER_PULSE_US) {
            if (bus.low()) {
                time_low();
                return QueryTxResult::BUSY;
            }
            since = bus.micros() - pulse_rise;
        }
        if (since > START_AFTER_PULSE_US + START_SLACK_US) {
            return QueryTxResult::BUSY;
        }
    } else {
        const uint32_t listen_start = bus.micros();
        do {
            if (bus.low()) {
                time_low();
                return QueryTxResult::BUSY;
            }
        } while (bus.micros() - listen_start < LISTEN_US);
    }

    const uint16_t frame = frame_bits(byte);
    const uint32_t t0 = bus.micros();
    const auto abort = [&](uint8_t bit) __attribute__((always_inline)) {
        bus.drive(false);
        time_low();
        out.aborted_at = static_cast<int8_t>(bit);
        return QueryTxResult::ABORTED;
    };

    bool watching = false;
    for (uint8_t bit = 0; bit < FRAME_BITS; ++bit) {
        const uint32_t start = bit_us(bit);
        const uint32_t end = bit_us(bit + 1);
        const bool released = (frame >> bit) & 1;
        const bool data = bit >= 1 && bit <= 8;

        if (data && abort_at_data_bit == bit - 1) {
            return abort(bit);
        }
        bus.drive(!released);
        // Checked after the edge, so a stall just before it is caught too.
        const uint32_t late = bus.micros() - t0 - start;
        if (late > out.late_us) {
            out.late_us = static_cast<uint16_t>(late > 0xFFFF ? 0xFFFF : late);
        }
        if (late > LATE_US) {
            return abort(bit);
        }
        if (bit == 0) {
            // The start bit must actually reach the bus.
            uint32_t fall = 0;
            while (!bus.low()) {
                fall = bus.micros() - t0;
                if (fall > FALL_DEADLINE_US) {
                    out.no_drive = true;
                    return abort(bit);
                }
            }
            out.fall_us = static_cast<uint16_t>(fall);
        }

        if (released && data) {
            // Time of the last look at the bus that found it idle. Carried
            // from the rise check into the watch, so a stall between the two
            // counts as a gap.
            uint32_t seen = start;
            if (!watching) {
                // Just released after a driven bit: let the line rise. The
                // time is taken when it is first seen idle, so a stall that
                // hides a low still counts against the deadline.
                while (bus.low()) {
                    if (bus.micros() - t0 - start > RISE_DEADLINE_US) {
                        return abort(bit);
                    }
                }
                seen = bus.micros() - t0;
                const uint32_t rise = seen - start;
                if (rise > RISE_DEADLINE_US) {
                    return abort(bit);
                }
                if (rise > out.rise_us) {
                    out.rise_us = static_cast<uint16_t>(rise);
                }
                watching = true;
            }
            while (seen < end) {
                if (bus.low()) {
                    return abort(bit);
                }
                const uint32_t now = bus.micros() - t0;
                if (now - seen > BLIND_US) {
                    return abort(bit);
                }
                seen = now;
            }
        } else {
            // Driven bits cannot be corrupted on a wired-AND bus. A flipped
            // parity or stop bit cannot make the byte a different command.
            watching = false;
            while (bus.micros() - t0 < end) {
            }
        }
    }
    return QueryTxResult::SENT;
}

} // namespace esphome::ratgdo::secplus1
