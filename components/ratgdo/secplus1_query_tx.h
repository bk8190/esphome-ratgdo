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
// Before sending it requires the line to have been idle for a whole frame
// time. Otherwise a wide low pulse that ended just before our start bit could
// have started a frame in the opener, and our start bit would land in that
// frame as data: a 5-bit low, 2 idle bits, then our start bit reads as 0x30
// to a receiver that does not check the stop bit.
//
// Assumption: the opener samples each bit no earlier than RISE_DEADLINE_US
// into it (a UART samples near the middle).
//
// The algorithm is a template over the bus so the host tests can run it
// against a simulated line and opener.

#include <cstdint>

namespace esphome::ratgdo::secplus1 {

enum class QueryTxResult : uint8_t {
    SENT, // The bus carried the byte.
    BUSY, // The bus was not idle; nothing was sent.
    ABORTED, // A mismatch was seen; the rest of the frame was not sent.
};

struct QueryTxInfo {
    // Longest time the bus took to read idle after we released it.
    uint16_t rise_us { 0 };
    // Largest delay between a bit deadline and reaching it.
    uint16_t late_us { 0 };
    // Frame bit (0 = start, 1-8 = data, 9 = parity) at which it was aborted.
    int8_t aborted_at { -1 };
    // How long the line then stayed low, up to LOW_MEASURE_CAP_US. Also set
    // for BUSY. Zero when the abort was for timing.
    uint16_t low_us { 0 };
};

namespace query_tx {
    // 1200 baud.
    constexpr uint32_t bit_us(uint8_t bit) { return (static_cast<uint32_t>(bit) * 10000U) / 12U; }
    static constexpr uint8_t FRAME_BITS = 11; // start, 8 data, even parity, stop
    // The line must be idle this long before the start bit: PRE_LISTEN_US
    // with interrupts on, then LISTEN_US inside the critical section.
    static constexpr uint32_t PRE_LISTEN_US = bit_us(10);
    static constexpr uint32_t LISTEN_US = bit_us(1);
    // An edge this late is treated as a corrupted frame.
    static constexpr uint32_t LATE_US = bit_us(1) / 8;
    // While watching a released bit, a gap this long between two looks at the
    // bus (a non-maskable interrupt) means a pulse could have been missed.
    static constexpr uint32_t BLIND_US = bit_us(1) / 8;
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
QueryTxResult send_query_checked(Bus& bus, uint8_t byte, QueryTxInfo* info = nullptr, int8_t abort_at_data_bit = -1)
{
    using namespace query_tx;
    QueryTxInfo local;
    QueryTxInfo& out = info ? *info : local;
    out = QueryTxInfo { };

    const auto time_low = [&] {
        const uint32_t from = bus.micros();
        uint32_t held = 0;
        while (bus.low() && held < LOW_MEASURE_CAP_US) {
            held = bus.micros() - from;
        }
        out.low_us = static_cast<uint16_t>(held);
    };

    // Only a low of about half a bit or more can start a frame in the
    // opener, so interrupts may stay on for most of the wait.
    const uint32_t pre_listen_start = bus.micros();
    do {
        if (bus.low()) {
            time_low();
            return QueryTxResult::BUSY;
        }
    } while (bus.micros() - pre_listen_start < PRE_LISTEN_US);

    struct Critical {
        Bus& bus;
        explicit Critical(Bus& b)
            : bus(b)
        {
            bus.lock();
        }
        ~Critical() { bus.unlock(); }
    } critical(bus);

    const uint32_t listen_start = bus.micros();
    do {
        if (bus.low()) {
            time_low();
            return QueryTxResult::BUSY;
        }
    } while (bus.micros() - listen_start < LISTEN_US);

    const uint16_t frame = frame_bits(byte);
    const uint32_t t0 = bus.micros();
    const auto abort = [&](uint8_t bit) {
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

        const uint32_t late = bus.micros() - t0 - start;
        if (late > out.late_us) {
            out.late_us = static_cast<uint16_t>(late > 0xFFFF ? 0xFFFF : late);
        }
        if (late > LATE_US || (data && abort_at_data_bit == bit - 1)) {
            return abort(bit);
        }
        bus.drive(!released);

        if (released && data) {
            if (!watching) {
                // Just released after a driven bit: let the line rise.
                uint32_t rise = 0;
                while (bus.low()) {
                    rise = bus.micros() - t0 - start;
                    if (rise > RISE_DEADLINE_US) {
                        return abort(bit);
                    }
                }
                if (rise > out.rise_us) {
                    out.rise_us = static_cast<uint16_t>(rise);
                }
                watching = true;
            }
            uint32_t seen = bus.micros() - t0;
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
