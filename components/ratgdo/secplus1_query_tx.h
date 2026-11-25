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
// high, or an edge went out late, it spoils the frame: it holds the line low
// through the stop bit. A receiver that checks the stop bit drops the frame.
// One that does not reads the data bits sent so far followed by zeros, which
// is never 0x30-0x37.
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
    SPOILED, // A mismatch was seen; the frame was destroyed.
};

struct QueryTxInfo {
    // Longest time the bus took to read idle after we released it.
    uint16_t rise_us { 0 };
    // Largest delay between a bit deadline and reaching it.
    uint16_t late_us { 0 };
    // Frame bit (0 = start, 1-8 = data, 9 = parity) at which it was spoiled.
    int8_t spoiled_at { -1 };
};

namespace query_tx {
    // 1200 baud.
    constexpr uint32_t bit_us(uint8_t bit) { return (static_cast<uint32_t>(bit) * 10000U) / 12U; }
    static constexpr uint8_t FRAME_BITS = 11; // start, 8 data, even parity, stop
    static constexpr uint32_t LISTEN_US = bit_us(1);
    // An edge this late is treated as a corrupted frame.
    static constexpr uint32_t LATE_US = bit_us(1) / 8;
    // While watching a released bit, a gap this long between two looks at the
    // bus (a non-maskable interrupt) means a pulse could have been missed.
    static constexpr uint32_t BLIND_US = bit_us(1) / 8;
    // After a release the bus must read idle within this time.
    static constexpr uint32_t RISE_DEADLINE_US = 250;
    // Hold a spoiled frame low until half a bit past the end of the stop bit.
    static constexpr uint32_t SPOIL_END_US = bit_us(FRAME_BITS) + bit_us(1) / 2;

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
//
// spoil_from_data_bit is a test hook: spoil the frame at that data bit (0-7)
// as if the bus had been found low there.
template <typename Bus>
QueryTxResult send_query_checked(Bus& bus, uint8_t byte, QueryTxInfo* info = nullptr, int8_t spoil_from_data_bit = -1)
{
    using namespace query_tx;
    QueryTxInfo local;
    QueryTxInfo& out = info ? *info : local;
    out = QueryTxInfo { };

    const uint32_t listen_start = bus.micros();
    do {
        if (bus.low()) {
            return QueryTxResult::BUSY;
        }
    } while (bus.micros() - listen_start < LISTEN_US);

    const uint16_t frame = frame_bits(byte);
    const uint32_t t0 = bus.micros();
    const auto spoil = [&](uint8_t bit) {
        bus.drive(true);
        while (bus.micros() - t0 < SPOIL_END_US) {
        }
        bus.drive(false);
        out.spoiled_at = static_cast<int8_t>(bit);
        return QueryTxResult::SPOILED;
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
        if (late > LATE_US || (data && spoil_from_data_bit == bit - 1)) {
            return spoil(bit);
        }
        bus.drive(!released);

        if (released && data) {
            if (!watching) {
                // Just released after a driven bit: let the line rise.
                uint32_t rise = 0;
                while (bus.low()) {
                    rise = bus.micros() - t0 - start;
                    if (rise > RISE_DEADLINE_US) {
                        return spoil(bit);
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
                    return spoil(bit);
                }
                const uint32_t now = bus.micros() - t0;
                if (now - seen > BLIND_US) {
                    return spoil(bit);
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
