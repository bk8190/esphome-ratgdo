#include "esphome/core/defines.h"

#ifdef USE_ESP8266

#include "ratgdo_uart_esp8266.h"

namespace esphome::ratgdo {

namespace {
    // Direct register access, as SoftwareSerial does for its own bits:
    // digitalWrite() calls waveform helpers that live in flash.
    struct QueryBus {
        uint32_t tx_mask;
        uint32_t rx_mask;
        bool invert;
        uint32_t saved_ps;
        RATGDO_QUERY_TX_INLINE void lock() { saved_ps = xt_rsil(15); }
        RATGDO_QUERY_TX_INLINE void unlock() { xt_wsr_ps(saved_ps); }
        RATGDO_QUERY_TX_INLINE uint32_t micros() { return ::micros(); }
        // Inverted: a high pin pulls the bus low.
        RATGDO_QUERY_TX_INLINE void drive(bool low)
        {
            if (low == invert) {
                GPOS = tx_mask;
            } else {
                GPOC = tx_mask;
            }
        }
        RATGDO_QUERY_TX_INLINE bool low() { return ((GPI & rx_mask) != 0) == invert; }
    };
}

// In IRAM, with the whole sender inlined into it: a flash cache miss in the
// middle of a frame would stall it for longer than the timing allows.
// micros() is in IRAM in the Arduino core.
IRAM_ATTR secplus1::QueryTxResult RatgdoUART::send_query_checked(uint8_t byte, secplus1::QueryTxInfo* info, int8_t abort_at_data_bit)
{
    // GPIO16 is not in the GPO/GPI registers; no ratgdo board uses it here.
    if (this->tx_pin_ < 0 || this->tx_pin_ > 15 || this->rx_pin_ < 0 || this->rx_pin_ > 15) {
        return secplus1::QueryTxResult::BUSY;
    }
    QueryBus bus { 1U << this->tx_pin_, 1U << this->rx_pin_, this->invert_, 0 };
    return secplus1::send_query_checked(bus, byte, info, abort_at_data_bit);
}

} // namespace esphome::ratgdo

#endif
