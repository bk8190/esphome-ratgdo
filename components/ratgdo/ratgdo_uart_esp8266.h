#pragma once

#include "esphome/core/defines.h"

#ifdef USE_ESP8266

#include "SoftwareSerial.h"

#include "esphome/core/application.h"
#include "esphome/core/hal.h"

#include "secplus1_query_tx.h"

namespace esphome::ratgdo {

// Placed in IRAM because plerup's rxBitISR may fire during flash writes.
inline void IRAM_ATTR ratgdo_wake_main_loop_from_isr() { App.wake_loop_threadsafe(); }

// Security+ 2.0 preamble timing (microseconds)
static constexpr uint16_t PREAMBLE_DURATION_US = 1300;
static constexpr uint16_t PREAMBLE_MARK_US = 130;

enum RatgdoUARTConfig {
    RATGDO_UART_8N1 = SWSERIAL_8N1,
    RATGDO_UART_8E1 = SWSERIAL_8E1,
};

// On ESP8266, simply fall back to SoftwareSerial
class RatgdoUART : public SoftwareSerial {
public:
    void begin(uint32_t baud, RatgdoUARTConfig config, int8_t rxPin, int8_t txPin,
        bool invert)
    {
        this->rx_pin_ = rxPin;
        this->tx_pin_ = txPin;
        this->invert_ = invert;
        SoftwareSerial::begin(baud, static_cast<Config>(config), rxPin, txPin, invert);
        // Wake the main loop on incoming bytes instead of polling each tick.
        this->onReceive(ratgdo_wake_main_loop_from_isr);
    }

    void on_shutdown()
    {
        if (this->tx_pin_ >= 0) {
            ::pinMode(this->tx_pin_, INPUT);
        }
        if (this->rx_pin_ >= 0) {
            ::pinMode(this->rx_pin_, INPUT);
        }
    }

    // Send one Security+ 1.0 query bit by bit with interrupts off, reading the
    // bus back while it is released. See secplus1_query_tx.h. Nothing of the
    // frame is received through SoftwareSerial; the caller stands in for the
    // echo when the result is SENT. Blocks for about two frame times.
    // Diagnostic: poll the RX pin for duration_us and record each change of
    // bus level as (microseconds since start) << 1 | (1 if the bus went low).
    // Returns the number of entries written. Interrupts stay on.
    size_t scope_capture(uint32_t duration_us, uint32_t* edges, size_t max_edges);

    // Defined in ratgdo_uart_esp8266.cpp so it can be placed in IRAM.
    secplus1::QueryTxResult send_query_checked(uint8_t byte, secplus1::QueryTxInfo* info = nullptr, int8_t abort_at_data_bit = -1);

    // Fallback for SECPLUS2 preamble on ESP8266
    void transmit_secplus2_preamble()
    {
        if (this->tx_pin_ < 0)
            return;
        ::digitalWrite(this->tx_pin_, HIGH);
        ::delayMicroseconds(PREAMBLE_DURATION_US);
        ::digitalWrite(this->tx_pin_, LOW);
        ::delayMicroseconds(PREAMBLE_MARK_US);
    }

private:
    int8_t rx_pin_ { -1 };
    int8_t tx_pin_ { -1 };
    bool invert_ { false };
};

} // namespace esphome::ratgdo

#endif
