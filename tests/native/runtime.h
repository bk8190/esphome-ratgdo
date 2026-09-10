#pragma once

#include <algorithm>
#include <cstdint>
#include <deque>
#include <functional>
#include <map>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

#define PROGMEM
#define RATGDO_UART_8E1 0
#define RATGDO_MAX_DOOR_STATE_SUBSCRIBERS 1
#define RATGDO_MAX_DOOR_ACTION_DELAYED_SUBSCRIBERS 1
#define ESP_LOGV(...)
#define ESP_LOGD(...)
#define ESP_LOGI(...)
#define ESP_LOGW(...)
#define ESP_LOGE(...)
#define ESP_LOGCONFIG(...)
#define LOG_STR_ARG(x) x
#define LOG_STR(x) x

namespace esphome {
using std::clamp;
inline uint32_t test_millis = 1000;
inline uint32_t millis() { return test_millis; }
template <typename T>
using optional = std::optional<T>;
constexpr size_t format_hex_pretty_size(size_t n) { return n * 3 + 1; }
inline const char* format_hex_pretty_to(char* out, const uint8_t*, size_t) { return out; }
class InternalGPIOPin {
public:
    int get_pin() const { return 0; }
};

// Single-threaded scheduler with replacement/cancellation by owner and ID.
// Remove a timer before invoking it, so callbacks can safely rearm that ID.
class Scheduler {
public:
    struct Timer {
        uint32_t at;
        std::function<void()> callback;
    };
    using Key = std::pair<void*, uint32_t>;
    std::map<Key, Timer> timers;
    struct Interval {
        uint32_t at;
        uint32_t period;
        std::function<void()> callback;
    };
    // ESPHome has separate namespaces for interval and timeout IDs.
    std::map<Key, Interval> intervals;
    template <typename F>
    void set_interval(void* owner, uint32_t id, uint32_t period, F callback)
    {
        intervals[{ owner, id }] = { millis() + period, period, callback };
    }
    template <typename F>
    void set_timeout(void* owner, uint32_t id, uint32_t delay, F callback)
    {
        timers[{ owner, id }] = { millis() + delay, callback };
    }
    void cancel_timeout(void* owner, uint32_t id) { timers.erase({ owner, id }); }
    void run()
    {
        while (true) {
            auto it = std::find_if(timers.begin(), timers.end(), [](const auto& item) {
                return static_cast<int32_t>(millis() - item.second.at) >= 0;
            });
            if (it == timers.end())
                break;
            auto callback = it->second.callback;
            timers.erase(it);
            callback();
        }
        while (true) {
            auto it = std::find_if(intervals.begin(), intervals.end(), [](const auto& item) {
                return static_cast<int32_t>(millis() - item.second.at) >= 0;
            });
            if (it == intervals.end())
                return;
            auto callback = it->second.callback;
            it->second.at += it->second.period;
            callback();
        }
    }
};

namespace ratgdo {
    class RatgdoUART {
    public:
        std::deque<uint8_t> rx;
        std::vector<std::pair<uint32_t, uint8_t>> tx;
        std::function<optional<uint8_t>(uint8_t)> respond;
        void begin(int, int, int, int, bool) { }
        void on_shutdown() { }
        void enableIntTx(bool) { }
        int baudRate() const { return 1200; }
        bool available() const { return !rx.empty(); }
        uint8_t read()
        {
            auto byte = rx.front();
            rx.pop_front();
            return byte;
        }
        void write(uint8_t byte)
        {
            tx.emplace_back(millis(), byte);
            if (respond) {
                auto reply = respond(byte);
                if (reply) {
                    rx.push_back(byte);
                    rx.push_back(*reply);
                }
            }
        }
    };
}
}
