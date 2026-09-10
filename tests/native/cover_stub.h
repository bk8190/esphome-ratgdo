#pragma once

#include "ratgdo.h"

namespace esphome::ratgdo {
inline constexpr float COVER_OPEN = 1.0f;
inline constexpr float COVER_CLOSED = 0.0f;
enum CoverOperation { COVER_OPERATION_IDLE,
    COVER_OPERATION_OPENING,
    COVER_OPERATION_CLOSING };
namespace cover {
    struct CoverCall {
        optional<float> target;
        bool stop = false;
        bool toggle = false;
        bool get_stop() const { return stop; }
        bool get_toggle() const { return toggle; }
        optional<float> get_position() const { return target; }
    };
}
using cover::CoverCall;
// Production control/mapping methods, with the ESPHome transport replaced.
struct RATGDOCover {
    RATGDOComponent* parent_;
    float position = COVER_CLOSED;
    CoverOperation current_operation = COVER_OPERATION_IDLE;
    bool saved = false;
    void control(const cover::CoverCall& call);
    void on_door_state(DoorState state, float position);
    void publish_state(bool save) { saved = save; }
};
}
