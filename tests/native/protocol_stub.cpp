#include "observable.h"
#include "protocol.h"

// Firmware link-time optimization removes the unused Protocol base vtable.
// The host build supplies fail-fast definitions instead; all calls must reach
// Secplus1's real overrides, never these base methods.
namespace esphome::ratgdo::protocol {
void Protocol::setup(RATGDOComponent*, Scheduler*, InternalGPIOPin*, InternalGPIOPin*) { throw std::runtime_error("base setup"); }
void Protocol::loop() { throw std::runtime_error("base loop"); }
void Protocol::dump_config() { throw std::runtime_error("base dump_config"); }
void Protocol::sync() { throw std::runtime_error("base sync"); }
void Protocol::set_open_limit(bool) { throw std::runtime_error("base set_open_limit"); }
void Protocol::set_close_limit(bool) { throw std::runtime_error("base set_close_limit"); }
void Protocol::set_discrete_open_pin(InternalGPIOPin*) { throw std::runtime_error("base set_discrete_open_pin"); }
void Protocol::set_discrete_close_pin(InternalGPIOPin*) { throw std::runtime_error("base set_discrete_close_pin"); }
const Traits& Protocol::traits() const { throw std::runtime_error("base traits"); }
void Protocol::light_action(LightAction) { throw std::runtime_error("base light_action"); }
void Protocol::lock_action(LockAction) { throw std::runtime_error("base lock_action"); }
void Protocol::door_action(DoorAction) { throw std::runtime_error("base door_action"); }
Result Protocol::call(Args) { throw std::runtime_error("base call"); }
}
