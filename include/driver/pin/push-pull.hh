#ifndef STM32G4_DRIVER_PIN_PUSH_PULL_HH_
#define STM32G4_DRIVER_PIN_PUSH_PULL_HH_

#include <utl/interface/hal/pin/push-pull.hh>
#include "driver/pin/output.hh"

namespace stm32g4::driver::pin {

class push_pull : public output, public virtual utl::hal::pin::interface::push_pull {
    using direction = utl::hal::pin::direction;
    using active_level = utl::hal::pin::active_level;
public:
    push_pull(GPIO_TypeDef* port, uint16_t pin, active_level level, bool initial_state, 
        uint32_t pull, uint32_t speed)
        : output{port, pin, level, GPIO_MODE_OUTPUT_PP, pull, speed}
    {
        set_state(initial_state);
        set_direction(direction::output);
        
    }

    void set_direction(direction dir) final {
        base::_set_direction(dir);
    }

    void set_state(bool active) final {
        _set_state(active);
    }

};

} //stm32g4::driver::pin


#endif //STM32G4_DRIVER_PIN_PUSH_PULL_HH_