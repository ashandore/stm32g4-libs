
#ifndef STM32G4_DRIVER_PIN_OUTPUT_HH_
#define STM32G4_DRIVER_PIN_OUTPUT_HH_

#include <interface/driver/pin/output.hh>
#include "driver/pin/base.hh"

namespace stm32g4::driver::pin {

class output : public base, public virtual utl::driver::pin::interface::output {    
    using direction = utl::driver::pin::direction;
    using active_level = utl::driver::pin::active_level;
public:
    output(GPIO_TypeDef* port, uint16_t pin, active_level level,
        uint32_t mode, uint32_t pull, uint32_t speed)
        : base{port,pin,level,mode,pull,speed}
    {}
};

} //namespace stm32g4::driver::pin
#endif //STM32G4_DRIVER_PIN_OUTPUT_HH_
