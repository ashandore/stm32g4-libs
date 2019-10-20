
#ifndef STM32G4_DRIVER_PIN_BASE_HH_
#define STM32G4_DRIVER_PIN_BASE_HH_

#include <board.h>
#include <stm32g4xx_hal_gpio.h>
#include <interface/driver/pin/base.hh>

namespace stm32g4::driver::pin {

// enum class mode {
//     INPUT,
//     OUTPUT_PP,
//     OUTPUT_OD,
//     AF_PP,
//     AF_OD,
//     ANALOG,
//     IT_RISING,
//     IT_FALLING,
//     IT_RISING_FALLING,
//     EVT_RISING,
//     EVT_FALLING,
//     EVT_RISING_FALLING
// };

// enum class pull {
//     NOPULL,
//     PULLUP,
//     PULLDOWN
// };

// enum class speed {
//     FREQ_LOW,
//     FREQ_MEDIUM,
//     FREQ_HIGH,
//     FREQ_VERY_HIGH
// };

class base : public virtual utl::driver::pin::interface::base {
    using direction = utl::driver::pin::direction;
    using active_level = utl::driver::pin::active_level;

    GPIO_TypeDef* const m_port;
    GPIO_InitTypeDef    m_handle;
    active_level        m_active_level;

protected:
    void _set_direction(direction dir) {
        switch(dir) {
            case direction::input:
                m_handle.Mode = GPIO_MODE_INPUT;
                HAL_GPIO_Init(m_port, &m_handle);
                break;
            case direction::output:
                m_handle.Mode = GPIO_MODE_OUTPUT_PP;
                HAL_GPIO_Init(m_port, &m_handle);
                break;
        }
    }

    void _set_state(bool active) {
        bool active_high = m_active_level == active_level::high;
        const bool write_high = (active && active_high) || (!active && !active_high);
        if(write_high) {
            HAL_GPIO_WritePin(m_port, static_cast<uint16_t>(m_handle.Pin), GPIO_PIN_SET);
        } else {
            HAL_GPIO_WritePin(m_port, static_cast<uint16_t>(m_handle.Pin), GPIO_PIN_RESET);
        }
    }

public:
    base(GPIO_TypeDef* port, uint16_t pin, active_level level, uint32_t mode, uint32_t pull, uint32_t speed) :
        m_port{port}, m_active_level{level}
    {
        //Enable the appropriate clock
        if(m_port == GPIOA) __HAL_RCC_GPIOA_CLK_ENABLE();

        #ifdef GPIOB
        else if(m_port == GPIOB) __HAL_RCC_GPIOB_CLK_ENABLE();
        #endif

        #ifdef GPIOC
        else if(m_port == GPIOC) __HAL_RCC_GPIOC_CLK_ENABLE();
        #endif

        #ifdef GPIOD
        else if(m_port == GPIOD) __HAL_RCC_GPIOD_CLK_ENABLE();
        #endif

        #ifdef GPIOE
        else if(m_port == GPIOE) __HAL_RCC_GPIOE_CLK_ENABLE();
        #endif

        #ifdef GPIOF
        else if(m_port == GPIOF) __HAL_RCC_GPIOF_CLK_ENABLE();
        #endif

        #ifdef GPIOG
        else if(m_port == GPIOG) __HAL_RCC_GPIOG_CLK_ENABLE();
        #endif

        #ifdef GPIOH
        else if(m_port == GPIOH) __HAL_RCC_GPIOH_CLK_ENABLE();
        #endif

        m_handle.Pin = pin;
        m_handle.Mode = mode;
        m_handle.Pull = pull;
        m_handle.Speed = speed;
    }

    bool get_state(void) const final {
        const bool state = (HAL_GPIO_ReadPin(m_port, static_cast<uint16_t>(m_handle.Pin)) == GPIO_PIN_SET);
        return (m_active_level == active_level::high) && state;
    }

    void set_active_level(active_level level) final {
        m_active_level = level;
    }

    active_level get_active_level(void) const final {
        return m_active_level;
    }

    direction get_direction(void) const final {
        switch(m_handle.Mode) {
            case GPIO_MODE_INPUT:
                return direction::input;
            case GPIO_MODE_OUTPUT_PP:
                [[fallthrough]];
            case GPIO_MODE_OUTPUT_OD:
                return direction::output;
            default:
                //FIXME: this really should be an error of some kind.
                return direction::input;
        }
    }

};

} //namespace driver::pin::stm32g4


#endif //STM32G4_DRIVER_PIN_BASE_HH_
