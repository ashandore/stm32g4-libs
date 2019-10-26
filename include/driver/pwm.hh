#ifndef STM32G4_LIBS_DRIVER_PWM_HH_
#define STM32G4_LIBS_DRIVER_PWM_HH_

#include "hal.hh"
#include "stm32g4xx_hal_tim.h"
#include "stm32g4xx_hal_tim_ex.h"
#include <result.hh>
#include <interface/driver/driver.hh>
#include <interface/driver/pwm.hh>

extern "C" void HAL_TIM_MspPostInit(TIM_HandleTypeDef* timHandle);

namespace stm32g4::driver::pwm {



enum class channel_id {
    CHANNEL_1 = TIM_CHANNEL_1,
    CHANNEL_2 = TIM_CHANNEL_2,
    CHANNEL_3 = TIM_CHANNEL_3,
    CHANNEL_4 = TIM_CHANNEL_4,
    CHANNEL_5 = TIM_CHANNEL_5,
    CHANNEL_6 = TIM_CHANNEL_6
};

class source : public utl::driver::interface::driver {
    TIM_HandleTypeDef       m_handle;
    uint32_t                m_period_us;
    float                   m_ticks_us;

    static uint32_t get_source_clock_hz(TIM_TypeDef *module) {
        RCC_ClkInitTypeDef clkConfig;
        uint32_t flashLatency;

        HAL_RCC_GetClockConfig(&clkConfig, &flashLatency);

        if(module == TIM2 || module == TIM3 || module == TIM4 || module == TIM5) {
            if(clkConfig.APB1CLKDivider == 1) return HAL_RCC_GetPCLK1Freq();
            //See datasheet. If the divider isn't 1, the clocks going to the timer
            //modules are multiplied by 2.
            return 2*HAL_RCC_GetPCLK1Freq();
        }

        if(module == TIM1 || module == TIM9 || module == TIM10 || module == TIM11) {
            if(clkConfig.APB2CLKDivider == 1) return HAL_RCC_GetPCLK2Freq();
            return 2*HAL_RCC_GetPCLK2Freq();
        }

        return 0;         
    }

    uint32_t us_to_count(uint32_t period_us) const {
        return static_cast<uint32_t>(period_us*m_ticks_us);
    }

    uint32_t count_to_us(uint32_t count) const {
        return static_cast<uint32_t>(count/m_ticks_us);
    }

    uint32_t calc_prescaler(uint32_t period_us) {
        //Calculate the minimum prescaler value that is necessary
        //to configure the provided period value.
        //We want the minimum value because that will provide us with
        //the highest time resolution possible.
        const float source_ticks_us = get_source_clock_hz(m_handle.Instance)/1000000.0f;
        const uint32_t period_ticks_us = static_cast<uint32_t>(period_us*source_ticks_us);
        //the extra addition/subtraction gives the ceiling of the result
        const uint32_t min_psc = 1 + (period_ticks_us - 1)/65535u; 
        //Need to account for the +1 that gets added to the register value
        if(min_psc == 0) return 0;
        if(min_psc > 65535) return 65535;
        return min_psc - 1;
    }

protected:
    source(TIM_TypeDef *timer, uint32_t period_us) 
        : m_handle{}, m_period_us{period_us}, m_ticks_us{}
    {
        m_handle.Instance = timer;
        m_handle.Init.Prescaler = calc_prescaler(m_period_us);
        m_handle.Init.CounterMode = TIM_COUNTERMODE_UP;
        m_handle.Init.Period = us_to_count(m_period_us);
        m_handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
        m_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;        

        m_ticks_us = (get_source_clock_hz(m_handle.Instance))/(m_handle.Init.Prescaler + 1);

        __HAL_TIM_ENABLE_IT(&m_handle, TIM_IT_UPDATE);
    }

    utl::result<void> validate() {
        TIM_ClockConfigTypeDef sClockSourceConfig = {0};
        TIM_MasterConfigTypeDef sMasterConfig = {0};
        TIM_OC_InitTypeDef sConfigOC = {0};        
        
        auto res = HAL_TIM_Base_Init(&m_handle);
        if(res != HAL_OK) return stm32g4::make_hal_error_code(res);

        sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
        
        res = HAL_TIM_ConfigClockSource(&m_handle, &sClockSourceConfig);
        if(res != HAL_OK) return stm32g4::make_hal_error_code(res);
        
        res = HAL_TIM_PWM_Init(&m_handle);
        if(res != HAL_OK) return stm32g4::make_hal_error_code(res);

        sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
        sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
        
        res = HAL_TIMEx_MasterConfigSynchronization(&m_handle, &sMasterConfig);
        if(res != HAL_OK) return stm32g4::make_hal_error_code(res);

        sConfigOC.OCMode = TIM_OCMODE_PWM1;
        sConfigOC.Pulse = 2500;
        sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
        sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
        
        res = HAL_TIM_PWM_ConfigChannel(&m_handle, &sConfigOC, TIM_CHANNEL_1);
        if(res != HAL_OK) return stm32g4::make_hal_error_code(res);

        HAL_TIM_MspPostInit(&m_handle);        

        return utl::success();
    }

public:
    using channel_id_t = channel_id;
    using channel_t = utl::driver::pwm::interface::channel<source>;

    const channel_t make_channel(channel_id_t channel) {
        return channel_t{*this,channel};
    }

    utl::result<void> start(channel_id_t channel) {
        auto res = HAL_TIM_PWM_Start(&m_handle, static_cast<uint32_t>(channel));
        if(res != HAL_OK) return stm32g4::make_hal_error_code(res);
        return utl::success();
    }

    utl::result<void> stop(channel_id_t channel) {
        auto res = HAL_TIM_PWM_Stop(&m_handle, static_cast<uint32_t>(channel));
        if(res != HAL_OK) return stm32g4::make_hal_error_code(res);
        return utl::success();
    }

    void set_period_us(uint32_t period_us) {
        if(period_us < 1) period_us = 1;
        //period is in microseconds. Convert to raw counts.
        m_period_us = period_us;

        //Update the prescaler if necessary
        const uint32_t prescaler = calc_prescaler(m_period_us);
        __HAL_TIM_SET_PRESCALER(&m_handle, prescaler);
        m_handle.Init.Prescaler = prescaler;

        //Update the period value
        __HAL_TIM_SET_AUTORELOAD(&m_handle, us_to_count(m_period_us));
    }

    uint32_t period_us() const {
        return m_period_us;
    }

    void set_width_us(channel_id_t channel, uint32_t width_us) {
        const uint32_t width_count = us_to_count(width_us);
        __HAL_TIM_SET_COMPARE(&m_handle, static_cast<uint32_t>(channel), width_count);
    }

    uint32_t width_us(channel_id_t channel) const {
        uint32_t value = __HAL_TIM_GET_COMPARE(&m_handle, static_cast<uint32_t>(channel));        
        return count_to_us(value);
    }
};

using channel_t = source::channel_t;

} //namespace stm32g4::driver::pwm



#endif //STM32G4_LIBS_DRIVER_PWM_HH_
