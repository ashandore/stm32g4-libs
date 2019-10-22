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
protected:
    source(TIM_TypeDef *timer, uint32_t period) 
        : m_handle{}
    {
        m_handle.Instance = timer;
        m_handle.Init.Prescaler = 0;
        m_handle.Init.CounterMode = TIM_COUNTERMODE_UP;
        m_handle.Init.Period = period;
        m_handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
        m_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;        
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

    // void set_duty(channel_id_t channel);

    // void set_period();
};

using channel_t = source::channel_t;

} //namespace stm32g4::driver::pwm



#endif //STM32G4_LIBS_DRIVER_PWM_HH_
