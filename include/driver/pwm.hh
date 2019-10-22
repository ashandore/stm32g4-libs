#ifndef STM32G4_LIBS_DRIVER_PWM_HH_
#define STM32G4_LIBS_DRIVER_PWM_HH_

#include "hal.hh"
#include "stm32g4xx_hal_tim.h"
#include "stm32g4xx_hal_tim_ex.h"
#include <result.hh>
#include <interface/driver/driver.hh>

extern "C" void HAL_TIM_MspPostInit(TIM_HandleTypeDef* timHandle);

namespace stm32g4::driver {

class pwm : public utl::driver::interface::driver {
    TIM_HandleTypeDef       m_handle;
protected:
    pwm(TIM_TypeDef *timer, uint32_t period) 
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
        
        res = HAL_TIM_PWM_ConfigChannel(&m_handle, &sConfigOC, TIM_CHANNEL_2);
        if(res != HAL_OK) return stm32g4::make_hal_error_code(res);

        HAL_TIM_MspPostInit(&m_handle);

        return utl::success();
    }
public:

};

} //namespace stm32g4::driver



#endif //STM32G4_LIBS_DRIVER_PWM_HH_
