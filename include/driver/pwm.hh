#ifndef STM32G4_LIBS_DRIVER_PWM_HH_
#define STM32G4_LIBS_DRIVER_PWM_HH_

#include "hal.hh"
#include "stm32g4xx_hal_tim.h"
#include "stm32g4xx_hal_tim_ex.h"
#include <result.hh>
#include <interface/driver/driver.hh>
#include <interface/driver/pwm.hh>
#include <driver/dma.hh>
#include <utl.hh>

extern "C" void HAL_TIM_MspPostInit(TIM_HandleTypeDef* timHandle);

namespace stm32g4::driver::pwm {

using namespace utl::literals;
using polarity = utl::driver::pwm::interface::polarity;

enum class channel_id {
    CHANNEL_1 = TIM_CHANNEL_1,
    CHANNEL_2 = TIM_CHANNEL_2,
    CHANNEL_3 = TIM_CHANNEL_3,
    CHANNEL_4 = TIM_CHANNEL_4,
    CHANNEL_5 = TIM_CHANNEL_5,
    CHANNEL_6 = TIM_CHANNEL_6
};

//FIXME: this should take a time unit as a policy.
//TODO: make unit types & literals (handle conversions, etc)

template <typename Precision>
class source : public utl::driver::interface::driver {  
public:  
    using polarity_t = utl::driver::pwm::interface::polarity;
    using channel_id_t = channel_id;
    using channel_t = utl::driver::pwm::interface::channel<source>;
    using dma_channel_t = utl::driver::pwm::interface::dma_channel<source, dma::channel>;
    using time_t = Precision;
private:
    TIM_HandleTypeDef       m_handle;
    time_t                  m_period;
    float                   m_ticks_per_ns;

    static uint32_t get_source_clock_hz(TIM_TypeDef *module) {
        RCC_ClkInitTypeDef clkConfig;
        uint32_t flashLatency;

        HAL_RCC_GetClockConfig(&clkConfig, &flashLatency);

        //fixme: this is not generic enough; not all parts have all timers.
        if(module == TIM2 || module == TIM3 || module == TIM4/*  || module == TIM5 */) {
            if(clkConfig.APB1CLKDivider == RCC_HCLK_DIV1) return HAL_RCC_GetPCLK1Freq();
            //See datasheet. If the divider isn't 1, the clocks going to the timer
            //modules are multiplied by 2.
            return 2*HAL_RCC_GetPCLK1Freq();
        }

        if(module == TIM1 /* || module == TIM9 || module == TIM10 || module == TIM11 */) {
            if(clkConfig.APB2CLKDivider == RCC_HCLK_DIV1) return HAL_RCC_GetPCLK2Freq();
            return 2*HAL_RCC_GetPCLK2Freq();
        }

        return 0;         
    }

    uint32_t time_to_count(time_t period) const {
        return static_cast<uint32_t>(period.value*m_ticks_per_ns);
    }

    time_t count_to_time(uint32_t count) const {
        return time_t{static_cast<uint32_t>(count/m_ticks_per_ns)};
    }

    uint32_t calc_prescaler(time_t period) {
        //Calculate the minimum prescaler value that is necessary
        //to configure the provided period value.
        //We want the minimum value because that will provide us with
        //the highest time resolution possible.
        const float desired_tick_time_ns = static_cast<float>(period.value)/65535.0;
        const float source_ticks_ns = get_source_clock_hz(m_handle.Instance)/1000000000.0f;
        //the extra addition/subtraction gives the ceiling of the result
        const uint32_t min_psc = static_cast<uint32_t>(source_ticks_ns*desired_tick_time_ns + 1) - 1;
        //Need to account for the +1 that gets added to the register value
        if(min_psc == 0) return 0;
        if(min_psc > 65535) return 65535;
        return min_psc;
    }

protected:
    source(TIM_TypeDef *timer, time_t period) 
        : m_handle{}, m_period{period}, m_ticks_per_ns{}
    {
        m_handle.Instance = timer;
        m_handle.Init.Prescaler = calc_prescaler(m_period);
        m_handle.Init.CounterMode = TIM_COUNTERMODE_UP;
        m_handle.Init.Period = time_to_count(m_period);
        m_handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
        m_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;        

        m_ticks_per_ns = (get_source_clock_hz(m_handle.Instance))/(m_handle.Init.Prescaler + 1)/1000000000.0f;
    }

    utl::result<void> validate() {
        TIM_ClockConfigTypeDef sClockSourceConfig = {0};
        TIM_MasterConfigTypeDef sMasterConfig = {0};
        TIM_OC_InitTypeDef sConfigOC = {0};        
        
        auto res = HAL_TIM_Base_Init(&m_handle);
        if(res != HAL_OK) return make_hal_error_code(res);

        sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
        
        res = HAL_TIM_ConfigClockSource(&m_handle, &sClockSourceConfig);
        if(res != HAL_OK) return make_hal_error_code(res);

        sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
        sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
        
        res = HAL_TIMEx_MasterConfigSynchronization(&m_handle, &sMasterConfig);
        if(res != HAL_OK) return make_hal_error_code(res);        

        HAL_TIM_MspPostInit(&m_handle);       

        return utl::success();
    }

    utl::result<void> configure_channel(channel_id_t channel, polarity pol) {
        TIM_OC_InitTypeDef channelConfig;

        channelConfig.OCMode     = TIM_OCMODE_PWM1;        
        channelConfig.OCFastMode = TIM_OCFAST_DISABLE;
        channelConfig.Pulse = __HAL_TIM_GET_COMPARE(&m_handle, static_cast<uint32_t>(channel));
        switch(pol) {
            case polarity::ACTIVE_HIGH:
                channelConfig.OCPolarity = TIM_OCPOLARITY_HIGH;
                break;
            case polarity::ACTIVE_LOW:
                channelConfig.OCPolarity = TIM_OCPOLARITY_LOW;
                break;
        }

        auto res = HAL_TIM_PWM_ConfigChannel(&m_handle, &channelConfig, static_cast<uint32_t>(channel));
        if(res != HAL_OK) return make_hal_error_code(res);
        return utl::success();
    }
public:

    const channel_t make_channel(channel_id_t channel, polarity pol) {
        return channel_t{*this,channel,pol};
    }

    void link_dma(channel_id_t channel, dma::channel& dma) {
        switch(channel) {
            case channel_id_t::CHANNEL_1:
                dma.link(m_handle,TIM_DMA_ID_CC1);
                return;
            case channel_id_t::CHANNEL_2:
                dma.link(m_handle,TIM_DMA_ID_CC2);
                return;
            case channel_id_t::CHANNEL_3:
                dma.link(m_handle,TIM_DMA_ID_CC3);
                return;
            case channel_id_t::CHANNEL_4:
                dma.link(m_handle,TIM_DMA_ID_CC4);
                return;
        }
        
    }

    utl::result<void> start() {
        auto res = HAL_TIM_PWM_Init(&m_handle);
        if(res != HAL_OK) return make_hal_error_code(res);
        __HAL_TIM_ENABLE_IT(&m_handle, TIM_IT_UPDATE); 
        return utl::success();
    }

    utl::result<void> stop() {
        auto res = HAL_TIM_PWM_DeInit(&m_handle);
        if(res != HAL_OK) return make_hal_error_code(res);
        __HAL_TIM_DISABLE_IT(&m_handle, TIM_IT_UPDATE); 
        return utl::success();
    }

    utl::result<void> start(channel_id_t channel, polarity pol) {
        auto res = configure_channel(channel, pol);
        if(!res) return res;

        auto hal_res = HAL_TIM_PWM_Start(&m_handle, static_cast<uint32_t>(channel));
        if(hal_res != HAL_OK) return make_hal_error_code(hal_res);
        return utl::success();
    }

    utl::result<void> start_dma(channel_id_t channel, polarity pol, uint32_t* data, uint32_t length)
    {
        auto res = configure_channel(channel, pol);
        if(!res) return res;

        auto hal_res = HAL_TIM_PWM_Start_DMA(&m_handle, static_cast<uint32_t>(channel), data, length);
        // if(length % 4 != 0) utl::log("WARNING: PWM DMA wants multiples of 4 bytes.");
        if(hal_res != HAL_OK) return make_hal_error_code(hal_res);
        return utl::success();
    }

    utl::result<void> stop(channel_id_t channel) {
        auto res = HAL_TIM_PWM_Stop(&m_handle, static_cast<uint32_t>(channel));
        if(res != HAL_OK) return make_hal_error_code(res);
        return utl::success();
    }

    utl::result<void> stop_dma(channel_id_t channel) {
        auto res = HAL_TIM_PWM_Stop_DMA(&m_handle, static_cast<uint32_t>(channel));
        if(res != HAL_OK) return make_hal_error_code(res);
        return utl::success();
    }

    void set_period(time_t period) {
        if(period < 10_ns) m_period = 10_ns;
        else m_period = period;

        //Update the prescaler if necessary
        const uint32_t prescaler = calc_prescaler(m_period);
        __HAL_TIM_SET_PRESCALER(&m_handle, prescaler);
        m_handle.Init.Prescaler = prescaler;

        //Update the period value
        __HAL_TIM_SET_AUTORELOAD(&m_handle, time_to_count(m_period));
    }

    time_t period() const {
        return m_period;
    }

    void set_width(channel_id_t channel, time_t width) {
        const uint32_t width_count = time_to_count(width);
        __HAL_TIM_SET_COMPARE(&m_handle, static_cast<uint32_t>(channel), width_count);
    }

    time_t width(channel_id_t channel) const {
        uint32_t value = __HAL_TIM_GET_COMPARE(&m_handle, static_cast<uint32_t>(channel));        
        return count_to_time(value);
    }

    constexpr uint32_t width_to_dma_value(time_t width) const {
        return time_to_count(width);
    }

    void clear_interrupt() {
        __HAL_TIM_CLEAR_IT(&m_handle, TIM_IT_UPDATE);
    }
};

} //namespace stm32g4::driver::pwm



#endif //STM32G4_LIBS_DRIVER_PWM_HH_
