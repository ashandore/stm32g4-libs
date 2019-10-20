#ifndef STM32G4_LIBS_ADC_HH_
#define STM32G4_LIBS_ADC_HH_

#include "board.h" //FIXME: need to express this dependency more clearly.
#include <stdio.h>
#include <stdarg.h>
#include <result.hh>
#include <interface/driver/driver.hh>
#include "stm32g4xx_hal_adc.h"
#include <system-error.hh>


namespace stm32g4::driver {

template <class Adc>
class adc_channel {
    Adc const * const   m_adc;
    uint32_t const      m_channel;
public:
    adc_channel(Adc *const master, uint32_t channel) : m_adc{master}, m_channel{channel} {}
    utl::result<uint16_t> conversion() const { return m_adc->conversion(m_channel); }
    float to_voltage(uint16_t conversion) const { return m_adc->to_voltage(conversion); }
    float supply() const { return m_adc->supply(); }
};

class adc : public utl::driver::interface::driver {
public:
    using adc_channel_t = adc_channel<adc>;
private:
    uint32_t                    m_timeout;
    mutable ADC_HandleTypeDef   m_handle;
    float                       m_supply;
public:
    adc(ADC_TypeDef *module, uint32_t timeout, float supply) :
        m_timeout{timeout}, m_supply{supply}
    {
        m_supply = supply;
        if (timeout > 1000) {
            timeout = 1000;
        }
        m_timeout = timeout;

        m_handle.Instance = module;

        m_handle.Init.ClockPrescaler        = ADC_CLOCKPRESCALER_PCLK_DIV2;
        m_handle.Init.Resolution            = ADC_RESOLUTION_12B;
        m_handle.Init.ScanConvMode          = DISABLE;
        m_handle.Init.ContinuousConvMode    = DISABLE;
        m_handle.Init.DiscontinuousConvMode = DISABLE;
        m_handle.Init.NbrOfDiscConversion   = 0;
        m_handle.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
        m_handle.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
        m_handle.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
        m_handle.Init.NbrOfConversion       = 1;
        m_handle.Init.DMAContinuousRequests = DISABLE;
        m_handle.Init.EOCSelection          = DISABLE;         
    }

    utl::result<void> validate() {
        auto res = HAL_ADC_Init(&m_handle);
        if(res == HAL_OK) return utl::success();
        return res;
    }
public:
    void start() {}    
    void stop() {}

    utl::result<uint16_t> conversion(const uint32_t channel) const {
        ADC_ChannelConfTypeDef channel_config;
        /**Configure Regular Channel*/
        channel_config.Channel      = channel;
        channel_config.Rank         = 1;
        channel_config.SamplingTime = ADC_SAMPLETIME_3CYCLES_5;
        channel_config.Offset       = 0;

        if (HAL_ADC_ConfigChannel(&m_handle, &channel_config) != HAL_OK)
        {
            return utl::system_error::UNKNOWN;
        }

        if(HAL_ADC_Start(&m_handle) != HAL_OK)
        {
            /* Start Conversation Error */
            return utl::system_error::UNKNOWN;
        }

        /*##-4- Wait for the end of conversion #####################################*/  
        /*  Before starting a new conversion, you need to check the current state of 
            the peripheral; if it’s busy you need to wait for the end of current
            conversion before starting a new one.
            For simplicity reasons, this example is just waiting till the end of the 
            conversion, but application may perform other tasks while conversion 
            operation is ongoing. */
        HAL_ADC_PollForConversion(&m_handle, 10);

        /* Check if the continuous conversion of regular channel is finished */
        if((HAL_ADC_GetState(&m_handle) & HAL_ADC_STATE_EOC_REG) == HAL_ADC_STATE_EOC_REG)
        {
            /*##-5- Get the converted value of regular channel  ######################*/
            return static_cast<uint16_t>(HAL_ADC_GetValue(&m_handle));
        }

        return utl::system_error::UNKNOWN;
    }

    void set_timeout(uint32_t timeout) {
        if (timeout > 1000) {
            timeout = 1000;
        }
        m_timeout = timeout;
    }

    float to_voltage(uint16_t conversion) const {
        return supply()*static_cast<float>(conversion)/static_cast<float>(0xFFF);
    }

    float supply() const {
        return m_supply;
    }

    adc_channel_t channel(uint8_t channel) {
		return {this, channel};
	}
};

} //namespace stm32g4::driver

#endif
