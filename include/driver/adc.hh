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
    Adc const&      m_adc;
    uint32_t const  m_channel;
public:
    adc_channel(Adc const& master, uint32_t channel) : m_adc{master}, m_channel{channel} {}
    utl::result<uint16_t> conversion() const { return m_adc.conversion(m_channel); }
    float to_voltage(uint16_t conversion) const { return m_adc.to_voltage(conversion); }
    float supply() const { return m_adc.supply(); }
};

class adc : public utl::driver::interface::driver {
public:
    using adc_channel_t = adc_channel<adc>;
private:
    mutable ADC_HandleTypeDef   m_handle;
    uint32_t                    m_timeout;
    float                       m_supply;
public:
    adc(ADC_TypeDef *module, uint32_t timeout, float supply) :
        m_handle{}, m_timeout{timeout}, m_supply{supply}
    {
        if(m_timeout > 1000) {
            m_timeout = 1000;
        }

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
        return stm32g4::make_hal_error_code(res);
    }
public:
    void start() {}    
    void stop() {}

    utl::result<uint16_t> conversion(const uint32_t channel) const {
        ADC_ChannelConfTypeDef channel_config;
        /**Configure Regular Channel*/
        channel_config.Channel      = channel;
        channel_config.Rank         = ADC_REGULAR_RANK_1;
        channel_config.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
        channel_config.SingleDiff = ADC_SINGLE_ENDED;
        channel_config.OffsetNumber = ADC_OFFSET_NONE;
        channel_config.Offset       = 0;

        auto res = HAL_ADC_ConfigChannel(&m_handle, &channel_config);
        if(res != HAL_OK) return stm32g4::make_hal_error_code(res);

        res = HAL_ADC_Start(&m_handle);
        if(res != HAL_OK) return stm32g4::make_hal_error_code(res);

        HAL_ADC_PollForConversion(&m_handle, 10);

        /* Check if the continuous conversion of regular channel is finished */
        if((HAL_ADC_GetState(&m_handle) & HAL_ADC_STATE_EOC_REG) == HAL_ADC_STATE_EOC_REG)
        {
            return static_cast<uint16_t>(HAL_ADC_GetValue(&m_handle));
        }

        return stm32g4::hal_error::ERROR;
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
		return {*this, channel};
	}
};

} //namespace stm32g4::driver

#endif
