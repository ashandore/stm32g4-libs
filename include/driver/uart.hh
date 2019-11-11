#ifndef STM32G4_LIBS_UART_HH_
#define STM32G4_LIBS_UART_HH_

// #include "device/stm32g4xx.h"
#include "hal.hh"
#include "stm32g4xx_hal_uart.h"

#include <stdio.h>
#include <string.h>
#include <utl/type-list.hh>

#include <utl/result.hh>
#include <utl/interface/hal/driver.hh>

namespace stm32g4::driver {

template <bool Use_Float>
class uart : public utl::interface::hal::driver {
	mutable UART_HandleTypeDef m_handle;

protected:
	uart(USART_TypeDef *module, uint32_t baud) : m_handle{} {
        m_handle.Instance        = module;

        m_handle.Init.BaudRate   = baud;
        m_handle.Init.WordLength = UART_WORDLENGTH_8B;
        m_handle.Init.StopBits   = UART_STOPBITS_1;
        m_handle.Init.Parity     = UART_PARITY_NONE;
        m_handle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
        m_handle.Init.Mode       = UART_MODE_TX_RX;
        m_handle.Init.OverSampling = UART_OVERSAMPLING_16;
        m_handle.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
        m_handle.Init.ClockPrescaler = UART_PRESCALER_DIV1;
        m_handle.FifoMode = UART_FIFOMODE_DISABLE;
    }

    utl::result<void> validate() {
        auto res = HAL_UART_DeInit(&m_handle);
        if(res != HAL_OK) return stm32g4::make_hal_error_code(res);
        
        res = HAL_UART_Init(&m_handle);
        if(res != HAL_OK) return stm32g4::make_hal_error_code(res);

        return utl::success();
    }

public:
    template <typename... Args>
	void printf(char const *format, Args... args) const {
        static_assert(Use_Float || 
            (!utl::contains_v<utl::type_list<Args...>,float> && !utl::contains_v<utl::type_list<Args...>,double>),
            "floating point printing is disabled!");

        char uart_buffer[512] = {0};
        uint32_t length;

        if constexpr(Use_Float) {
            //Auto convert things to double?
            length = snprintf(uart_buffer, 512, format, args...);
        } else {
            length = sniprintf(uart_buffer, 512, format, args...);
        }

        auto res = write({uart_buffer, length});
        if(!res) {
            return;
        }
        utl::ignore_result(write({"\r\n", 2}));
    }

    utl::result<void> write(utl::string_view const& s) const {
        auto res = HAL_UART_Transmit(&m_handle, reinterpret_cast<uint8_t*>(const_cast<char*>(s.data())), 
            static_cast<uint16_t>(s.size()), 5000);
        if(res == HAL_OK) return utl::success();
        return stm32g4::make_hal_error_code(res);
    }
};

} //namespace stm32g4::driver

#endif
