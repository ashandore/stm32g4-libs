#ifndef STM32G4_LIBS_UART_HH_
#define STM32G4_LIBS_UART_HH_

#include "device/stm32g4xx.h"
#include "board.h"
#include <stdio.h>
#include <string.h>
#include <type-list.hh>


namespace drivers {
namespace stm32g4 {

template <bool Use_Float>
class uart {
	UART_HandleTypeDef m_handle;
public:
	uart(USART_TypeDef *module, uint32_t baud) : m_handle{} {
        /*##-1- Configure the UART peripheral ######################################*/
        /* Put the USART peripheral in the Asynchronous mode (UART Mode) */
        /* UART configured as follows:
          - Word Length = 8 Bits
          - Stop Bit = One Stop bit
          - Parity = None
          - BaudRate = 9600 baud
          - Hardware flow control disabled (RTS and CTS signals) */
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
        if(HAL_UART_DeInit(&m_handle) != HAL_OK) while(1);
        if(HAL_UART_Init(&m_handle) != HAL_OK) while(1);
    }

    template <typename... Args>
	void printf(char const *format, Args... args) {
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

        write(uart_buffer, length);
        write("\r\n", 2);
    }

	void write(char const *string, uint32_t length) {
        HAL_UART_Transmit(&m_handle, reinterpret_cast<uint8_t*>(const_cast<char*>(string)), 
            static_cast<uint16_t>(length), 5000);
    }
};

} //namespace stm32g4
} //namespace drivers

#endif
