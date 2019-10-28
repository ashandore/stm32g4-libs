#ifndef STM32G4_LIBS_DMA_REQUESTS_HH_
#define STM32G4_LIBS_DMA_REQUESTS_HH_

#include "stm32g4xx_hal_dma.h"

namespace stm32g4::driver::dma {

enum class request {
    #ifdef     DMA_REQUEST_MEM2MEM
    mem2mem = DMA_REQUEST_MEM2MEM,
    #endif
    #ifdef DMA_REQUEST_GENERATOR0
    generator0 = DMA_REQUEST_GENERATOR0,
    #endif
    #ifdef DMA_REQUEST_GENERATOR1
    generator1 = DMA_REQUEST_GENERATOR1,
    #endif
    #ifdef DMA_REQUEST_GENERATOR2
    generator2 = DMA_REQUEST_GENERATOR2,
    #endif
    #ifdef DMA_REQUEST_GENERATOR3
    generator3 = DMA_REQUEST_GENERATOR3,
    #endif
    #ifdef DMA_REQUEST_ADC1
    adc1 = DMA_REQUEST_ADC1,
    #endif
    #ifdef DMA_REQUEST_DAC1_CHANNEL1
    dac1_channel1 = DMA_REQUEST_DAC1_CHANNEL1,
    #endif
    #ifdef DMA_REQUEST_DAC1_CHANNEL2
    dac1_channel2 = DMA_REQUEST_DAC1_CHANNEL2,
    #endif
    #ifdef DMA_REQUEST_TIM6_UP
    tim6_up = DMA_REQUEST_TIM6_UP,
    #endif
    #ifdef DMA_REQUEST_TIM7_UP
    tim7_up = DMA_REQUEST_TIM7_UP,
    #endif
    #ifdef DMA_REQUEST_SPI1_RX
    spi1_rx = DMA_REQUEST_SPI1_RX,
    #endif
    #ifdef DMA_REQUEST_SPI1_TX
    spi1_tx = DMA_REQUEST_SPI1_TX,
    #endif
    #ifdef DMA_REQUEST_SPI2_RX
    spi2_rx = DMA_REQUEST_SPI2_RX,
    #endif
    #ifdef DMA_REQUEST_SPI2_TX
    spi2_tx = DMA_REQUEST_SPI2_TX,
    #endif
    #ifdef DMA_REQUEST_SPI3_RX
    spi3_rx = DMA_REQUEST_SPI3_RX,
    #endif
    #ifdef DMA_REQUEST_SPI3_TX
    spi3_tx = DMA_REQUEST_SPI3_TX,
    #endif
    #ifdef DMA_REQUEST_I2C1_RX
    i2c1_rx = DMA_REQUEST_I2C1_RX,
    #endif
    #ifdef DMA_REQUEST_I2C1_TX
    i2c1_tx = DMA_REQUEST_I2C1_TX,
    #endif
    #ifdef DMA_REQUEST_I2C2_RX
    i2c2_rx = DMA_REQUEST_I2C2_RX,
    #endif
    #ifdef DMA_REQUEST_I2C2_TX
    i2c2_tx = DMA_REQUEST_I2C2_TX,
    #endif
    #ifdef DMA_REQUEST_I2C3_RX
    i2c3_rx = DMA_REQUEST_I2C3_RX,
    #endif
    #ifdef DMA_REQUEST_I2C3_TX
    i2c3_tx = DMA_REQUEST_I2C3_TX,
    #endif
    #ifdef DMA_REQUEST_I2C4_RX
    i2c4_rx = DMA_REQUEST_I2C4_RX,
    #endif
    #ifdef DMA_REQUEST_I2C4_TX
    i2c4_tx = DMA_REQUEST_I2C4_TX,
    #endif
    #ifdef DMA_REQUEST_USART1_RX
    usart1_rx = DMA_REQUEST_USART1_RX,
    #endif
    #ifdef DMA_REQUEST_USART1_TX
    usart1_tx = DMA_REQUEST_USART1_TX,
    #endif
    #ifdef DMA_REQUEST_USART2_RX
    usart2_rx = DMA_REQUEST_USART2_RX,
    #endif
    #ifdef DMA_REQUEST_USART2_TX
    usart2_tx = DMA_REQUEST_USART2_TX,
    #endif
    #ifdef DMA_REQUEST_USART3_RX
    usart3_rx = DMA_REQUEST_USART3_RX,
    #endif
    #ifdef DMA_REQUEST_USART3_TX
    usart3_tx = DMA_REQUEST_USART3_TX,
    #endif
    #ifdef DMA_REQUEST_UART4_RX
    uart4_rx = DMA_REQUEST_UART4_RX,
    #endif
    #ifdef DMA_REQUEST_UART4_TX
    uart4_tx = DMA_REQUEST_UART4_TX,
    #endif
    #ifdef DMA_REQUEST_UART5_RX
    uart5_rx = DMA_REQUEST_UART5_RX,
    #endif
    #ifdef DMA_REQUEST_UART5_TX
    uart5_tx = DMA_REQUEST_UART5_TX,
    #endif
    #ifdef DMA_REQUEST_LPUART1_RX
    lpuart1_rx = DMA_REQUEST_LPUART1_RX,
    #endif
    #ifdef DMA_REQUEST_LPUART1_TX
    lpuart1_tx = DMA_REQUEST_LPUART1_TX,
    #endif
    #ifdef DMA_REQUEST_ADC2
    adc2 = DMA_REQUEST_ADC2,
    #endif
    #ifdef DMA_REQUEST_ADC3
    adc3 = DMA_REQUEST_ADC3,
    #endif
    #ifdef DMA_REQUEST_ADC4
    adc4 = DMA_REQUEST_ADC4,
    #endif
    #ifdef DMA_REQUEST_ADC5
    adc5 = DMA_REQUEST_ADC5,
    #endif
    #ifdef DMA_REQUEST_QUADSPI
    quadspi = DMA_REQUEST_QUADSPI,
    #endif
    #ifdef DMA_REQUEST_DAC2_CHANNEL1
    dac2_channel1 = DMA_REQUEST_DAC2_CHANNEL1,
    #endif
    #ifdef DMA_REQUEST_TIM1_CH1
    tim1_ch1 = DMA_REQUEST_TIM1_CH1,
    #endif
    #ifdef DMA_REQUEST_TIM1_CH2
    tim1_ch2 = DMA_REQUEST_TIM1_CH2,
    #endif
    #ifdef DMA_REQUEST_TIM1_CH3
    tim1_ch3 = DMA_REQUEST_TIM1_CH3,
    #endif
    #ifdef DMA_REQUEST_TIM1_CH4
    tim1_ch4 = DMA_REQUEST_TIM1_CH4,
    #endif
    #ifdef DMA_REQUEST_TIM1_UP
    tim1_up = DMA_REQUEST_TIM1_UP,
    #endif
    #ifdef DMA_REQUEST_TIM1_TRIG
    tim1_trig = DMA_REQUEST_TIM1_TRIG,
    #endif
    #ifdef DMA_REQUEST_TIM1_COM
    tim1_com = DMA_REQUEST_TIM1_COM,
    #endif
    #ifdef DMA_REQUEST_TIM8_CH1
    tim8_ch1 = DMA_REQUEST_TIM8_CH1,
    #endif
    #ifdef DMA_REQUEST_TIM8_CH2
    tim8_ch2 = DMA_REQUEST_TIM8_CH2,
    #endif
    #ifdef DMA_REQUEST_TIM8_CH3
    tim8_ch3 = DMA_REQUEST_TIM8_CH3,
    #endif
    #ifdef DMA_REQUEST_TIM8_CH4
    tim8_ch4 = DMA_REQUEST_TIM8_CH4,
    #endif
    #ifdef DMA_REQUEST_TIM8_UP
    tim8_up = DMA_REQUEST_TIM8_UP,
    #endif
    #ifdef DMA_REQUEST_TIM8_TRIG
    tim8_trig = DMA_REQUEST_TIM8_TRIG,
    #endif
    #ifdef DMA_REQUEST_TIM8_COM
    tim8_com = DMA_REQUEST_TIM8_COM,
    #endif
    #ifdef DMA_REQUEST_TIM2_CH1
    tim2_ch1 = DMA_REQUEST_TIM2_CH1,
    #endif
    #ifdef DMA_REQUEST_TIM2_CH2
    tim2_ch2 = DMA_REQUEST_TIM2_CH2,
    #endif
    #ifdef DMA_REQUEST_TIM2_CH3
    tim2_ch3 = DMA_REQUEST_TIM2_CH3,
    #endif
    #ifdef DMA_REQUEST_TIM2_CH4
    tim2_ch4 = DMA_REQUEST_TIM2_CH4,
    #endif
    #ifdef DMA_REQUEST_TIM2_UP
    tim2_up = DMA_REQUEST_TIM2_UP,
    #endif
    #ifdef DMA_REQUEST_TIM3_CH1
    tim3_ch1 = DMA_REQUEST_TIM3_CH1,
    #endif
    #ifdef DMA_REQUEST_TIM3_CH2
    tim3_ch2 = DMA_REQUEST_TIM3_CH2,
    #endif
    #ifdef DMA_REQUEST_TIM3_CH3
    tim3_ch3 = DMA_REQUEST_TIM3_CH3,
    #endif
    #ifdef DMA_REQUEST_TIM3_CH4
    tim3_ch4 = DMA_REQUEST_TIM3_CH4,
    #endif
    #ifdef DMA_REQUEST_TIM3_UP
    tim3_up = DMA_REQUEST_TIM3_UP,
    #endif
    #ifdef DMA_REQUEST_TIM3_TRIG
    tim3_trig = DMA_REQUEST_TIM3_TRIG,
    #endif
    #ifdef DMA_REQUEST_TIM4_CH1
    tim4_ch1 = DMA_REQUEST_TIM4_CH1,
    #endif
    #ifdef DMA_REQUEST_TIM4_CH2
    tim4_ch2 = DMA_REQUEST_TIM4_CH2,
    #endif
    #ifdef DMA_REQUEST_TIM4_CH3
    tim4_ch3 = DMA_REQUEST_TIM4_CH3,
    #endif
    #ifdef DMA_REQUEST_TIM4_CH4
    tim4_ch4 = DMA_REQUEST_TIM4_CH4,
    #endif
    #ifdef DMA_REQUEST_TIM4_UP
    tim4_up = DMA_REQUEST_TIM4_UP,
    #endif
    #ifdef DMA_REQUEST_TIM5_CH1
    tim5_ch1 = DMA_REQUEST_TIM5_CH1,
    #endif
    #ifdef DMA_REQUEST_TIM5_CH2
    tim5_ch2 = DMA_REQUEST_TIM5_CH2,
    #endif
    #ifdef DMA_REQUEST_TIM5_CH3
    tim5_ch3 = DMA_REQUEST_TIM5_CH3,
    #endif
    #ifdef DMA_REQUEST_TIM5_CH4
    tim5_ch4 = DMA_REQUEST_TIM5_CH4,
    #endif
    #ifdef DMA_REQUEST_TIM5_UP
    tim5_up = DMA_REQUEST_TIM5_UP,
    #endif
    #ifdef DMA_REQUEST_TIM5_TRIG
    tim5_trig = DMA_REQUEST_TIM5_TRIG,
    #endif
    #ifdef DMA_REQUEST_TIM15_CH1
    tim15_ch1 = DMA_REQUEST_TIM15_CH1,
    #endif
    #ifdef DMA_REQUEST_TIM15_UP
    tim15_up = DMA_REQUEST_TIM15_UP,
    #endif
    #ifdef DMA_REQUEST_TIM15_TRIG
    tim15_trig = DMA_REQUEST_TIM15_TRIG,
    #endif
    #ifdef DMA_REQUEST_TIM15_COM
    tim15_com = DMA_REQUEST_TIM15_COM,
    #endif
    #ifdef DMA_REQUEST_TIM16_CH1
    tim16_ch1 = DMA_REQUEST_TIM16_CH1,
    #endif
    #ifdef DMA_REQUEST_TIM16_UP
    tim16_up = DMA_REQUEST_TIM16_UP,
    #endif
    #ifdef DMA_REQUEST_TIM17_CH1
    tim17_ch1 = DMA_REQUEST_TIM17_CH1,
    #endif
    #ifdef DMA_REQUEST_TIM17_UP
    tim17_up = DMA_REQUEST_TIM17_UP,
    #endif
    #ifdef DMA_REQUEST_TIM20_CH1
    tim20_ch1 = DMA_REQUEST_TIM20_CH1,
    #endif
    #ifdef DMA_REQUEST_TIM20_CH2
    tim20_ch2 = DMA_REQUEST_TIM20_CH2,
    #endif
    #ifdef DMA_REQUEST_TIM20_CH3
    tim20_ch3 = DMA_REQUEST_TIM20_CH3,
    #endif
    #ifdef DMA_REQUEST_TIM20_CH4
    tim20_ch4 = DMA_REQUEST_TIM20_CH4,
    #endif
    #ifdef DMA_REQUEST_TIM20_UP
    tim20_up = DMA_REQUEST_TIM20_UP,
    #endif
    #ifdef DMA_REQUEST_AES_IN
    aes_in = DMA_REQUEST_AES_IN,
    #endif
    #ifdef DMA_REQUEST_AES_OUT
    aes_out = DMA_REQUEST_AES_OUT,
    #endif
    #ifdef DMA_REQUEST_TIM20_TRIG
    tim20_trig = DMA_REQUEST_TIM20_TRIG,
    #endif
    #ifdef DMA_REQUEST_TIM20_COM
    tim20_com = DMA_REQUEST_TIM20_COM,
    #endif
    #ifdef DMA_REQUEST_HRTIM1_M
    hrtim1_m = DMA_REQUEST_HRTIM1_M,
    #endif
    #ifdef DMA_REQUEST_HRTIM1_A
    hrtim1_a = DMA_REQUEST_HRTIM1_A,
    #endif
    #ifdef DMA_REQUEST_HRTIM1_B
    hrtim1_b = DMA_REQUEST_HRTIM1_B,
    #endif
    #ifdef DMA_REQUEST_HRTIM1_C
    hrtim1_c = DMA_REQUEST_HRTIM1_C,
    #endif
    #ifdef DMA_REQUEST_HRTIM1_D
    hrtim1_d = DMA_REQUEST_HRTIM1_D,
    #endif
    #ifdef DMA_REQUEST_HRTIM1_E
    hrtim1_e = DMA_REQUEST_HRTIM1_E,
    #endif
    #ifdef DMA_REQUEST_HRTIM1_F
    hrtim1_f = DMA_REQUEST_HRTIM1_F,
    #endif
    #ifdef DMA_REQUEST_DAC3_CHANNEL1
    dac3_channel1 = DMA_REQUEST_DAC3_CHANNEL1,
    #endif
    #ifdef DMA_REQUEST_DAC3_CHANNEL2
    dac3_channel2 = DMA_REQUEST_DAC3_CHANNEL2,
    #endif
    #ifdef DMA_REQUEST_DAC4_CHANNEL1
    dac4_channel1 = DMA_REQUEST_DAC4_CHANNEL1,
    #endif
    #ifdef DMA_REQUEST_DAC4_CHANNEL2
    dac4_channel2 = DMA_REQUEST_DAC4_CHANNEL2,
    #endif
    #ifdef DMA_REQUEST_SPI4_RX
    spi4_rx = DMA_REQUEST_SPI4_RX,
    #endif
    #ifdef DMA_REQUEST_SPI4_TX
    spi4_tx = DMA_REQUEST_SPI4_TX,
    #endif
    #ifdef DMA_REQUEST_SAI1_A
    sai1_a = DMA_REQUEST_SAI1_A,
    #endif
    #ifdef DMA_REQUEST_SAI1_B
    sai1_b = DMA_REQUEST_SAI1_B,
    #endif
    #ifdef DMA_REQUEST_FMAC_READ
    fmac_read = DMA_REQUEST_FMAC_READ,
    #endif
    #ifdef DMA_REQUEST_FMAC_WRITE
    fmac_write = DMA_REQUEST_FMAC_WRITE,
    #endif
    #ifdef DMA_REQUEST_CORDIC_READ
    cordic_read = DMA_REQUEST_CORDIC_READ,
    #endif
    #ifdef DMA_REQUEST_CORDIC_WRITE
    cordic_write = DMA_REQUEST_CORDIC_WRITE,
    #endif
    #ifdef DMA_REQUEST_UCPD1_RX
    ucpd1_rx = DMA_REQUEST_UCPD1_RX,
    #endif
};

} //namespace stm32g4::driver::dma

#endif //STM32G4_LIBS_DMA_REQUESTS_HH_
