
#ifndef STM32G4_LIBS_DRIVER_SPI_HH_
#define STM32G4_LIBS_DRIVER_SPI_HH_

#include "hal.hh"
#include "stm32g4xx_hal_spi.h"
#include <result.hh>
#include <interface/driver/driver.hh>
#include <driver/dma.hh>
#include <utl.hh>

namespace stm32g4::driver::spi {

enum class direction {
    both = SPI_DIRECTION_2LINES,
    rx_only = SPI_DIRECTION_2LINES_RXONLY,
    single_bidi = SPI_DIRECTION_1LINE
};

enum class polarity {
    POL0, //idle low
    POL1 //idle high
};

enum class phase {
    PHA0, //input data latched on leading edge, output changes on trailing edge
    PHA1 //input data latched on trailing edge, output changes on leading edge
};

enum class data_size {
    bits_4 = SPI_DATASIZE_4BIT,
    bits_5 = SPI_DATASIZE_5BIT,
    bits_6 = SPI_DATASIZE_6BIT,
    bits_7 = SPI_DATASIZE_7BIT,
    bits_8 = SPI_DATASIZE_8BIT,
    bits_9 = SPI_DATASIZE_9BIT,
    bits_10 = SPI_DATASIZE_10BIT,
    bits_11 = SPI_DATASIZE_11BIT,
    bits_12 = SPI_DATASIZE_12BIT,
    bits_13 = SPI_DATASIZE_13BIT,
    bits_14 = SPI_DATASIZE_14BIT,
    bits_15 = SPI_DATASIZE_15BIT,
    bits_16 = SPI_DATASIZE_16BIT
};

namespace detail {

struct decorated_tx_dma_channel : public dma::channel {
    decorated_tx_dma_channel(DMA_Channel_TypeDef* instance, dma::request req) :
        dma::channel{instance, req, dma::direction::memory_to_periph, 
            dma::p_increment::disable, dma::m_increment::enable,
            dma::p_align::byte, dma::m_align::byte, 
            dma::mode::normal, dma::priority::very_high} 
    {}
};

struct decorated_rx_dma_channel : public dma::channel {
    decorated_rx_dma_channel(DMA_Channel_TypeDef* instance, dma::request req) :
        dma::channel{instance, req, dma::direction::periph_to_memory, 
            dma::p_increment::disable, dma::m_increment::enable,
            dma::p_align::byte, dma::m_align::byte, 
            dma::mode::normal, dma::priority::very_high} 
    {}
};

}



//TODO: create a utl driver interface
class dma_master : public utl::driver::interface::driver {
public:
    using tx_dma_channel_t = detail::decorated_tx_dma_channel;
    using rx_dma_channel_t = detail::decorated_rx_dma_channel;
private:
    mutable SPI_HandleTypeDef               m_handle;
    const utl::imprecise<utl::freq::Hz>     m_clock_frequency;
    rx_dma_channel_t&                       m_rx_dma_channel;
    tx_dma_channel_t&                       m_tx_dma_channel;

    bool transfer_complete() const {
        return HAL_SPI_GetState(&m_handle) == HAL_SPI_STATE_READY;
    }

    utl::result<uint32_t> calculate_prescaler(utl::imprecise<utl::freq::Hz> clock_frequency) {
        utl::maybe_unused(clock_frequency);
        
        uint32_t base_clock, resulting_clock;
        if(m_handle.Instance == SPI2 || m_handle.Instance == SPI3) {
            //Base clock for this module is APB1
            base_clock = HAL_RCC_GetPCLK1Freq();
        } else if(m_handle.Instance == SPI1) {
            //Base clock for this module is APB2
            base_clock = HAL_RCC_GetPCLK2Freq();
        } else {
            return utl::system_error::UNKNOWN;
        }

        resulting_clock = base_clock;
        while(resulting_clock > 0 and !clock_frequency.contains(utl::freq::Hz(resulting_clock))) resulting_clock /= 2;

        if(resulting_clock < base_clock/256) {
            return utl::system_error::UNKNOWN;
        } else if(resulting_clock > base_clock/2) {
            return utl::system_error::UNKNOWN;
        } 

        switch(base_clock/resulting_clock) {
            case 2:
                return SPI_BAUDRATEPRESCALER_2;
            case 4:
                return SPI_BAUDRATEPRESCALER_4;
            case 8:
                return SPI_BAUDRATEPRESCALER_8;
            case 16:
                return SPI_BAUDRATEPRESCALER_16;
            case 32:
                return SPI_BAUDRATEPRESCALER_32;
            case 64:
                return SPI_BAUDRATEPRESCALER_64;
            case 128:
                return SPI_BAUDRATEPRESCALER_128;
            case 256:
                return SPI_BAUDRATEPRESCALER_256;
            default:
                return utl::system_error::UNKNOWN;
        }
    }

protected:
    constexpr dma_master(SPI_TypeDef *module, utl::imprecise<utl::freq::Hz> clock_frequency, 
        direction lines, polarity clock_pol, phase clock_pha, data_size word_length, 
        bool msb_first, bool ti_mode, bool soft_nss,
        rx_dma_channel_t& rx_dma_channel, tx_dma_channel_t& tx_dma_channel)
      : m_handle{}, m_clock_frequency{clock_frequency}, 
        m_rx_dma_channel{rx_dma_channel}, m_tx_dma_channel{tx_dma_channel}
    {
        m_handle.Instance               = module;
        m_handle.Init.Direction         = static_cast<uint32_t>(lines);
        m_handle.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
        m_handle.Init.CRCPolynomial     = 7;
        m_handle.Init.DataSize          = static_cast<uint32_t>(word_length);
        m_handle.Init.FirstBit          = msb_first ? SPI_FIRSTBIT_MSB : SPI_FIRSTBIT_LSB;
        m_handle.Init.NSS               = soft_nss ? SPI_NSS_SOFT : SPI_NSS_HARD_OUTPUT;
        m_handle.Init.TIMode            = ti_mode ? SPI_TIMODE_ENABLE : SPI_TIMODE_DISABLE;
        m_handle.Init.Mode              = SPI_MODE_MASTER;
        m_handle.Init.NSSPMode          = soft_nss ? SPI_NSS_PULSE_DISABLE : SPI_NSS_PULSE_ENABLE;

        if(clock_pha == phase::PHA0) m_handle.Init.CLKPhase = SPI_PHASE_1EDGE;
        else m_handle.Init.CLKPhase = SPI_PHASE_2EDGE;

        if(clock_pol == polarity::POL0) m_handle.Init.CLKPolarity = SPI_POLARITY_LOW;
        else m_handle.Init.CLKPolarity = SPI_POLARITY_HIGH;

        m_handle.hdmarx = rx_dma_channel.handle();
        m_handle.hdmatx = tx_dma_channel.handle();
    }

    utl::result<void> validate() {
        auto prescaler = calculate_prescaler(m_clock_frequency);
        if(!prescaler) return utl::system_error::UNKNOWN;
        m_handle.Init.BaudRatePrescaler = prescaler.value();

        auto res = HAL_SPI_Init(&m_handle);
        if(res != HAL_OK) return stm32g4::make_hal_error_code(res);        

        IRQn_Type spi_irq;
        if(m_handle.Instance == SPI1) spi_irq = SPI1_IRQn;
        else if(m_handle.Instance == SPI2) spi_irq = SPI2_IRQn;
        else if(m_handle.Instance == SPI3) spi_irq = SPI3_IRQn;
        else return utl::system_error::UNKNOWN;

        //FIXME: this configuration needs to go elsewhere.
        HAL_NVIC_SetPriority(spi_irq, 1, 2);
        HAL_NVIC_EnableIRQ(spi_irq);

        m_rx_dma_channel.link(&m_handle);
        m_tx_dma_channel.link(&m_handle);

        return utl::success();
    }

public:    

    void wait() const {
        while(!transfer_complete());
    }

    void service() {
        HAL_SPI_IRQHandler(&m_handle);
    }

    utl::result<void> transact(uint8_t* write, uint8_t* receive, uint32_t length) {
        wait();
        auto res = HAL_SPI_TransmitReceive_DMA(&m_handle, 
            write, 
            receive, 
            length
        );

        if(res != HAL_OK) return stm32g4::make_hal_error_code(res);
        return utl::success();
    }
};

} //namespace stm32g4::driver::spi

#endif //STM32G4_LIBS_DRIVER_SPI_HH_
