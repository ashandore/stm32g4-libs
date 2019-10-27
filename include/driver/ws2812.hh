#ifndef STM32G4_LIBS_WS2812_HH_
#define STM32G4_LIBS_WS2812_HH_

#include <utl.hh>
#include <interface/driver/driver.hh>

#include "hal.hh"
#include "stm32g4xx_hal_dma.h"
#include "stm32g4xx_hal_spi.h"

namespace ws2812 {

struct color {
    union {
        struct {
            uint8_t g;
            uint8_t r;
            uint8_t b;
            uint8_t _;
        };
        uint32_t data;
    };
    constexpr color(uint8_t r, uint8_t g, uint8_t b) : g{g}, r{r}, b{b}, _{} {}
    constexpr color() : g{0}, r{0}, b{0}, _{} {}
};

static constexpr color red = {0xff,0,0};
static constexpr color green = {0,0xff,0};
static constexpr color blue = {0,0,0xff};

#if 0
template <uint32_t N_LEDS>
class spi : public utl::driver::interface::driver {
    SPI_HandleTypeDef  			m_spiHandle;
    DMA_HandleTypeDef 			m_txDMA;
    DMA_HandleTypeDef 			m_rxDMA;
    color                       m_rgb_data[N_LEDS];
    uint8_t                     m_dma_buffer[N_LEDS*3*4];

    inline void start_dma() {
		HAL_SPI_TransmitReceive_DMA(&m_spiHandle, 
			const_cast<uint8_t*>(m_dma_buffer), 
			NULL, 
			sizeof(m_dma_buffer)
		);
	}

    inline bool dma_transfer_complete() {
		return HAL_SPI_GetState(&m_spiHandle) == HAL_SPI_STATE_READY;
	}

    utl::result<void> setup_dma() {
        /* DMA controller clock enable */
        __HAL_RCC_DMA1_CLK_ENABLE();    
    //FIXME: rxdma maybe isn't necessary.
        m_rxDMA.Instance                 = DMA1_Stream3;
        m_rxDMA.Init.Channel             = DMA_CHANNEL_0;
        m_rxDMA.Init.Direction           = DMA_PERIPH_TO_MEMORY;
        m_rxDMA.Init.PeriphInc           = DMA_PINC_DISABLE;
        m_rxDMA.Init.MemInc              = DMA_MINC_ENABLE;
        m_rxDMA.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        m_rxDMA.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
        m_rxDMA.Init.Mode                = DMA_NORMAL;
        m_rxDMA.Init.Priority            = DMA_PRIORITY_HIGH;
        m_rxDMA.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;         
        m_rxDMA.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
        m_rxDMA.Init.MemBurst            = DMA_MBURST_INC4;
        m_rxDMA.Init.PeriphBurst         = DMA_PBURST_INC4; 
        HAL_DMA_Init(&m_rxDMA);   

        /* Associate the initialized DMA handle to the the SPI handle */
        __HAL_LINKDMA(&m_spiHandle, hdmarx, m_rxDMA);

        m_txDMA.Instance                 = DMA1_Stream4;
        m_txDMA.Init.Channel             = DMA_CHANNEL_0;
        m_txDMA.Init.Direction           = DMA_MEMORY_TO_PERIPH;
        m_txDMA.Init.PeriphInc           = DMA_PINC_DISABLE;
        m_txDMA.Init.MemInc              = DMA_MINC_ENABLE;
        m_txDMA.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        m_txDMA.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
        m_txDMA.Init.Mode                = DMA_NORMAL;
        m_txDMA.Init.Priority            = DMA_PRIORITY_LOW;
        m_txDMA.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;         
        m_txDMA.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
        m_txDMA.Init.MemBurst            = DMA_MBURST_INC4;
        m_txDMA.Init.PeriphBurst         = DMA_PBURST_INC4;
        HAL_DMA_Init(&m_txDMA);   

        /* Associate the initialized DMA handle to the the SPI handle */
        __HAL_LINKDMA(&m_spiHandle, hdmatx, m_txDMA);

        //Configure interrupts
        HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 1, 1);
        HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);

        /* NVIC configuration for DMA transfer complete interrupt (SPI3_RX) */
        HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 1, 1);   
        HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);

        //NVIC for SPI
        IRQn_Type spi_irq;

        if(m_spiHandle.Instance == SPI1) spi_irq = SPI1_IRQn;
        else if(m_spiHandle.Instance == SPI2) spi_irq = SPI2_IRQn;
        else if(m_spiHandle.Instance == SPI3) spi_irq = SPI3_IRQn;
        else if(m_spiHandle.Instance == SPI4) spi_irq = SPI4_IRQn;
        // else luminaire::exception("unrecognized spi module");

        HAL_NVIC_SetPriority(spi_irq, 1, 2);
        HAL_NVIC_EnableIRQ(spi_irq);

        //Do the first transfer to set things up
        HAL_SPI_TransmitReceive_DMA(&m_spiHandle, 
			const_cast<uint8_t*>(m_dma_buffer), 
			NULL, 
			sizeof(m_dma_buffer)
		);

        wait();
        return utl::success();
    }

protected: 
    spi(SPI_TypeDef* module) : m_spiHandle{}, m_txDMA{}, m_rxDMA{},
        m_rgb_data{}, m_dma_buffer{}
    {}

    utl::result<void> validate() {
        //Initialize the SPI module
        m_spiHandle.Instance               = module;
        m_spiHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
        m_spiHandle.Init.Direction         = SPI_DIRECTION_2LINES;
        m_spiHandle.Init.CLKPhase          = SPI_PHASE_1EDGE;
        m_spiHandle.Init.CLKPolarity       = SPI_POLARITY_LOW;
        m_spiHandle.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
        m_spiHandle.Init.CRCPolynomial     = 7;
        m_spiHandle.Init.DataSize          = SPI_DATASIZE_8BIT;
        m_spiHandle.Init.FirstBit          = SPI_FIRSTBIT_MSB;
        m_spiHandle.Init.NSS               = SPI_NSS_HARD_OUTPUT;
        m_spiHandle.Init.TIMode            = SPI_TIMODE_DISABLE;
        m_spiHandle.Init.Mode              = SPI_MODE_MASTER;

        if(HAL_SPI_Init(&m_spiHandle) != HAL_OK)
        {
        /* Initialization Error */
            while(1);
        }
    }

public:
    color& operator[](size_t idx) { return m_rgb_data[idx]; }

    constexpr size_t count() { return N_LEDS; }

    utl::result<void> write() {
		//Block if a transfer is already in progress
		wait();
		start_dma();
        return utl::success();
	}

	inline void wait() {
		while(!dma_transfer_complete());
	}


    

    void service_dma_rx() {
        HAL_DMA_IRQHandler(m_spiHandle.hdmarx);
    }

    void service_dma_tx() {
        HAL_DMA_IRQHandler(m_spiHandle.hdmatx);
    }

    void service_spi() {
        HAL_SPI_IRQHandler(&m_spiHandle);
    }


};


#endif //0




template <uint32_t N_LEDS, typename Pwm, typename Dma>
class pwm : public utl::driver::interface::driver {
    Pwm&                            m_pwm_source;
    typename Pwm::dma_channel_t&    m_pwm_channel;
    Dma&                            m_dma_config;
    color                           m_rgb_data[N_LEDS];
    uint32_t                         m_write_buffer[N_LEDS*3*8 + 10];
    uint8_t                         m_active_light;
    uint8_t                         m_bit;
    uint32_t                        m_shift_out;
    bool                            m_running;
    bool                            m_write_pending;
    uint32_t                        m_write_idx;
protected:
    pwm(Pwm& pwm_source, typename Pwm::dma_channel_t& pwm_channel, Dma& dma)
        : m_pwm_source{pwm_source}, m_pwm_channel{pwm_channel}, m_dma_config{dma},
        m_rgb_data{}, m_write_buffer{}, m_active_light{0}, m_bit{0}, m_shift_out{0},
        m_running{false}, m_write_pending{false}, m_write_idx{0}
    {
        m_pwm_channel.set_polarity(Pwm::polarity_t::ACTIVE_LOW);
        m_pwm_source.set_period_ns(1250u);
        

        //fixme: this is channel specific!
        m_pwm_source.link_dma(m_dma_config, TIM_DMA_ID_CC1);
    }

    utl::result<void> validate() {
        return m_pwm_source.start();
    }

    uint32_t get_width(bool bit) {
        // auto pwm_period_ns = m_pwm_source.period_ns();
        if(bit) {
            // return 600u;
            return 100u;
            // m_pwm_channel.set_width_ns(600u);
        } else {
            return 50u;
            // return 300u;
            // m_pwm_channel.set_width_ns(300u);
        }
    }
public:
    color& operator[](size_t idx) { return m_rgb_data[idx]; }

    constexpr size_t count() { return N_LEDS; }

    void service_dma() {
        m_dma_config.service();
    }

    utl::result<void> write() {
        // m_active_light = N_LEDS ;
        // m_bits_remain = 24;
        // m_shift_out = m_rgb_data[m_active_light-1].data;

        m_active_light = N_LEDS;
        m_bit = 0;


        // for()

        while(m_active_light) {
            while(m_bit < 24) {
                uint32_t read_index = m_active_light - 1;
                uint32_t write_index = read_index*24 + m_bit;
                m_write_buffer[write_index + 9] 
                    = get_width((m_rgb_data[read_index].data >> m_bit) & 0b1);

                // utl::log("wrote %d (%d) to %d", m_write_buffer[write_index], (m_rgb_data[read_index].data >> m_bit) & 0b1, write_index);

                m_bit++;

                
            }
            m_active_light--;
            m_bit = 0;
        }

        m_write_buffer[sizeof(m_write_buffer)/sizeof(m_write_buffer[0]) - 1] = 0;
        // for(uint32_t idx = 0; idx < sizeof(m_write_buffer)/sizeof(m_write_buffer[0]); idx++) {
        //     utl::log("%d, %d: %d", idx, idx % 24, m_write_buffer[idx]);
        // }
        // set_width(m_shift_out & 0b1);
        // m_shift_out >>= 1;
        // m_bits_remain--;
        m_write_idx = 0;
        m_pwm_channel.set_width_ns(0);
        m_write_pending = true;        
        m_running = false;
        
       return m_pwm_channel.start(m_write_buffer, sizeof(m_write_buffer));
    }

    void update_pwm() {
        m_pwm_source.clear_interrupt();

        if(m_write_pending) {
            m_write_pending = false;
            m_running = true;

            m_pwm_channel.set_width_ns(0);

            utl::ignore_result(m_pwm_channel.start());
        } else if(m_running) {
            set_width(m_write_buffer[m_write_idx++]);
            if(m_write_idx == sizeof(m_write_buffer)) {
                m_pwm_channel.set_width_ns(0);
                m_running = false;
                // utl::ignore_result(m_pwm_source.stop());
                //fixme: should this be doing some kind of pulse counting?
                utl::ignore_result(m_pwm_channel.stop()); 
            }
        }
        // else if(m_running) {
        //     if(m_bits_remain > 1) {
        //         set_width(m_shift_out & 0b1);
        //         m_bits_remain--;
        //         m_shift_out >>= 1;
        //     } else if(m_bits_remain == 1) {
        //         m_pwm_channel.set_width_ns(0);
        //         m_bits_remain--;
        //     } else if(m_bits_remain == 0) {
        //         m_shift_out = m_rgb_data[m_active_light-1].data;
        //         m_bits_remain = 23;
        //         if(m_active_light == 0) {
        //             m_pwm_channel.set_width_ns(0);
        //             m_running = false;
        //             utl::ignore_result(m_pwm_source.stop());
        //             //fixme: should this be doing some kind of pulse counting?
        //             utl::ignore_result(m_pwm_channel.stop());
        //         }
        //         m_active_light--;
        //     }            
        // }        
    }
};

} //namespace ws2812

#endif //STM32G4_LIBS_WS2812_HH_
