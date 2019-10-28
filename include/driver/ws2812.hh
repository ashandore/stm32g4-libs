#ifndef STM32G4_LIBS_WS2812_HH_
#define STM32G4_LIBS_WS2812_HH_

#include <utl.hh>
#include <interface/driver/driver.hh>

#include "hal.hh"
#include "stm32g4xx_hal_dma.h"
#include "stm32g4xx_hal_spi.h"

namespace stm32g4::driver {

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

namespace detail::ws2812 {

template <typename T>
struct dma_channel : T {
    dma_channel(DMA_Channel_TypeDef* instance, dma::request req) :
        T{instance, req, dma::direction::memory_to_periph, 
        dma::p_increment::disable, dma::m_increment::enable,
        dma::p_align::word, dma::m_align::word, 
        dma::mode::normal, dma::priority::very_high} 
    {}
};

} //namespace detail::ws2812


//FIXME: might want to factor out some of the drawing responsibilities; this
//class has too many things going on.
template <uint32_t N_LEDS, typename Pwm, typename DmaChannel>
class ws2812 : public utl::driver::interface::driver {
public:
    using dma_channel_t = detail::ws2812::dma_channel<DmaChannel>;
    using pwm_channel_t = typename Pwm::dma_channel_t;
private:
    Pwm&                            m_pwm_source;
    pwm_channel_t&                  m_pwm_channel;
    dma_channel_t&                  m_dma_channel;
    color                           m_rgb_data[N_LEDS];
    uint32_t                        m_write_buffer[N_LEDS*3*8 + 10];
    uint8_t                         m_active_light;
    uint8_t                         m_bit;
    uint32_t                        m_shift_out;
    bool                            m_running;
    bool                            m_write_pending;
    uint32_t                        m_write_idx;
protected:

    ws2812(Pwm& pwm_source, pwm_channel_t& pwm_channel, dma_channel_t& dma_channel)
        : m_pwm_source{pwm_source}, m_pwm_channel{pwm_channel}, m_dma_channel{dma_channel},
        m_rgb_data{}, m_write_buffer{}, m_active_light{0}, m_bit{0}, m_shift_out{0},
        m_running{false}, m_write_pending{false}, m_write_idx{0}
    {
        m_pwm_channel.set_polarity(Pwm::polarity_t::ACTIVE_LOW);
        m_pwm_source.set_period_ns(1250u);
        

        //fixme: this is channel specific!
        m_pwm_source.link_dma(m_dma_channel, TIM_DMA_ID_CC1);
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
        m_dma_channel.service();
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
                utl::ignore_result(m_pwm_channel.stop()); 
            }
        }       
    }
};

} //namespace stm32g4::driver

#endif //STM32G4_LIBS_WS2812_HH_
