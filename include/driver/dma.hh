#ifndef STM32G4_LIBS_DRIVER_DMA_HH_
#define STM32G4_LIBS_DRIVER_DMA_HH_

#include <utl.hh>
#include "hal.hh"
#include "dma-requests.hh"

namespace stm32g4::driver::dma {

enum class direction {
    periph_to_memory = DMA_PERIPH_TO_MEMORY,
    memory_to_periph = DMA_MEMORY_TO_PERIPH,
    memory_to_memory = DMA_MEMORY_TO_MEMORY
};

enum class m_increment {
    enable = DMA_MINC_ENABLE,
    disable = DMA_MINC_DISABLE
};

enum class p_increment {
    enable = DMA_PINC_ENABLE,
    disable = DMA_PINC_DISABLE
};

enum class p_align {
    byte = DMA_PDATAALIGN_BYTE,
    hword = DMA_PDATAALIGN_HALFWORD,
    word = DMA_PDATAALIGN_WORD
};

enum class m_align {
    byte = DMA_MDATAALIGN_BYTE,
    hword = DMA_MDATAALIGN_HALFWORD,
    word = DMA_MDATAALIGN_WORD
};

enum class mode {
    normal = DMA_NORMAL,
    circular = DMA_CIRCULAR
};

enum class priority {
    low = DMA_PRIORITY_LOW,
    medium = DMA_PRIORITY_MEDIUM,
    high = DMA_PRIORITY_HIGH,
    very_high = DMA_PRIORITY_VERY_HIGH
};

class channel {
    DMA_HandleTypeDef   m_handle;
protected:
    channel(DMA_Channel_TypeDef* instance, request req, direction dir,
        p_increment periph_inc, m_increment mem_inc, p_align periph_data_align,
        m_align mem_data_align, mode mode, priority priority) 
        : m_handle{} 
    {
        m_handle.Instance = instance;
        m_handle.Init.Request = static_cast<uint32_t>(req);
        m_handle.Init.Direction = static_cast<uint32_t>(dir);
        m_handle.Init.PeriphInc = static_cast<uint32_t>(periph_inc);
        m_handle.Init.MemInc = static_cast<uint32_t>(mem_inc);
        m_handle.Init.PeriphDataAlignment = static_cast<uint32_t>(periph_data_align);
        m_handle.Init.MemDataAlignment = static_cast<uint32_t>(mem_data_align);
        m_handle.Init.Mode = static_cast<uint32_t>(mode);
        m_handle.Init.Priority = static_cast<uint32_t>(priority);

        __HAL_RCC_DMAMUX1_CLK_ENABLE();
        __HAL_RCC_DMA1_CLK_ENABLE();
    }

    utl::result<void> validate() {
        auto res = HAL_DMA_Init(&m_handle);
        if(res != HAL_OK) return make_hal_error_code(res);
        return utl::success();
    }
public:
    template<typename T>
    void link(T& handle, uint16_t handler) {
        __HAL_LINKDMA(&handle,hdma[handler],m_handle);

        if(m_handle.Instance == DMA1_Channel1) {
            //FIXME: fixed priority
            HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
            HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
        } else {

        }
    }

    void service() {
        HAL_DMA_IRQHandler(&m_handle);
    }
};


} //namespace stm32g4::driver::dma


#endif
