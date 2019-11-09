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
    constexpr channel(DMA_Channel_TypeDef* instance, request req, direction dir,
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
    }

    utl::result<void> validate() {
        if(m_handle.Instance == DMA1_Channel1 ||
            m_handle.Instance == DMA1_Channel2 ||
            m_handle.Instance == DMA1_Channel3 ||
            m_handle.Instance == DMA1_Channel4 ||
            m_handle.Instance == DMA1_Channel5 ||
            m_handle.Instance == DMA1_Channel6
        ) {            
            __HAL_RCC_DMA1_CLK_ENABLE();
        } else if(m_handle.Instance == DMA2_Channel1 ||
            m_handle.Instance == DMA2_Channel2 ||
            m_handle.Instance == DMA2_Channel3 ||
            m_handle.Instance == DMA2_Channel4 ||
            m_handle.Instance == DMA2_Channel5 ||
            m_handle.Instance == DMA2_Channel6
        ) {
            __HAL_RCC_DMA2_CLK_ENABLE();
        }

        __HAL_RCC_DMAMUX1_CLK_ENABLE();


        auto res = HAL_DMA_Init(&m_handle);
        if(res != HAL_OK) return make_hal_error_code(res);
        return utl::success();
    }
public:
    DMA_HandleTypeDef* handle() { return &m_handle; }

    void link(void* handle) {
        m_handle.Parent = handle;

        //FIXME: all of this needs to go elsewhere!
        if(m_handle.Instance == DMA1_Channel1) {
            HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 2, 0);
            HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
        } else if(m_handle.Instance == DMA1_Channel2) {
            HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 2, 0);
            HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
        } else if(m_handle.Instance == DMA1_Channel3) {
            HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 2, 0);
            HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
        } else if(m_handle.Instance == DMA1_Channel4) {
            HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 2, 0);
            HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
        } else if(m_handle.Instance == DMA1_Channel5) {
            HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 2, 0);
            HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
        } else if(m_handle.Instance == DMA1_Channel6) {
            HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 2, 0);
            HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
        } else if(m_handle.Instance == DMA2_Channel1) {
            HAL_NVIC_SetPriority(DMA2_Channel1_IRQn, 2, 0);
            HAL_NVIC_EnableIRQ(DMA2_Channel1_IRQn);
        } else if(m_handle.Instance == DMA2_Channel2) {
            HAL_NVIC_SetPriority(DMA2_Channel2_IRQn, 2, 0);
            HAL_NVIC_EnableIRQ(DMA2_Channel2_IRQn);
        } else if(m_handle.Instance == DMA2_Channel3) {
            HAL_NVIC_SetPriority(DMA2_Channel3_IRQn, 2, 0);
            HAL_NVIC_EnableIRQ(DMA2_Channel3_IRQn);
        } else if(m_handle.Instance == DMA2_Channel4) {
            HAL_NVIC_SetPriority(DMA2_Channel4_IRQn, 2, 0);
            HAL_NVIC_EnableIRQ(DMA2_Channel4_IRQn);
        } else if(m_handle.Instance == DMA2_Channel5) {
            HAL_NVIC_SetPriority(DMA2_Channel5_IRQn, 2, 0);
            HAL_NVIC_EnableIRQ(DMA2_Channel5_IRQn);
        } else if(m_handle.Instance == DMA2_Channel6) {
            HAL_NVIC_SetPriority(DMA2_Channel6_IRQn, 2, 0);
            HAL_NVIC_EnableIRQ(DMA2_Channel6_IRQn);
        }
    }

    void service() {
        HAL_DMA_IRQHandler(&m_handle);
    }
};


} //namespace stm32g4::driver::dma


#endif
