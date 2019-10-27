#ifndef STM32G4_LIBS_DRIVER_DMA_HH_
#define STM32G4_LIBS_DRIVER_DMA_HH_

#include "stm32g4xx_hal_dma.h"
#include "utl.hh"

namespace stm32g4::driver {

class dma {
    DMA_HandleTypeDef   m_handle;
protected:
    // dma(DMA_Channel_TypeDef* instance, uint32_t request, uint32_t direction,
    //     uint32_t periph_inc, uint32_t mem_inc, uint32_t periph_data_align,
    //     uint32_t mem_data_align, uint32_t mode, uint32_t priority) 
    //     : m_handle{} 
    // {
    //     // m_handle.Instance = instance;
    //     // m_handle.Init.Request = request;
    //     // m_handle.Init.Direction = direction;
    //     // m_handle.Init.PeriphInc = periph_inc;
    //     // m_handle.Init.MemInc = mem_inc;
    //     // m_handle.Init.PeriphDataAlignment = periph_data_align;
    //     // m_handle.Init.MemDataAlignment = mem_data_align;
    //     // m_handle.Init.Mode = mode;
    //     // m_handle.Init.Priority = priority;
    // }
    template <typename... Args>
    dma(Args... args) : m_handle{} { utl::maybe_unused(args...); }

    utl::result<void> validate() {
        // auto res = HAL_DMA_Init(&m_handle);
        // if(res != HAL_OK) return stm32g4::make_hal_error_code(res);
        // return utl::success();
    }
public:
    template<typename T>
    void link(T& handle, uint16_t handler) {
        utl::maybe_unused(handle, handler);
        // __HAL_LINKDMA(&handle,hdma[handler],m_handle);

        // if(m_handle.Instance == DMA1_Channel1) {
        //      __HAL_RCC_DMAMUX1_CLK_ENABLE();
        //     __HAL_RCC_DMA1_CLK_ENABLE();
        //     //FIXME: fixed priority
        //     HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
        //     HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
        // } else {

        // }
    }

    void service() {
        // HAL_DMA_IRQHandler(&m_handle);
    }
};

} //namespace stm32g4


#endif
