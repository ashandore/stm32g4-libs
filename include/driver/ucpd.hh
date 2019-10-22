#ifndef STM32G4_LIBS_DRIVER_UCPD_HH_
#define STM32G4_LIBS_DRIVER_UCPD_HH_

#include "stm32g4xx_ll_ucpd.h"
#include "stm32g4xx_ll_pwr.h"
#include <algorithm.hh>

namespace stm32g4::driver {

class ucpd {
public:
    ucpd() {
        __HAL_RCC_UCPD1_CLK_ENABLE();
        LL_UCPD_InitTypeDef config;
        LL_UCPD_StructInit(&config);
        LL_UCPD_Init(UCPD1, &config);
        LL_UCPD_Enable(UCPD1);
        LL_UCPD_SetSNKRole(UCPD1);
        LL_UCPD_SetccEnable(UCPD1, LL_UCPD_CCENABLE_CC1CC2);
        LL_UCPD_TypeCDetectionCC1Enable(UCPD1);
        LL_UCPD_TypeCDetectionCC2Enable(UCPD1);
        LL_PWR_DisableUSBDeadBattery();
    }

    uint32_t get_current_advertisement_ma() {
        uint8_t cc1 = LL_UCPD_GetTypeCVstateCC1(UCPD1) >> UCPD_SR_TYPEC_VSTATE_CC1_Pos;
        uint8_t cc2 = LL_UCPD_GetTypeCVstateCC2(UCPD1) >> UCPD_SR_TYPEC_VSTATE_CC2_Pos;

        switch(utl::max(cc1,cc2)) {
            case 0b00:
                return 0u;
            case 0b01:
                return 500u;
            case 0b10:
                return 1500u;
            case 0b11:
                return 3000u;
            default:
                return 0u;
        }
    }
};


} //namespace stm32g4::driver


#endif //STM32G4_LIBS_DRIVER_UCPD_HH_