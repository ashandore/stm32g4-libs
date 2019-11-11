
#ifndef STM32G4_LIBS_USB_HH_
#define STM32G4_LIBS_USB_HH_

#include "usb/usb_device.h"
#include "usb/usbd_desc.h"
#include "usbd_hid.h"
#include <utl/interface/hal/driver.hh>
#include <utl/hal/usb.hh>

namespace stm32g4::driver::usb::device {

class hid : public utl::interface::hal::driver {
public:

private:
    PCD_HandleTypeDef       m_usb_handle; //hpcd_USB_FS;
    USBD_HandleTypeDef      m_device_handle; //hUsbDeviceFS;

protected:

    constexpr hid() : m_usb_handle{}, m_device_handle{}
    {}

    utl::result<void> validate()
    {
        USBD_Clock_Config();

        m_device_handle.pData = &m_usb_handle;

        //FIXME: HID_Desc is a global and it should be somewhere else -
        //a data structure + dependency of this type (hid).
        auto res = USBD_Init(&m_device_handle, &HID_Desc, DEVICE_FS);
        if(res != USBD_OK) return stm32g4::hal_error::ERROR;
        
        //NB: USBD_HID is a global but it's in ST's middleware rather than
        //generated code. So I'm going to leave it that way.
        res = USBD_RegisterClass(&m_device_handle, &USBD_HID);
        if(res != USBD_OK) return stm32g4::hal_error::ERROR;

        res = USBD_Start(&m_device_handle);
        if(res != USBD_OK) return stm32g4::hal_error::ERROR;

        return utl::success();
    }

public:

    // utl::result<detail::connection> connection() {

    // }

    bool dev_remote_wakeup() {
        return reinterpret_cast<USBD_HandleTypeDef*>(m_usb_handle.pData)->dev_remote_wakeup == 1;
    }

    utl::hal::usbd::state state() {
        auto s = reinterpret_cast<USBD_HandleTypeDef*>(m_usb_handle.pData)->dev_state;

        switch(s) {
            case USBD_STATE_DEFAULT:
                return utl::hal::usbd::state::DEFAULT;
            case USBD_STATE_ADDRESSED:
                return utl::hal::usbd::state::ADDRESSED;
            case USBD_STATE_CONFIGURED:
                return utl::hal::usbd::state::CONFIGURED;
            case USBD_STATE_SUSPENDED:
                return utl::hal::usbd::state::SUSPENDED;
            default:
                return utl::hal::usbd::state::UNKNOWN;
        }
    }

    void service() {
        HAL_PCD_IRQHandler(&m_usb_handle);
    }

};

} //namespace stm32g4::driver::usb::device





#endif //STM32G4_LIBS_USB_HH_
