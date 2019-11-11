#ifndef STM32G4_DRIVER_HAL_ERROR_HH_
#define STM32G4_DRIVER_HAL_ERROR_HH_

#include "stm32g4xx_hal_def.h"
#include "utl/error.hh"

namespace stm32g4 {

using namespace utl::literals;

enum class hal_error {
    OK = HAL_OK,
    ERROR = HAL_ERROR,
    BUSY = HAL_BUSY,
    TIMEOUT = HAL_TIMEOUT,
    UNKNOWN
};

struct hal_error_category : public utl::error_category {
    const utl::string_view message(int32_t value) const final {
        auto code = static_cast<hal_error>(value);
        switch(code) {
            case hal_error::OK:
                return "hal_error::OK"_sv;
            case hal_error::ERROR:
                return "hal_error::ERROR"_sv;
            case hal_error::BUSY:
                return "hal_error::BUSY"_sv;
            case hal_error::TIMEOUT:
                return "hal_error::TIMEOUT"_sv;
            default:
                return "Unknown hal_error"_sv;
        }
    }

    const utl::string_view name() const final {
        return "uart"_sv;
    }    
};

inline static const hal_error_category _hal_error_category{};



#if 0
enum class dcmi_error {
    OVF
};

struct hal_dcmi_error_category : public utl::error_category {
    const utl::string_view message(int32_t value) const final {
        auto code = static_cast<dcmi_error>(value);
        switch(code) {
            case dcmi_error::OVF:
                return "dcmi_error::OVF"_sv;
            default:
                return "Unknown dcmi_error"_sv;
        }
    }

    const utl::string_view name() const final {
        return "dcmi"_sv;
    }    
};

inline static const hal_dcmi_error_category _hal_dcmi_error_category{};
////////////////// adc_error

enum class adc_error {
    NONE,
    INTERNAL,
    OVR,
    DMA,
    JQOVF,
    INVALID_CALLBACK
};

struct hal_adc_error_category : public utl::error_category {
    const utl::string_view message(int32_t value) const final {
        auto code = static_cast<adc_error>(value);
        switch(code) {
            case adc_error::NONE:
                return "adc_error::NONE"_sv;
            default:
                return "Unknown adc_error"_sv;
        }
    }

    const utl::string_view name() const final {
        return "adc"_sv;
    }    
};

inline static const hal_adc_error_category _hal_adc_error_category{};
////////////////// comp_error

enum class comp_error {
    NONE,
    INVALID_CALLBACK
};

struct hal_comp_error_category : public utl::error_category {
    const utl::string_view message(int32_t value) const final {
        auto code = static_cast<comp_error>(value);
        switch(code) {
            case comp_error::NONE:
                return "comp_error::NONE"_sv;
            default:
                return "Unknown comp_error"_sv;
        }
    }

    const utl::string_view name() const final {
        return "comp"_sv;
    }    
};

inline static const hal_comp_error_category _hal_comp_error_category{};
////////////////// cordic_error

enum class cordic_error {
    NONE,
    PARAM,
    NOT_READY,
    TIMEOUT,
    DMA,
    INVALID_CALLBACK
};

struct hal_cordic_error_category : public utl::error_category {
    const utl::string_view message(int32_t value) const final {
        auto code = static_cast<cordic_error>(value);
        switch(code) {
            case cordic_error::NONE:
                return "cordic_error::NONE"_sv;
            default:
                return "Unknown cordic_error"_sv;
        }
    }

    const utl::string_view name() const final {
        return "cordic"_sv;
    }    
};

inline static const hal_cordic_error_category _hal_cordic_error_category{};
////////////////// cryp_error

enum class cryp_error {
    NONE,
    WRITE,
    READ,
    DMA,
    BUSY,
    TIMEOUT,
    NOT_SUPPORTED,
    AUTH_TAG_SEQUENCE,
    INVALID_CALLBACK
};

struct hal_cryp_error_category : public utl::error_category {
    const utl::string_view message(int32_t value) const final {
        auto code = static_cast<cryp_error>(value);
        switch(code) {
            case cryp_error::NONE:
                return "cryp_error::NONE"_sv;
            default:
                return "Unknown cryp_error"_sv;
        }
    }

    const utl::string_view name() const final {
        return "cryp"_sv;
    }    
};

inline static const hal_cryp_error_category _hal_cryp_error_category{};
////////////////// dac_error

enum class dac_error {
    NONE,
    DMAUNDERRUNCH1,
    DMAUNDERRUNCH2,
    DMA,
    TIMEOUT,
    INVALID_CALLBACK
};

struct hal_dac_error_category : public utl::error_category {
    const utl::string_view message(int32_t value) const final {
        auto code = static_cast<dac_error>(value);
        switch(code) {
            case dac_error::NONE:
                return "dac_error::NONE"_sv;
            default:
                return "Unknown dac_error"_sv;
        }
    }

    const utl::string_view name() const final {
        return "dac"_sv;
    }    
};

inline static const hal_dac_error_category _hal_dac_error_category{};
////////////////// dma_error

enum class dma_error {
    NONE,
    TE,
    NO_XFER,
    TIMEOUT,
    NOT_SUPPORTED,
    SYNC,
    REQGEN
};

struct hal_dma_error_category : public utl::error_category {
    const utl::string_view message(int32_t value) const final {
        auto code = static_cast<dma_error>(value);
        switch(code) {
            case dma_error::NONE:
                return "dma_error::NONE"_sv;
            default:
                return "Unknown dma_error"_sv;
        }
    }

    const utl::string_view name() const final {
        return "dma"_sv;
    }    
};

inline static const hal_dma_error_category _hal_dma_error_category{};
////////////////// fdcan_error

enum class fdcan_error {
    NONE,
    TIMEOUT,
    NOT_INITIALIZED,
    NOT_READY,
    NOT_STARTED,
    NOT_SUPPORTED,
    PARAM,
    PENDING,
    RAM_ACCESS,
    FIFO_EMPTY,
    FIFO_FULL,
    LOG_OVERFLOW,
    RAM_WDG,
    PROTOCOL_ARBT,
    PROTOCOL_DATA,
    RESERVED_AREA,
    INVALID_CALLBACK
};

struct hal_fdcan_error_category : public utl::error_category {
    const utl::string_view message(int32_t value) const final {
        auto code = static_cast<fdcan_error>(value);
        switch(code) {
            case fdcan_error::NONE:
                return "fdcan_error::NONE"_sv;
            default:
                return "Unknown fdcan_error"_sv;
        }
    }

    const utl::string_view name() const final {
        return "fdcan"_sv;
    }    
};

inline static const hal_fdcan_error_category _hal_fdcan_error_category{};
////////////////// fmac_error

enum class fmac_error {
    NONE,
    SAT,
    UNFL,
    OVFL,
    DMA,
    RESET,
    PARAM,
    INVALID_CALLBACK,
    TIMEOUT
};

struct hal_fmac_error_category : public utl::error_category {
    const utl::string_view message(int32_t value) const final {
        auto code = static_cast<fmac_error>(value);
        switch(code) {
            case fmac_error::NONE:
                return "fmac_error::NONE"_sv;
            default:
                return "Unknown fmac_error"_sv;
        }
    }

    const utl::string_view name() const final {
        return "fmac"_sv;
    }    
};

inline static const hal_fmac_error_category _hal_fmac_error_category{};
////////////////// i2c_error

enum class i2c_error {
    NONE,
    BERR,
    ARLO,
    AF,
    OVR,
    DMA,
    TIMEOUT,
    SIZE,
    DMA_PARAM,
    INVALID_CALLBACK,
    INVALID_PARAM
};

struct hal_i2c_error_category : public utl::error_category {
    const utl::string_view message(int32_t value) const final {
        auto code = static_cast<i2c_error>(value);
        switch(code) {
            case i2c_error::NONE:
                return "i2c_error::NONE"_sv;
            default:
                return "Unknown i2c_error"_sv;
        }
    }

    const utl::string_view name() const final {
        return "i2c"_sv;
    }    
};

inline static const hal_i2c_error_category _hal_i2c_error_category{};
////////////////// irda_error

enum class irda_error {
    NONE,
    PE,
    NE,
    FE,
    ORE,
    DMA,
    BUSY,
    INVALID_CALLBACK
};

struct hal_irda_error_category : public utl::error_category {
    const utl::string_view message(int32_t value) const final {
        auto code = static_cast<irda_error>(value);
        switch(code) {
            case irda_error::NONE:
                return "irda_error::NONE"_sv;
            default:
                return "Unknown irda_error"_sv;
        }
    }

    const utl::string_view name() const final {
        return "irda"_sv;
    }    
};

inline static const hal_irda_error_category _hal_irda_error_category{};
////////////////// pcd_error

enum class pcd_error {
    INVALID_CALLBACK
};

struct hal_pcd_error_category : public utl::error_category {
    const utl::string_view message(int32_t value) const final {
        auto code = static_cast<pcd_error>(value);
        switch(code) {
            case pcd_error::INVALID_CALLBACK:
                return "pcd_error::INVALID_CALLBACK"_sv;
            default:
                return "Unknown pcd_error"_sv;
        }
    }

    const utl::string_view name() const final {
        return "pcd"_sv;
    }    
};

inline static const hal_pcd_error_category _hal_pcd_error_category{};
////////////////// rng_error

enum class rng_error {
    NONE,
    INVALID_CALLBACK,
    TIMEOUT
};

struct hal_rng_error_category : public utl::error_category {
    const utl::string_view message(int32_t value) const final {
        auto code = static_cast<rng_error>(value);
        switch(code) {
            case rng_error::NONE:
                return "rng_error::NONE"_sv;
            default:
                return "Unknown rng_error"_sv;
        }
    }

    const utl::string_view name() const final {
        return "rng"_sv;
    }    
};

inline static const hal_rng_error_category _hal_rng_error_category{};
////////////////// uart_error

enum class uart_error {
    NONE,
    PE,
    NE,
    FE,
    ORE,
    DMA,
    INVALID_CALLBACK
};

struct hal_uart_error_category : public utl::error_category {
    const utl::string_view message(int32_t value) const final {
        auto code = static_cast<uart_error>(value);
        switch(code) {
            case uart_error::NONE:
                return "uart_error::NONE"_sv;
            case uart_error::PE:
                return "uart_error::PE"_sv;
            case uart_error::NE:
                return "uart_error::NE"_sv;
            case uart_error::FE:
                return "uart_error::FE"_sv;
            case uart_error::ORE:
                return "uart_error::ORE"_sv;
            case uart_error::DMA:
                return "uart_error::DMA"_sv;
            case uart_error::INVALID_CALLBACK:
                return "uart_error::INVALID_CALLBACK"_sv;
            default:
                return "Unknown uart_error"_sv;
        }
    }

    const utl::string_view name() const final {
        return "uart"_sv;
    }    
};

inline static const hal_uart_error_category _hal_uart_error_category{};

#endif //0

} //namespace stm32g4


template <>
struct utl::is_error_code_enum<stm32g4::hal_error> : std::true_type {};

template <>
constexpr utl::error_code utl::make_error_code<stm32g4::hal_error>(stm32g4::hal_error code) {
    return {static_cast<int32_t>(code), &stm32g4::_hal_error_category};
}


namespace stm32g4 {

constexpr utl::error_code make_hal_error_code(HAL_StatusTypeDef code) {
    switch(code) {
        case HAL_OK:
            return utl::make_error_code(hal_error::OK);
        case HAL_ERROR:
            return utl::make_error_code(hal_error::ERROR);
        case HAL_BUSY:
            return utl::make_error_code(hal_error::BUSY);
        case HAL_TIMEOUT:
            return utl::make_error_code(hal_error::TIMEOUT);
        default:
            return utl::make_error_code(hal_error::UNKNOWN);
    }
}

} //namespace stm32g4

#if 0

template<>
constexpr utl::error_code utl::make_error_code<stm32g4::dcmi_error>(stm32g4::dcmi_error code) {
    return {static_cast<int32_t>(code), &stm32g4::_hal_dcmi_error_category};
}

template <>
struct utl::is_error_code_enum<stm32g4::dcmi_error> : std::true_type {};



template<>
constexpr utl::error_code utl::make_error_code<stm32g4::adc_error>(stm32g4::adc_error code) {
    return {static_cast<int32_t>(code), &stm32g4::_hal_adc_error_category};
}

template <>
struct utl::is_error_code_enum<stm32g4::adc_error> : std::true_type {};



template<>
constexpr utl::error_code utl::make_error_code<stm32g4::comp_error>(stm32g4::comp_error code) {
    return {static_cast<int32_t>(code), &stm32g4::_hal_comp_error_category};
}

template <>
struct utl::is_error_code_enum<stm32g4::comp_error> : std::true_type {};



template<>
constexpr utl::error_code utl::make_error_code<stm32g4::cordic_error>(stm32g4::cordic_error code) {
    return {static_cast<int32_t>(code), &stm32g4::_hal_cordic_error_category};
}

template <>
struct utl::is_error_code_enum<stm32g4::cordic_error> : std::true_type {};



template<>
constexpr utl::error_code utl::make_error_code<stm32g4::cryp_error>(stm32g4::cryp_error code) {
    return {static_cast<int32_t>(code), &stm32g4::_hal_cryp_error_category};
}

template <>
struct utl::is_error_code_enum<stm32g4::cryp_error> : std::true_type {};



template<>
constexpr utl::error_code utl::make_error_code<stm32g4::dac_error>(stm32g4::dac_error code) {
    return {static_cast<int32_t>(code), &stm32g4::_hal_dac_error_category};
}

template <>
struct utl::is_error_code_enum<stm32g4::dac_error> : std::true_type {};



template<>
constexpr utl::error_code utl::make_error_code<stm32g4::dma_error>(stm32g4::dma_error code) {
    return {static_cast<int32_t>(code), &stm32g4::_hal_dma_error_category};
}

template <>
struct utl::is_error_code_enum<stm32g4::dma_error> : std::true_type {};



template<>
constexpr utl::error_code utl::make_error_code<stm32g4::fdcan_error>(stm32g4::fdcan_error code) {
    return {static_cast<int32_t>(code), &stm32g4::_hal_fdcan_error_category};
}

template <>
struct utl::is_error_code_enum<stm32g4::fdcan_error> : std::true_type {};



template<>
constexpr utl::error_code utl::make_error_code<stm32g4::fmac_error>(stm32g4::fmac_error code) {
    return {static_cast<int32_t>(code), &stm32g4::_hal_fmac_error_category};
}

template <>
struct utl::is_error_code_enum<stm32g4::fmac_error> : std::true_type {};



template<>
constexpr utl::error_code utl::make_error_code<stm32g4::i2c_error>(stm32g4::i2c_error code) {
    return {static_cast<int32_t>(code), &stm32g4::_hal_i2c_error_category};
}

template <>
struct utl::is_error_code_enum<stm32g4::i2c_error> : std::true_type {};



template<>
constexpr utl::error_code utl::make_error_code<stm32g4::irda_error>(stm32g4::irda_error code) {
    return {static_cast<int32_t>(code), &stm32g4::_hal_irda_error_category};
}

template <>
struct utl::is_error_code_enum<stm32g4::irda_error> : std::true_type {};



template<>
constexpr utl::error_code utl::make_error_code<stm32g4::pcd_error>(stm32g4::pcd_error code) {
    return {static_cast<int32_t>(code), &stm32g4::_hal_pcd_error_category};
}

template <>
struct utl::is_error_code_enum<stm32g4::pcd_error> : std::true_type {};



template<>
constexpr utl::error_code utl::make_error_code<stm32g4::rng_error>(stm32g4::rng_error code) {
    return {static_cast<int32_t>(code), &stm32g4::_hal_rng_error_category};
}

template <>
struct utl::is_error_code_enum<stm32g4::rng_error> : std::true_type {};



template<>
constexpr utl::error_code utl::make_error_code<stm32g4::uart_error>(stm32g4::uart_error code) {
    return {static_cast<int32_t>(code), &stm32g4::_hal_uart_error_category};
}

template <>
struct utl::is_error_code_enum<stm32g4::uart_error> : std::true_type {};

#endif //0

#endif //STM32G4_DRIVER_HAL_ERROR_HH_