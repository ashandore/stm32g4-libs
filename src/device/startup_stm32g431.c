//*****************************************************************************
// STM32G431 Microcontroller Startup code
//*****************************************************************************
//
// Copyright(C) Luminaire Coffee, 2017
//*****************************************************************************
#include "device/system_stm32g4xx.h"
#include "sys/types.h"
#include "board.h"

#if defined (__cplusplus)

extern "C" {
    extern void __libc_init_array(void);
}
#endif

#define WEAK __attribute__ ((weak))
#define ALIAS(f) __attribute__ ((weak, alias (#f)))

//*****************************************************************************
#if defined (__cplusplus)
extern "C" {
#endif

//*****************************************************************************

extern void SystemInit(void);

     void ResetISR(void);
WEAK void NMI_Handler(void);
WEAK void HardFault_Handler(void);
WEAK void MemManage_Handler(void);
WEAK void BusFault_Handler(void);
WEAK void UsageFault_Handler(void);
WEAK void SVC_Handler(void);
WEAK void DebugMon_Handler(void);
WEAK void PendSV_Handler(void);
WEAK void SysTick_Handler(void);
WEAK void IntDefaultHandler(void);

void WWDG_IRQHandler(void) ALIAS(IntDefaultHandler);
void PVD_PVM_IRQHandler(void) ALIAS(IntDefaultHandler);
void RTC_TAMP_LSECSS_IRQHandler(void) ALIAS(IntDefaultHandler);
void RTC_WKUP_IRQHandler(void) ALIAS(IntDefaultHandler);
void FLASH_IRQHandler(void) ALIAS(IntDefaultHandler);
void RCC_IRQHandler(void) ALIAS(IntDefaultHandler);
void EXTI0_IRQHandler(void) ALIAS(IntDefaultHandler);
void EXTI1_IRQHandler(void) ALIAS(IntDefaultHandler);
void EXTI2_IRQHandler(void) ALIAS(IntDefaultHandler);
void EXTI3_IRQHandler(void) ALIAS(IntDefaultHandler);
void EXTI4_IRQHandler(void) ALIAS(IntDefaultHandler);
void DMA1_Channel1_IRQHandler(void) ALIAS(IntDefaultHandler);
void DMA1_Channel2_IRQHandler(void) ALIAS(IntDefaultHandler);
void DMA1_Channel3_IRQHandler(void) ALIAS(IntDefaultHandler);
void DMA1_Channel4_IRQHandler(void) ALIAS(IntDefaultHandler);
void DMA1_Channel5_IRQHandler(void) ALIAS(IntDefaultHandler);
void DMA1_Channel6_IRQHandler(void) ALIAS(IntDefaultHandler);
void ADC1_2_IRQHandler(void) ALIAS(IntDefaultHandler);
void USB_HP_IRQHandler(void) ALIAS(IntDefaultHandler);
void USB_LP_IRQHandler(void) ALIAS(IntDefaultHandler);
void FDCAN1_IT0_IRQHandler(void) ALIAS(IntDefaultHandler);
void FDCAN1_IT1_IRQHandler(void) ALIAS(IntDefaultHandler);
void EXTI9_5_IRQHandler(void) ALIAS(IntDefaultHandler);
void TIM1_BRK_TIM15_IRQHandler(void) ALIAS(IntDefaultHandler);
void TIM1_UP_TIM16_IRQHandler(void) ALIAS(IntDefaultHandler);
void TIM1_TRG_COM_TIM17_IRQHandler(void) ALIAS(IntDefaultHandler);
void TIM1_CC_IRQHandler(void) ALIAS(IntDefaultHandler);
void TIM2_IRQHandler(void) ALIAS(IntDefaultHandler);
void TIM3_IRQHandler(void) ALIAS(IntDefaultHandler);
void TIM4_IRQHandler(void) ALIAS(IntDefaultHandler);
void I2C1_EV_IRQHandler(void) ALIAS(IntDefaultHandler);
void I2C1_ER_IRQHandler(void) ALIAS(IntDefaultHandler);
void I2C2_EV_IRQHandler(void) ALIAS(IntDefaultHandler);
void I2C2_ER_IRQHandler(void) ALIAS(IntDefaultHandler);
void SPI1_IRQHandler(void) ALIAS(IntDefaultHandler);
void SPI2_IRQHandler(void) ALIAS(IntDefaultHandler);
void USART1_IRQHandler(void) ALIAS(IntDefaultHandler);
void USART2_IRQHandler(void) ALIAS(IntDefaultHandler);
void USART3_IRQHandler(void) ALIAS(IntDefaultHandler);
void EXTI15_10_IRQHandler(void) ALIAS(IntDefaultHandler);
void RTC_Alarm_IRQHandler(void) ALIAS(IntDefaultHandler);
void USBWakeUp_IRQHandler(void) ALIAS(IntDefaultHandler);
void TIM8_BRK_IRQHandler(void) ALIAS(IntDefaultHandler);
void TIM8_UP_IRQHandler(void) ALIAS(IntDefaultHandler);
void TIM8_TRG_COM_IRQHandler(void) ALIAS(IntDefaultHandler);
void TIM8_CC_IRQHandler(void) ALIAS(IntDefaultHandler);
void LPTIM1_IRQHandler(void) ALIAS(IntDefaultHandler);
void SPI3_IRQHandler(void) ALIAS(IntDefaultHandler);
void UART4_IRQHandler(void) ALIAS(IntDefaultHandler);
void TIM6_DAC_IRQHandler(void) ALIAS(IntDefaultHandler);
void TIM7_IRQHandler(void) ALIAS(IntDefaultHandler);
void DMA2_Channel1_IRQHandler(void) ALIAS(IntDefaultHandler);
void DMA2_Channel2_IRQHandler(void) ALIAS(IntDefaultHandler);
void DMA2_Channel3_IRQHandler(void) ALIAS(IntDefaultHandler);
void DMA2_Channel4_IRQHandler(void) ALIAS(IntDefaultHandler);
void DMA2_Channel5_IRQHandler(void) ALIAS(IntDefaultHandler);
void UCPD1_IRQHandler(void) ALIAS(IntDefaultHandler);
void COMP1_2_3_IRQHandler(void) ALIAS(IntDefaultHandler);
void COMP4_IRQHandler(void) ALIAS(IntDefaultHandler);
void CRS_IRQHandler(void) ALIAS(IntDefaultHandler);
void SAI1_IRQHandler(void) ALIAS(IntDefaultHandler);
void FPU_IRQHandler(void) ALIAS(IntDefaultHandler);
void RNG_IRQHandler(void) ALIAS(IntDefaultHandler);
void LPUART1_IRQHandler(void) ALIAS(IntDefaultHandler);
void I2C3_EV_IRQHandler(void) ALIAS(IntDefaultHandler);
void I2C3_ER_IRQHandler(void) ALIAS(IntDefaultHandler);
void DMAMUX_OVR_IRQHandler(void) ALIAS(IntDefaultHandler);
void DMA2_Channel6_IRQHandler(void) ALIAS(IntDefaultHandler);
void CORDIC_IRQHandler(void) ALIAS(IntDefaultHandler);
void FMAC_IRQHandler(void) ALIAS(IntDefaultHandler);


extern int main(void);
extern void _vStackTop(void);


#if defined (__cplusplus)
} // extern "C"
#endif

extern void (* g_pfnVectors[])(void);
__attribute__ ((section(".isr_vector")))

void (* g_pfnVectors[])(void) = {
    &_vStackTop,
    ResetISR,
    NMI_Handler,
    HardFault_Handler,
    MemManage_Handler,
    BusFault_Handler,
    UsageFault_Handler,
    0,
    0,
    0,
    0,
    SVC_Handler,
    DebugMon_Handler,
    0,
    PendSV_Handler,
    SysTick_Handler,
    WWDG_IRQHandler,
    PVD_PVM_IRQHandler,
    RTC_TAMP_LSECSS_IRQHandler,
    RTC_WKUP_IRQHandler,
    FLASH_IRQHandler,
    RCC_IRQHandler,
    EXTI0_IRQHandler,
    EXTI1_IRQHandler,
    EXTI2_IRQHandler,
    EXTI3_IRQHandler,
    EXTI4_IRQHandler,
    DMA1_Channel1_IRQHandler,
    DMA1_Channel2_IRQHandler,
    DMA1_Channel3_IRQHandler,
    DMA1_Channel4_IRQHandler,
    DMA1_Channel5_IRQHandler,
    DMA1_Channel6_IRQHandler,
    0,
    ADC1_2_IRQHandler,
    USB_HP_IRQHandler,
    USB_LP_IRQHandler,
    FDCAN1_IT0_IRQHandler,
    FDCAN1_IT1_IRQHandler,
    EXTI9_5_IRQHandler,
    TIM1_BRK_TIM15_IRQHandler,
    TIM1_UP_TIM16_IRQHandler,
    TIM1_TRG_COM_TIM17_IRQHandler,
    TIM1_CC_IRQHandler,
    TIM2_IRQHandler,
    TIM3_IRQHandler,
    TIM4_IRQHandler,
    I2C1_EV_IRQHandler,
    I2C1_ER_IRQHandler,
    I2C2_EV_IRQHandler,
    I2C2_ER_IRQHandler,
    SPI1_IRQHandler,
    SPI2_IRQHandler,
    USART1_IRQHandler,
    USART2_IRQHandler,
    USART3_IRQHandler,
    EXTI15_10_IRQHandler,
    RTC_Alarm_IRQHandler,
    USBWakeUp_IRQHandler,
    TIM8_BRK_IRQHandler,
    TIM8_UP_IRQHandler,
    TIM8_TRG_COM_IRQHandler,
    TIM8_CC_IRQHandler,
    0,
    0,
    LPTIM1_IRQHandler,
    0,
    SPI3_IRQHandler,
    UART4_IRQHandler,
    0,
    TIM6_DAC_IRQHandler,
    TIM7_IRQHandler,
    DMA2_Channel1_IRQHandler,
    DMA2_Channel2_IRQHandler,
    DMA2_Channel3_IRQHandler,
    DMA2_Channel4_IRQHandler,
    DMA2_Channel5_IRQHandler,
    0,
    0,
    UCPD1_IRQHandler,
    COMP1_2_3_IRQHandler,
    COMP4_IRQHandler,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    CRS_IRQHandler,
    SAI1_IRQHandler,
    0,
    0,
    0,
    0,
    FPU_IRQHandler,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    RNG_IRQHandler,
    LPUART1_IRQHandler,
    I2C3_EV_IRQHandler,
    I2C3_ER_IRQHandler,
    DMAMUX_OVR_IRQHandler,
    0,
    0,
    DMA2_Channel6_IRQHandler,
    0,
    0,
    CORDIC_IRQHandler,
    FMAC_IRQHandler
};




//*****************************************************************************
// Functions to carry out the initialization of data and BSS data sections. These
// are written as separate functions rather than being inlined within the
// ResetISR() function in order to cope with MCUs with multiple banks of
// memory.
//*****************************************************************************
__attribute__((section(".section_table")))
void data_init(unsigned int romstart, unsigned int start, unsigned int len) {
    unsigned int *pulDest = (unsigned int*) start;
    unsigned int *pulSrc = (unsigned int*) romstart;
    unsigned int loop;
    for (loop = 0; loop < len; loop = loop + 4)
        *pulDest++ = *pulSrc++;
}

__attribute__ ((section(".section_table")))
void bss_init(unsigned int start, unsigned int len) {
    unsigned int *pulDest = (unsigned int*) start;
    unsigned int loop;
    for (loop = 0; loop < len; loop = loop + 4)
        *pulDest++ = 0;
}

 extern unsigned int __section_table_start;
 extern unsigned int __section_table_end;
 extern unsigned int __bss_table_start;
 extern unsigned int __bss_table_end;

__attribute__ ((section(".section_table")))
void copy_sections(unsigned int range_start, unsigned int range_end) {
    unsigned int LoadAddr, ExeAddr, SectionLen;
    unsigned int *SectionTableAddr;

    // Load base address of Global Section Table
    SectionTableAddr = &__section_table_start;

    // Copy sections within this memory range.
    while (SectionTableAddr < &__section_table_end) {
        LoadAddr = *SectionTableAddr++;
        ExeAddr = *SectionTableAddr++;
        SectionLen = *SectionTableAddr++;

        //For now, only load/init data that isn't in flash.
        if(LoadAddr != ExeAddr && ExeAddr < range_end && ExeAddr >= range_start) {
            data_init(LoadAddr, ExeAddr, SectionLen);
        }
    }
}

__attribute__ ((section(".section_table")))
void init_bss(unsigned int range_start, unsigned int range_end) {
    unsigned int ExeAddr, SectionLen;
    unsigned int *SectionTableAddr;

    // Load base address of Global Section Table
    SectionTableAddr = &__bss_table_start;

    while (SectionTableAddr < &__bss_table_end) {
        ExeAddr = *SectionTableAddr++;
        SectionLen = *SectionTableAddr++;
        if(ExeAddr < range_end && ExeAddr >= range_start)
            bss_init(ExeAddr, SectionLen);
    }
}


//*****************************************************************************
// Reset entry point.
//*****************************************************************************
static uint8_t system_initialized = 0;

void
ResetISR(void) {

    //Copy data sections to RAM
    copy_sections(0x20000000, 0x28000000);
    //Initialize BSS sections
    init_bss(0x20000000, 0x28000000);

    SystemInit();

    //Relocate vector table if necessary.
    unsigned int * pSCB_VTOR = (unsigned int *) 0xE000ED08;
    if ((unsigned int *) g_pfnVectors != (unsigned int *) 0x08000000) {
        // CMSIS : SCB->VTOR = <address of vector table>
        *pSCB_VTOR = (unsigned int) g_pfnVectors;
    }

#if defined (__cplusplus)
    //
    // Call C++ library initialisation
    //
    __libc_init_array();
#endif
    system_initialized = 1;
    main();
    //
    // main() shouldn't return, but if it does, we'll just enter an infinite loop
    //
    while (1) {
        ;
    }
}

//*****************************************************************************
// Default exception handlers. Override the ones here by defining your own
// handler routines in your application code.
//*****************************************************************************
__attribute__ ((section(".after_vectors")))
void NMI_Handler(void)
{ while(1) {}
}

// __attribute__ ((section(".after_vectors")))
// void HardFault_Handler(void)
// { while(1) {}
// }

__attribute__ ((section(".after_vectors")))
void MemManage_Handler(void)
{ while(1) {}
}

__attribute__ ((section(".after_vectors")))
void BusFault_Handler(void)
{ while(1) {}
}

__attribute__ ((section(".after_vectors")))
void UsageFault_Handler(void)
{ while(1) {}
}

__attribute__ ((section(".after_vectors")))
void SVC_Handler(void)
{ while(1) {}
}

__attribute__ ((section(".after_vectors")))
void DebugMon_Handler(void)
{ while(1) {}
}

__attribute__ ((section(".after_vectors")))
void PendSV_Handler(void)
{ while(1) {}
}

__attribute__ ((section(".after_vectors")))
void SysTick_Handler(void)
{ while(1) {}
}

//*****************************************************************************
//
// Processor ends up here if an unexpected interrupt occurs or a specific
// handler is not present in the application code.
//
//*****************************************************************************
__attribute__ ((section(".after_vectors")))
void IntDefaultHandler(void)
{ while(1) {}
}

/**
 * HardFault_HandlerAsm:
 * Alternative Hard Fault handler to help debug the reason for a fault.
 * To use, edit the vector table to reference this function in the HardFault vector
 * This code is suitable for Cortex-M3 and Cortex-M0 cores
 */

// Use the 'naked' attribute so that C stacking is not used.
//static void HardFault_Handler( void ) ;

/* The fault handler implementation calls a function called
prvGetRegistersFromStack(). */
 __attribute__ ((section(".after_vectors")))
__attribute__( ( naked ) ) void HardFault_Handler(void)
{
    __asm volatile
    (
        " tst lr, #4                                                \n"
        " ite eq                                                    \n"
        " mrseq r0, msp                                             \n"
        " mrsne r0, psp                                             \n"
        " ldr r1, [r0, #24]                                         \n"
        " ldr r2, handler2_address_const                            \n"
        " bx r2                                                     \n"
        " handler2_address_const: .word prvGetRegistersFromStack    \n"
    );
}

#pragma clang optimize off
volatile uint32_t r0;
volatile uint32_t r1;
volatile uint32_t r2;
volatile uint32_t r3;
volatile uint32_t r12;
volatile uint32_t lr; /* Link register. */
volatile uint32_t pc; /* Program counter. */
volatile uint32_t psr;/* Program status register. */
volatile uint32_t sp;

typedef struct {
    uint32_t _reserved1:1;
    uint32_t VECTTBL:1;
    uint32_t _reserved0:28;
    uint32_t FORCED:1;
    uint32_t DEBUGEVT:1;
} HFSR_Reg;

typedef struct {
    struct {
        uint8_t IACCVIOL:1;
        uint8_t DACCVIOL:1;
        uint8_t _reserved0:1;
        uint8_t MUNSTKERR:1;
        uint8_t MSTKERR:1;
        uint8_t _reserved1:1;
        uint8_t MMARVALID:1;
    } MMFSR;
    struct {
        uint8_t IBUSERR:1;
        uint8_t PRECISERR:1;
        uint8_t IMPRECISERR:1;
        uint8_t UNSTKERR:1;
        uint8_t STKERR:1;
        uint8_t _reserved0:1;
        uint8_t BFARVALID:1;
    } BFSR;
    struct {
        uint16_t UNDEFINSTR:1;
        uint16_t INVSTATE:1;
        uint16_t INVPC:1;
        uint16_t NOCP:1;
        uint16_t _reserved1:4;
        uint16_t UNALIGNED:1;
        uint16_t DIVBYZERO:1;
        uint16_t _reserved0:6;
    } UFSR;
} CFSR_Reg;

typedef struct {
    uint32_t ADDRESS;
} MMAR_Reg;

typedef struct {
    uint32_t ADDRESS;
} BFAR_Reg;

volatile HFSR_Reg* HFSR;
volatile CFSR_Reg* CFSR;
volatile MMAR_Reg* MMAR;
volatile BFAR_Reg* BFAR;

#ifndef NO_FAULT_DUMP

WEAK void fault_output_init() {}

WEAK void fault_output_write(const char *string, uint32_t length) {
    (void)(string);
    (void)(length);
}

WEAK void fault_output_printf(const char *format, ...) {
    (void)(format);
}

extern unsigned int _heap_start;
extern unsigned int _heap_end;
extern caddr_t _sbrk (int);

__attribute__((used))
static void callback_unwind_frame_printer(uint32_t const* registers, void* user)
{
    (void)(user);
    uint32_t const FP = registers[0];
    uint32_t const LR = registers[1];
    fault_output_printf("0x%08x, 0x%08x", FP, LR);
}

#endif

void prvGetRegistersFromStack(uint32_t* pulFaultStackAddress)
{
    // These are volatile to try and prevent the compiler/linker optimising them
    // away as the variables never actually get used.
    // If the debugger won't show the values of the variables,
    // make them global my moving their declaration outside of this function.

    r0 = pulFaultStackAddress[ 0 ];
    r1 = pulFaultStackAddress[ 1 ];
    r2 = pulFaultStackAddress[ 2 ];
    r3 = pulFaultStackAddress[ 3 ];

    r12 = pulFaultStackAddress[ 4 ];
    lr = pulFaultStackAddress[ 5 ];
    pc = pulFaultStackAddress[ 6 ];
    psr = pulFaultStackAddress[ 7 ];

    HFSR = (HFSR_Reg*)&SCB->HFSR;
    CFSR = (CFSR_Reg*)&SCB->CFSR;
    MMAR = (MMAR_Reg*)&SCB->MMFAR;
    BFAR = (BFAR_Reg*)&SCB->BFAR;

    #ifndef NO_FAULT_DUMP

    if(system_initialized) {
        fault_output_init();
        fault_output_printf("------ HARD FAULT ------");
        fault_output_printf("pc  =\t0x%08x", pc);
        fault_output_printf("lr  =\t0x%08x", lr);
        fault_output_printf("psr =\t0x%08x", psr);
        fault_output_printf("r0  =\t0x%08x", r0);
        fault_output_printf("r1  =\t0x%08x", r1);
        fault_output_printf("r2  =\t0x%08x", r2);
        fault_output_printf("r3  =\t0x%08x", r3);
        fault_output_printf("r12 =\t0x%08x", r12);
        fault_output_printf("sp  =\t0x%08x", pulFaultStackAddress);
        fault_output_printf("");

        fault_output_printf("HFSR:");
        fault_output_printf("\t.VECTTBL  =\t%d", HFSR->VECTTBL);
        fault_output_printf("\t.FORCED   =\t%d", HFSR->FORCED);
        fault_output_printf("\t.DEBUGEVT =\t%d", HFSR->DEBUGEVT);

        fault_output_printf("MMFSR:");
        fault_output_printf("\t.IACCVIOL  =\t%d", CFSR->MMFSR.IACCVIOL);
        fault_output_printf("\t.DACCVIOL  =\t%d", CFSR->MMFSR.DACCVIOL);
        fault_output_printf("\t.MUNSTKERR =\t%d", CFSR->MMFSR.MUNSTKERR);
        fault_output_printf("\t.MSTKERR   =\t%d", CFSR->MMFSR.MSTKERR);
        fault_output_printf("\t.MMARVALID =\t%d", CFSR->MMFSR.MMARVALID);

        fault_output_printf("BFSR:");
        fault_output_printf("\t.IBUSERR     =\t%d", CFSR->BFSR.IBUSERR);
        fault_output_printf("\t.PRECISERR   =\t%d", CFSR->BFSR.PRECISERR);
        fault_output_printf("\t.IMPRECISERR =\t%d", CFSR->BFSR.IMPRECISERR);
        fault_output_printf("\t.UNSTKERR    =\t%d", CFSR->BFSR.UNSTKERR);
        fault_output_printf("\t.STKERR      =\t%d", CFSR->BFSR.STKERR);
        fault_output_printf("\t.BFARVALID   =\t%d", CFSR->BFSR.BFARVALID);

        fault_output_printf("UFSR:");
        fault_output_printf("\t.UNDEFINSTR =\t%d", CFSR->UFSR.UNDEFINSTR);
        fault_output_printf("\t.INVSTATE   =\t%d", CFSR->UFSR.INVSTATE);
        fault_output_printf("\t.INVPC      =\t%d", CFSR->UFSR.INVPC);
        fault_output_printf("\t.NOCP       =\t%d", CFSR->UFSR.NOCP);
        fault_output_printf("\t.UNALIGNED  =\t%d", CFSR->UFSR.UNALIGNED);
        fault_output_printf("\t.DIVBYZERO  =\t%d", CFSR->UFSR.DIVBYZERO);

        fault_output_printf("MMAR.ADDRESS = 0x%08x", MMAR->ADDRESS);
        fault_output_printf("BFAR.ADDRESS = 0x%08x", BFAR->ADDRESS);

        fault_output_printf("");
        fault_output_printf("Heap info:");
        fault_output_printf("\t_heap_start =\t0x%08x", &_heap_start);
        fault_output_printf("\t_heap_end   =\t0x%08x", &_heap_end);
        fault_output_printf("\tposition    =\t0x%08x", (uint32_t)_sbrk(0));
        fault_output_printf("\t%d bytes used", ((uint32_t)_sbrk(0) - (uint32_t)&_heap_start));
        fault_output_printf("\t%d bytes remaining", ((uint32_t)&_heap_end) - (uint32_t)_sbrk(0));

        fault_output_printf("");
        fault_output_printf("Stack trace:");

        // unwind_frame(&callback_unwind_frame_printer,NULL);

        fault_output_printf("");
        fault_output_printf("Stack dump:");

        char line[70];
        char *line_ptr;

        uint32_t sp = (uint32_t)pulFaultStackAddress;

        while (sp < (uint32_t)g_pfnVectors[0]) {
            line_ptr = line;
            for(uint8_t idx = 0; idx < 24; idx++) {
                if(idx > 0 && idx % 4 == 0) line_ptr += sprintf(line_ptr, " ");
                if(sp + idx < (uint32_t)g_pfnVectors[0])
                    line_ptr += sprintf(line_ptr, "%02x", *(uint8_t*)(sp+idx));
            }
            fault_output_write(line, strlen(line));
            fault_output_write("\r\n", 2);

            sp += 30;
        }
    }

    #endif

    /* When the following line is hit, the variables contain the register values. */
    for( ;; );
}

#pragma clang optimize on
