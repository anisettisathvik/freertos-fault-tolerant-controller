/* ---------------------------------------------------------------------------
 * Cortex-M3 startup for the ARM MPS2-AN385.
 *
 * No HAL, no vendor library. The vector table, the reset handler and the C
 * runtime initialisation are written out because that is the layer the job
 * description is asking about, and a HAL hides exactly this.
 *
 * On reset the core reads two words from address 0 before executing anything:
 *   0x00  initial main stack pointer
 *   0x04  reset vector
 * Everything else follows from there.
 * ------------------------------------------------------------------------- */

#include <stdint.h>

extern uint32_t _estack, _sidata, _sdata, _edata, _sbss, _ebss;
extern int main(void);

/* FreeRTOS supplies these three; they must appear in the vector table or the
 * scheduler never runs. Getting this wrong produces a board that boots, sits
 * in the idle task and never switches — a classic first-port symptom. */
void SVC_Handler(void)     __attribute__((weak));
void PendSV_Handler(void)  __attribute__((weak));
void SysTick_Handler(void) __attribute__((weak));
void UART0_Handler(void)   __attribute__((weak));

void Reset_Handler(void);
static void Default_Handler(void);
void HardFault_Handler(void) __attribute__((weak, alias("Default_Handler_Fault")));

void Default_Handler_Fault(void);

__attribute__((section(".isr_vector"), used))
void (* const g_vectors[])(void) = {
    (void (*)(void))&_estack,   /*  0  initial stack pointer */
    Reset_Handler,              /*  1  reset                 */
    Default_Handler,            /*  2  NMI                   */
    Default_Handler_Fault,      /*  3  HardFault             */
    Default_Handler_Fault,      /*  4  MemManage             */
    Default_Handler_Fault,      /*  5  BusFault              */
    Default_Handler_Fault,      /*  6  UsageFault            */
    0, 0, 0, 0,                 /*  7-10 reserved            */
    SVC_Handler,                /* 11  SVCall   -> FreeRTOS  */
    Default_Handler,            /* 12  DebugMon              */
    0,                          /* 13  reserved              */
    PendSV_Handler,             /* 14  PendSV   -> FreeRTOS  */
    SysTick_Handler,            /* 15  SysTick  -> FreeRTOS  */
    /* external interrupts */
    UART0_Handler,              /* 16  IRQ0  UART0 RX        */
};

void Reset_Handler(void)
{
    uint32_t *src, *dst;

    /* .data lives in flash but must run from RAM: copy it across. Skipping
     * this leaves every initialised global reading as garbage. */
    src = &_sidata;
    for (dst = &_sdata; dst < &_edata; ) *dst++ = *src++;

    /* .bss must be zeroed. C guarantees zero-initialised statics; nothing
     * else does it for you on bare metal. */
    for (dst = &_sbss; dst < &_ebss; ) *dst++ = 0u;

    (void)main();
    for (;;) { }
}

static void Default_Handler(void)
{
    for (;;) { }
}

void Default_Handler_Fault(void)
{
    /* A fault handler that spins is standard, but on a safety controller the
     * output must be cut first. The pin is driven low here directly rather
     * than through any layer that might itself be the thing that faulted. */
    volatile uint32_t *gpio_out = (volatile uint32_t *)0x40010004u;
    *gpio_out = 0u;
    for (;;) { }
}

/* Weak defaults so the image links before FreeRTOS is added. Once the kernel
 * is linked in, its strong definitions take over. */
void SVC_Handler(void)     { for (;;) { } }
void PendSV_Handler(void)  { for (;;) { } }
void SysTick_Handler(void) { }
void UART0_Handler(void)   { }
