.syntax unified
.cpu cortex-m4
.thumb

.word 0x20000400 @ STACK POINTER
.word 0x080000ed @ COMMAND ADDR
.space 0xe4

@ DEF RCC ADDRESS
.equ RCC_ADDR, 0x40023800

@ DEF OFFSETS
.equ OFFSET_RCC_AHB1ENR, 0x00000030
.equ OFFSET_RCC_APB1ENR, 0x00000040
.equ OFFSET_GPIO_OUTMODE, 0x00000000
.equ OFFSET_GPIO_OUTDATA, 0x00000014
.equ OFFSET_TIM1TO8_CTRL_REG, 0x00000000
.equ OFFSET_TIM1TO8_AUTORELOAD_REG, 0x0000002C
.equ OFFSET_TIM1TO8_PRESCALER, 0x00000028
.equ OFFSET_TIM1TO8_COUNTER, 0x00000024

@ DEF RCC BUS
.equ RCC_AHB1ENR, (OFFSET_RCC_AHB1ENR + RCC_ADDR)
.equ RCC_APB1ENR, (OFFSET_RCC_APB1ENR + RCC_ADDR)

@ DEF GPIOx ADDR
.equ GPIOB_ADDR, 0x40020400
.equ GPIOC_ADDR, 0x40020800
.equ GPIOD_ADDR, 0x40020C00

@ DEF TIMx ADDR
.equ TIM2_ADDR, 0x40000000

@ DEF GPIOx MODER
.equ GPIOB_MODER, (OFFSET_GPIO_OUTMODE + GPIOB_ADDR)
.equ GPIOC_MODER, (OFFSET_GPIO_OUTMODE + GPIOC_ADDR)
.equ GPIOD_MODER, (OFFSET_GPIO_OUTMODE + GPIOD_ADDR)

@ DEF GPIOx OUTDATA REG
.equ GPIOB_ODR, (OFFSET_GPIO_OUTDATA + GPIOB_ADDR)
.equ GPIOC_ODR, (OFFSET_GPIO_OUTDATA + GPIOC_ADDR)
.equ GPIOD_ODR, (OFFSET_GPIO_OUTDATA + GPIOD_ADDR)

@ CORE PERIPHERAL NVIC BASE ADDR
.equ NVIC_BASE_ADDR, 0xE0000000

@ VECTOR TABLE IRQ NUM = 28
.equ IRQ28, 0x000000B0

@ DEF INTERRUPT SET-ENABLE REGS
.equ ISER0, 0x0000E100
.equ ISER1, 0x0000E104
.equ ISER2, 0x0000E108
.equ ISER3, 0x0000E10C
.equ ISER4, 0x0000E110
.equ ISER5, 0x0000E114
.equ ISER6, 0x0000E118
.equ ISER7, 0x0000E11C

@ DEF INTERRUPT SET-ENABLE0 REG
.equ NVIC_ISER0, (CORE_PPB + ISER0)

@ DEF SYST CONTROL & STATUS REG
.equ CSR, 0x0000E010
.equ RVR, 0x0000E014
.equ CVR, 0x0000E018

@ DEF SYST CONTROL & STATUS RELOAD CURRENTREG
.equ SYST_CSR, (CORE_PPB + CSR)
.equ SYST_RVR, (CORE_PPB + RVR)
.equ SYST_CVR, (CORE_PPB + CVR)

@ DEF TIMx REG
.equ TIM2_CR1, (TIM2_ADDR + OFFSET_TIM1TO8_CTRL_REG)
.equ TIM2_ARR, (TIM2_ADDR + OFFSET_TIM1TO8_AUTORELOAD_REG)
.equ TIM2_PSC, (TIM2_ADDR + OFFSET_TIM1TO8_PRESCALER)
.equ TIM2_CNT, (TIM2_ADDR + OFFSET_TIM1TO8_COUNTER)

@ DEF PINS
.equ PB7_MODE, (1 << (7 * 2))
.equ PD7_MODE, (1 << (7 * 2))

@ DEF DELAY COUNT
.equ DELAY_COUNT, 1000000
.equ DELAY_COUNT_ON, 500000
.equ DELAY_COUNT_OFF, 500000

.macro LED_ON gpio, pin
    ldr r0, =\gpio
    ldr r1, [r0]
    orr r1, r1, #(1 << \pin)
    str r1, [r0]
.endm

.macro LED_OFF gpio, pin
    ldr r0, =\gpio
    ldr r1, [r0]
    bic r1, r1, #(1 << \pin)
    str r1, [r0]
.endm

.global _start
_start:
    @ Enable TIM2 Clock
    ldr r0, =RCC_APB1ENR
    ldr r1, [r0]
    orr r1, r1, #1
    str r1, [r0]

    @ Enable TIM2_CR1
    ldr r0, =TIM2_CR1
    ldr r1, [r0]
    orr r1, r1, #1 @ CR1_CEN = TIM2_CR1 + 1
    orr r1, r1, #4 @ CR1_URS = TIM2_CR1 + 4
    str r1, [r0]

    @ Enable GPIOx Clock GPIOB = 1, GPIOD = 3
    ldr r0, =RCC_AHB1ENR
    ldr r1, [r0]
    orr r1, r1, #(1 << 1)
    str r1, [r0]
    orr r1, r1, #(1 << 3)
    str r1, [r0]

    @ Set GPIOB_MODER
    ldr r0, =GPIOB_MODER
    orr r1, r1, #(PB7_MODE)
    str r1, [r0]

    @ Set GPIOD_MODER
    ldr r0, =GPIOD_MODER
    orr r1, r1, #(PD7_MODE)
    str r1, [r0]
    
loop:
    @ Set Delay Count
    ldr r2, =DELAY_COUNT

    @ LED PB7 ON, PD7 OFF
    LED_OFF GPIOD_ODR, 7
    LED_ON GPIOB_ODR, 7
    bl delay_ms

    @ LED PB7 OFF, PD7 ON
    LED_ON GPIOD_ODR, 7
    LED_OFF GPIOB_ODR, 7
    bl delay_ms

    b loop

delay_ms :
    sub r2, r2, #1
    cmp r2, #0
    bne delay_ms
    ldr r2, =DELAY_COUNT
    bx lr
    
/*.type Enable_TM1, %function
Enable_TM1 : 
    ldr r1, =
*/
