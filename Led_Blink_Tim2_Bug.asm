.syntax unified
.cpu cortex-m4
.thumb

@ DEF RCC ADDRESS
.equ RCC_ADDR, 0x40023800

@ DEF OFFSETS
.equ OFFSET_RCC_AHB1ENR, 0x00000030
.equ OFFSET_RCC_APB1ENR, 0x00000040
.equ OFFSET_RCC_APB1RSTR, 0x00000020
.equ OFFSET_GPIO_OUTMODE, 0x00000000
.equ OFFSET_GPIO_OUTDATA, 0x00000014
.equ OFFSET_TIM2TO5_CTRL_REG, 0x00000000
.equ OFFSET_TIM2TO5_AUTORELOAD_REG, 0x0000002C
.equ OFFSET_TIM2TO5_DMA_INTERRUPT_REG, 0x0000000C
.equ OFFSET_TIM2TO5_PRESCALER, 0x00000028
.equ OFFSET_TIM2TO5_COUNTER, 0x00000024
.equ OFFSET_TIM2TO5_CAPTURE_COMPARE_MODE_REG1, 0x00000018
.equ OFFSET_TIM2TO5_CAPTURE_COMPARE_REG1, 0x00000034
.equ OFFSET_TIM2TO5_STATUS_REG , 0x00000010
.equ OFFSET_TIM2TO5_EVENTGERNERATION_REG, 0x00000014
.equ OFFSET_WWDG_CTRL_REG, 0x0000007F

@ DEF RCC BUS
.equ RCC_AHB1ENR, (RCC_ADDR + OFFSET_RCC_AHB1ENR)
.equ RCC_APB1ENR, (RCC_ADDR + OFFSET_RCC_APB1ENR)
.equ RCC_APB1RSTR, (RCC_ADDR + OFFSET_RCC_APB1RSTR)

@ DEF GPIOx ADDR
.equ GPIOB_ADDR, 0x40020400
.equ GPIOC_ADDR, 0x40020800
.equ GPIOD_ADDR, 0x40020C00

@ DEF TIMx ADDR
.equ TIM2_ADDR, 0x40000000

@ DEF WWDG ADDR
.equ WWDG_BASE_ADDR, 0x40002C00

@ DEF GPIOx MODER
.equ GPIOB_MODER, (OFFSET_GPIO_OUTMODE + GPIOB_ADDR)
.equ GPIOC_MODER, (OFFSET_GPIO_OUTMODE + GPIOC_ADDR)
.equ GPIOD_MODER, (OFFSET_GPIO_OUTMODE + GPIOD_ADDR)

@ DEF GPIOx OUTDATA REG
.equ GPIOB_ODR, (OFFSET_GPIO_OUTDATA + GPIOB_ADDR)
.equ GPIOC_ODR, (OFFSET_GPIO_OUTDATA + GPIOC_ADDR)
.equ GPIOD_ODR, (OFFSET_GPIO_OUTDATA + GPIOD_ADDR)

@ DEF GPIOx PIN
.equ PB7_MODE, (1 << (7 * 2))
.equ PD7_MODE, (1 << (7 * 2))

@ WINDOWS WATCHDOG CTRL REG 
.equ WWDG_CR, (WWDG_BASE_ADDR + OFFSET_WWDG_CTRL_REG)

@ CORE PERIPHERAL NVIC BASE ADDR
.equ NVIC_BASE_ADDR, 0xE0000000

@ VECTOR TABLE IRQ NUM TIM2 = 28
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

@ DEF INTERRUPT PRIORITY REGS
.equ IPR7, 0x0000E41C @ IPR7의 0번째 = 28(TIM2_IRQ)

@ DEF INTERRUPT PENDING REGS
.equ ICPR0, 0x0000E280 @ IPR7의 0번째 = 28(TIM2_IRQ)

@ DEF INTERRUPT SET-ENABLE0 REG
.equ NVIC_ISER0, (NVIC_BASE_ADDR + ISER0)

@ DEF INTERRUPT PRIORITY7 REG
.equ NVIC_IPR7, (NVIC_BASE_ADDR + IPR7)

@ DEF INTERRUPT CLEAR-PENDING REG
.equ NVIC_ICPR0, (NVIC_BASE_ADDR + ICPR0)

@ DEF SYST CONTROL & STATUS REG
.equ CSR, 0x0000E010
.equ RVR, 0x0000E014
.equ CVR, 0x0000E018

@ DEF SYST CONTROL & STATUS RELOAD CURRENTREG
.equ SYST_CSR, (NVIC_BASE_ADDR + CSR)
.equ SYST_RVR, (NVIC_BASE_ADDR + RVR)
.equ SYST_CVR, (NVIC_BASE_ADDR + CVR)

@ DEF TIMx REG
.equ TIM2_CR1, (TIM2_ADDR + OFFSET_TIM2TO5_CTRL_REG)
.equ TIM2_ARR, (TIM2_ADDR + OFFSET_TIM2TO5_AUTORELOAD_REG)
.equ TIM2_PSC, (TIM2_ADDR + OFFSET_TIM2TO5_PRESCALER)
.equ TIM2_CNT, (TIM2_ADDR + OFFSET_TIM2TO5_COUNTER)
.equ TIM2_DIER, (TIM2_ADDR + OFFSET_TIM2TO5_DMA_INTERRUPT_REG)
.equ TIM2_CCR1, (TIM2_ADDR + OFFSET_TIM2TO5_CAPTURECOMPARE_REG1)
.equ TIM2_SR, (TIM2_ADDR + OFFSET_TIM2TO5_STATUS_REG)
.equ TIM2_EGR, (TIM2_ADDR + OFFSET_TIM2TO5_EVENTGERNERATION_REG)
.equ TimerValue, 225000
.equ TIM2_IRQ, 28
.equ PRESCALER, 7999

.type Default_Handler, %function
.global Default_Handler
Default_Handler:
	bkpt
	b.n Default_Handler
    @bx lr
    @b .

.macro defisr name
    @.global \name
    .weak \name
    .thumb_set \name, Default_Handler
    @.word \name
.endm

.global VectorTable
.section .VectorTable, "a"
.type VectorTable, %object

VectorTable:
.word _StackEnd
.word Reset_Handler
defisr NMI_Handler
defisr HardFault_Handler
defisr MemManage_Handler
defisr BusFault_Handler
defisr UsageFault_Handler
.word 0
.word 0
.word 0
.word 0
defisr SVC_Handler
defisr DebugMon_Handler
.word 0
defisr PendSV_Handler
defisr SysTick_Handler
defisr WWDG_IRQHandler
defisr PVD_IRQHandler
defisr TAMP_STAMP_IRQHandler
defisr RTC_WKUP_IRQHandler
defisr FLASH_IRQHandler
defisr RCC_IRQHandler
defisr EXTI0_IRQHandler
defisr EXTI1_IRQHandler
defisr EXTI2_IRQHandler
defisr EXTI3_IRQHandler
defisr EXTI4_IRQHandler
defisr DMA1_Stream0_IRQHandler
defisr DMA1_Stream1_IRQHandler
defisr DMA1_Stream2_IRQHandler
defisr DMA1_Stream3_IRQHandler
defisr DMA1_Stream4_IRQHandler
defisr DMA1_Stream5_IRQHandler
defisr DMA1_Stream6_IRQHandler
defisr ADC_IRQHandler
defisr CAN1_TX_IRQHandler
defisr CAN1_RX0_IRQHandler
defisr CAN1_RX1_IRQHandler
defisr CAN1_SCE_IRQHandler
defisr EXTI9_5_IRQHandler
defisr TIM1_BRK_TIM9_IRQHandler
defisr TIM1_UP_TIM10_IRQHandler
defisr TIM1_TRG_COM_TIM11_IRQHandler
defisr TIM1_CC_IRQHandler
.word TIM2_IRQHandler  @ TIM2_IRQHandler -> TIM2_IRQ_Handle
defisr TIM3_IRQHandler
defisr TIM4_IRQHandler
defisr I2C1_EV_IRQHandler
defisr I2C1_ER_IRQHandler
defisr I2C2_EV_IRQHandler
defisr I2C2_ER_IRQHandler
defisr SPI1_IRQHandler
defisr SPI2_IRQHandler
defisr USART1_IRQHandler
defisr USART2_IRQHandler
defisr USART3_IRQHandler
defisr EXTI15_10_IRQHandler
defisr RTC_Alarm_IRQHandler
defisr OTG_FS_WKUP_IRQHandler
defisr TIM8_BRK_TIM12_IRQHandler
defisr TIM8_UP_TIM13_IRQHandler
defisr TIM8_TRG_COM_TIM14_IRQHandler
defisr TIM8_CC_IRQHandler
defisr DMA1_Stream7_IRQHandler
defisr FSMC_IRQHandler
defisr SDIO_IRQHandler
defisr TIM5_IRQHandler
defisr SPI3_IRQHandler
defisr UART4_IRQHandler
defisr UART5_IRQHandler
defisr TIM6_DAC_IRQHandler
defisr TIM7_IRQHandler
defisr DMA2_Stream0_IRQHandler
defisr DMA2_Stream1_IRQHandler
defisr DMA2_Stream2_IRQHandler
defisr DMA2_Stream3_IRQHandler
defisr DMA2_Stream4_IRQHandler
defisr ETH_IRQHandler
defisr ETH_WKUP_IRQHandler
defisr CAN2_TX_IRQHandler
defisr CAN2_RX0_IRQHandler
defisr CAN2_RX1_IRQHandler
defisr CAN2_SCE_IRQHandler
defisr OTG_FS_IRQHandler
defisr DMA2_Stream5_IRQHandler
defisr DMA2_Stream6_IRQHandler
defisr DMA2_Stream7_IRQHandler
defisr USART6_IRQHandler
defisr I2C3_EV_IRQHandler
defisr I2C3_ER_IRQHandler
defisr OTG_HS_EP1_OUT_IRQHandler
defisr OTG_HS_EP1_IN_IRQHandler
defisr OTG_HS_WKUP_IRQHandler
defisr OTG_HS_IRQHandler
defisr DCMI_IRQHandler
defisr CRYP_IRQHandler
defisr HASH_RNG_IRQHandler
defisr FPU_IRQHandler
defisr UART7_IRQHandler
defisr UART8_IRQHandler
defisr SPI4_IRQHandler
defisr SPI5_IRQHandler
defisr SPI6_IRQHandler
defisr SAI1_IRQHandler
defisr LCD_TFT_IRQHandler
defisr LCD_TFT_1_IRQHandler
defisr DMA2D_IRQHandler

.data
Variables:

.text
.macro LED_TOGGLE gpio, pin
    ldr r0, =\gpio
    ldr r1, [r0]
    eor r1, r1, #(1 << \pin)
    str r1, [r0]
.endm

.type Reset_Handler, %function
.global Reset_Handler
Reset_Handler:

    @ RESET TIM2 Clock
    ldr r0, =RCC_APB1RSTR
    ldr r1, [r0]
    orr r1, r1, #1
    str r1, [r0]

    @ Enable TIM2 Clock
    ldr r0, =RCC_APB1ENR
    ldr r1, [r0]
    orr r1, r1, #1
    str r1, [r0]

    @ Enable GPIOx Clock GPIOB = 1, GPIOD = 3
    ldr r0, =RCC_AHB1ENR
    ldr r1, [r0]
    orr r1, r1, #(1 << 1)
    str r1, [r0]
    orr r1, r1, #(1 << 3)
    str r1, [r0]

    @ Disable WWDG ACTIVATION BIT (7)
    ldr r0, =WWDG_CR
    ldr r1, =0
    orr r1, r1, #(1 << 7)
    str r1, [r0]

    @ Set GPIOB_MODER
    ldr r0, =GPIOB_MODER
    orr r1, r1, #(PB7_MODE)
    str r1, [r0]

    @ Set GPIOD_MODER
    ldr r0, =GPIOD_MODER
    orr r1, r1, #(PD7_MODE)
    str r1, [r0]

    @ Set TIM2_IRQ PRI FIRST 
    ldr r0, =0xE000E400
    mov r1, #0x00FF00
    mov r2, #0
    setvectorpriloop:
        str r1, [r0, r2, LSL #2]
        add r2, r2, #1 
        cmp r2, #60
        blt setvectorpriloop
    ldr r0, =NVIC_IPR7
    mov r1, #0
    str r1, [r0]

    @NVIC_ICPR0 TIM2 (28)
    ldr r0, =NVIC_ICPR0
    orr r1, r1, #(1 << TIM2_IRQ)
    str r1, [r0]

    @ Enable NVIC_ISER0 TIMER2_IRQ
    ldr r0, =NVIC_ISER0
    ldr r1, =0
    str r1, [r0]
    orr r1, r1, #(1 << TIM2_IRQ)
    str r1, [r0]

    @ Enable TIM2_CR1
    ldr r0, =TIM2_CR1
    ldr r1, [r0]
    orr r1, r1, #(1 << 2) @ CR1_URS = TIM2_CR1 + 2
    str r1, [r0]
    ldr r0, =TIM2_CR1
    ldr r1, [r0]
    orr r1, r1, #(1 << 7) @ CR1_ARPE = TIM2_CR1 + 7
    str r1, [r0]

    @ SET TIME VALUE
    ldr r0, =TIM2_ARR
    ldr r1, =TimerValue
    str r1, [r0]

    @ Enable TIM2_DIER UIE
    ldr r0, =TIM2_DIER
    ldr r1, [r0]
    orr r1, #1
    str r1, [r0]

    @ TIM2_PSC = CLOCK / (7999 + 1) = 22.5KHz
    ldr r0, =TIM2_PSC
    ldr r1, =7999
    str r1, [r0]

    @ ACK EVENT TIM2
    ldr r0, =TIM2_EGR
    ldr r1, [r0]
    orr r1, #1
    str r1, [r0]

    @ START TIMER2
    ldr r0, =TIM2_CR1
    ldr r1, [r0]
    orr r1, r1, #1 @ (1 << 0) CR1_CEN = TIM2_CR1 + 0
    str r1, [r0]

    loop:
        wfi
        b loop

.type TIM2_IRQHandler, %function
TIM2_IRQHandler:
    push { lr }
    ldr r0, =TIM2_SR
    cmp r0, #0x1
    
    beq Tim2LedToggle
    bne InterruptHandlerExit
    
Tim2LedToggle:
    LED_TOGGLE GPIOB_ODR, 7
    LED_TOGGLE GPIOD_ODR, 7

    ldr r0, =TIM2_SR
    mov r1, #0
    str r1, [r0]
    ldr r0, =TIM2_EGR
    mov r1, #1
    str r1, [r0]
    @pop { lr }
    @bl Reset_Handler
    bx lr

InterruptHandlerExit:

    ldr r0, =TIM2_SR
    mov r1, #0
    str r1, [r0]

    @ ACK EVENT TIM2
    ldr r0, =TIM2_EGR
    mov r1, #1
    str r1, [r0]
    @pop { lr }
    @bl Reset_Handler
    bx lr
