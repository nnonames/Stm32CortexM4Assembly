.syntax unified
.cpu cortex-m4
.thumb

.word 0x20000400
.word 0x080000ed
.space 0xe4

.equ RCC_AHB1ENR, 0x40023830
.equ GPIOB_MODER, 0x40020400
.equ GPIOB_ODR, 0x40020414
.equ LED_PIN, 7
.equ DELAY_COUNT_ON, 1000000
.equ DELAY_COUNT_OFF, 500000
.equ LED_MODE, (1 << (LED_PIN * 2))

.global _start
_start:
    @ Enable GPIOB clock
    ldr r0, =RCC_AHB1ENR
    ldr r1, [r0]
    mov r2, #(1 << 1)
    orr r1, r1, r2
    str r1, [r0]

    @ Set GPIOB_MODER
    ldr r0, =GPIOB_MODER
    ldr r1, [r0]

    @ Set Output LED(PB7)
    ldr r2, =LED_MODE
    orr r1, r1, r2
    str r1, [r0]

loop:
    @ Set Delay Count
    ldr r2, =DELAY_COUNT_ON
    ldr r3, =DELAY_COUNT_OFF

    @ READ GPIOB_ODR
    ldr r0, =GPIOB_ODR

    @ LED ON
    mov r1, #(1 << LED_PIN)
    str r1, [r0]
    bl delay_ms
    
    @ LED OFF
    mov r1, #(0 << LED_PIN)
    str r1, [r0]
    bl delay_off
    b loop

delay_ms :
    sub r2, r2, #1
    cmp r2, #0
    bne delay_ms
    bx lr

delay_off :
    sub r3, r3, #1
    cmp r3, #0
    bne delay_off
    bx lr
