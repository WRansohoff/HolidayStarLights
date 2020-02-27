/* Vector table and reset handler for STM32F103xB chips. */
.syntax unified
.cpu cortex-m3
.thumb

.global vtable
.global default_interrupt_handler

.type vtable, %object
.section .vector_table,"a",%progbits
vtable:
  .word _estack
  .word reset_handler
  .word NonMaskableInt_IRQn_handler
  .word HardFault_IRQn_handler
  .word MemoryManagement_IRQn_handler
  .word BusFault_IRQn_handler
  .word UsageFault_IRQn_handler
  .word 0
  .word 0
  .word 0
  .word 0
  .word SVCall_IRQn_handler
  .word DebugMonitor_IRQn_handler
  .word 0
  .word PendSV_IRQn_handler
  .word SysTick_IRQn_handler
  .word WWDG_IRQn_handler
  .word PVD_IRQn_handler
  .word TAMPER_IRQn_handler
  .word RTC_IRQn_handler
  .word FLASH_IRQn_handler
  .word RCC_IRQn_handler
  .word EXTI0_IRQn_handler
  .word EXTI1_IRQn_handler
  .word EXTI2_IRQn_handler
  .word EXTI3_IRQn_handler
  .word EXTI4_IRQn_handler
  .word DMA1_Channel1_IRQn_handler
  .word DMA1_Channel2_IRQn_handler
  .word DMA1_Channel3_IRQn_handler
  .word DMA1_Channel4_IRQn_handler
  .word DMA1_Channel5_IRQn_handler
  .word DMA1_Channel6_IRQn_handler
  .word DMA1_Channel7_IRQn_handler
  .word ADC1_2_IRQn_handler
  .word USB_HP_CAN1_TX_IRQn_handler
  .word USB_LP_CAN1_RX0_IRQn_handler
  .word CAN1_RX1_IRQn_handler
  .word CAN1_SCE_IRQn_handler
  .word EXTI9_5_IRQn_handler
  .word TIM1_BRK_IRQn_handler
  .word TIM1_UP_IRQn_handler
  .word TIM1_TRG_COM_IRQn_handler
  .word TIM1_CC_IRQn_handler
  .word TIM2_IRQn_handler
  .word TIM3_IRQn_handler
  .word TIM4_IRQn_handler
  .word I2C1_EV_IRQn_handler
  .word I2C1_ER_IRQn_handler
  .word I2C2_EV_IRQn_handler
  .word I2C2_ER_IRQn_handler
  .word SPI1_IRQn_handler
  .word SPI2_IRQn_handler
  .word USART1_IRQn_handler
  .word USART2_IRQn_handler
  .word USART3_IRQn_handler
  .word EXTI15_10_IRQn_handler
  .word RTC_Alarm_IRQn_handler
  .word USBWakeUp_IRQn_handler

  .weak NonMaskableInt_IRQn_handler
  .thumb_set NonMaskableInt_IRQn_handler,default_interrupt_handler
  .weak HardFault_IRQn_handler
  .thumb_set HardFault_IRQn_handler,default_interrupt_handler
  .weak MemoryManagement_IRQn_handler
  .thumb_set MemoryManagement_IRQn_handler,default_interrupt_handler
  .weak BusFault_IRQn_handler
  .thumb_set BusFault_IRQn_handler,default_interrupt_handler
  .weak UsageFault_IRQn_handler
  .thumb_set UsageFault_IRQn_handler,default_interrupt_handler
  .weak SVCall_IRQn_handler
  .thumb_set SVCall_IRQn_handler,default_interrupt_handler
  .weak DebugMonitor_IRQn_handler
  .thumb_set DebugMonitor_IRQn_handler,default_interrupt_handler
  .weak PendSV_IRQn_handler
  .thumb_set PendSV_IRQn_handler,default_interrupt_handler
  .weak SysTick_IRQn_handler
  .thumb_set SysTick_IRQn_handler,default_interrupt_handler
  .weak WWDG_IRQn_handler
  .thumb_set WWDG_IRQn_handler,default_interrupt_handler
  .weak PVD_IRQn_handler
  .thumb_set PVD_IRQn_handler,default_interrupt_handler
  .weak TAMPER_IRQn_handler
  .thumb_set TAMPER_IRQn_handler,default_interrupt_handler
  .weak RTC_IRQn_handler
  .thumb_set RTC_IRQn_handler,default_interrupt_handler
  .weak FLASH_IRQn_handler
  .thumb_set FLASH_IRQn_handler,default_interrupt_handler
  .weak RCC_IRQn_handler
  .thumb_set RCC_IRQn_handler,default_interrupt_handler
  .weak EXTI0_IRQn_handler
  .thumb_set EXTI0_IRQn_handler,default_interrupt_handler
  .weak EXTI1_IRQn_handler
  .thumb_set EXTI1_IRQn_handler,default_interrupt_handler
  .weak EXTI2_IRQn_handler
  .thumb_set EXTI2_IRQn_handler,default_interrupt_handler
  .weak EXTI3_IRQn_handler
  .thumb_set EXTI3_IRQn_handler,default_interrupt_handler
  .weak EXTI4_IRQn_handler
  .thumb_set EXTI4_IRQn_handler,default_interrupt_handler
  .weak DMA1_Channel1_IRQn_handler
  .thumb_set DMA1_Channel1_IRQn_handler,default_interrupt_handler
  .weak DMA1_Channel2_IRQn_handler
  .thumb_set DMA1_Channel2_IRQn_handler,default_interrupt_handler
  .weak DMA1_Channel3_IRQn_handler
  .thumb_set DMA1_Channel3_IRQn_handler,default_interrupt_handler
  .weak DMA1_Channel4_IRQn_handler
  .thumb_set DMA1_Channel4_IRQn_handler,default_interrupt_handler
  .weak DMA1_Channel5_IRQn_handler
  .thumb_set DMA1_Channel5_IRQn_handler,default_interrupt_handler
  .weak DMA1_Channel6_IRQn_handler
  .thumb_set DMA1_Channel6_IRQn_handler,default_interrupt_handler
  .weak DMA1_Channel7_IRQn_handler
  .thumb_set DMA1_Channel7_IRQn_handler,default_interrupt_handler
  .weak ADC1_2_IRQn_handler
  .thumb_set ADC1_2_IRQn_handler,default_interrupt_handler
  .weak USB_HP_CAN1_TX_IRQn_handler
  .thumb_set USB_HP_CAN1_TX_IRQn_handler,default_interrupt_handler
  .weak USB_LP_CAN1_RX0_IRQn_handler
  .thumb_set USB_LP_CAN1_RX0_IRQn_handler,default_interrupt_handler
  .weak CAN1_RX1_IRQn_handler
  .thumb_set CAN1_RX1_IRQn_handler,default_interrupt_handler
  .weak CAN1_SCE_IRQn_handler
  .thumb_set CAN1_SCE_IRQn_handler,default_interrupt_handler
  .weak EXTI9_5_IRQn_handler
  .thumb_set EXTI9_5_IRQn_handler,default_interrupt_handler
  .weak TIM1_BRK_IRQn_handler
  .thumb_set TIM1_BRK_IRQn_handler,default_interrupt_handler
  .weak TIM1_UP_IRQn_handler
  .thumb_set TIM1_UP_IRQn_handler,default_interrupt_handler
  .weak TIM1_TRG_COM_IRQn_handler
  .thumb_set TIM1_TRG_COM_IRQn_handler,default_interrupt_handler
  .weak TIM1_CC_IRQn_handler
  .thumb_set TIM1_CC_IRQn_handler,default_interrupt_handler
  .weak TIM2_IRQn_handler
  .thumb_set TIM2_IRQn_handler,default_interrupt_handler
  .weak TIM3_IRQn_handler
  .thumb_set TIM3_IRQn_handler,default_interrupt_handler
  .weak TIM4_IRQn_handler
  .thumb_set TIM4_IRQn_handler,default_interrupt_handler
  .weak I2C1_EV_IRQn_handler
  .thumb_set I2C1_EV_IRQn_handler,default_interrupt_handler
  .weak I2C1_ER_IRQn_handler
  .thumb_set I2C1_ER_IRQn_handler,default_interrupt_handler
  .weak I2C2_EV_IRQn_handler
  .thumb_set I2C2_EV_IRQn_handler,default_interrupt_handler
  .weak I2C2_ER_IRQn_handler
  .thumb_set I2C2_ER_IRQn_handler,default_interrupt_handler
  .weak SPI1_IRQn_handler
  .thumb_set SPI1_IRQn_handler,default_interrupt_handler
  .weak SPI2_IRQn_handler
  .thumb_set SPI2_IRQn_handler,default_interrupt_handler
  .weak USART1_IRQn_handler
  .thumb_set USART1_IRQn_handler,default_interrupt_handler
  .weak USART2_IRQn_handler
  .thumb_set USART2_IRQn_handler,default_interrupt_handler
  .weak USART3_IRQn_handler
  .thumb_set USART3_IRQn_handler,default_interrupt_handler
  .weak EXTI15_10_IRQn_handler
  .thumb_set EXTI15_10_IRQn_handler,default_interrupt_handler
  .weak RTC_Alarm_IRQn_handler
  .thumb_set RTC_Alarm_IRQn_handler,default_interrupt_handler
  .weak USBWakeUp_IRQn_handler
  .thumb_set USBWakeUp_IRQn_handler,default_interrupt_handler
.size vtable, .-vtable

/* Default interrupt handler, if no other one is defined. */
.section .text.default_interrupt_handler,"ax",%progbits
default_interrupt_handler:
  default_interrupt_loop:
    B default_interrupt_loop
.size default_interrupt_handler, .-default_interrupt_handler

/* Reset handler. */
.type reset_handler, %function
reset_handler:
  // Set the stack pointer to the end of the stack.
  LDR  r0, =_estack
  MOV  sp, r0
  // Branch to the 'main' method.
  B    main
.size reset_handler, .-reset_handler
