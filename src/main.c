#include "main.h"

// Default core system clock frequency.
volatile uint32_t SystemCoreClock = 8000000;

// SysTick counter definition.
volatile uint32_t systick = 0;

// Delay for a specified number of milliseconds.
// TODO: Prevent rollover bug on the 'systick' value.
void delay_ms( uint32_t ms ) {
  // Calculate the tick value when the system should stop delaying.
  uint32_t next = systick + ms;

  // Wait until the system reaches that tick value.
  // Use the 'wait for interrupt' instruction to save power.
  while ( systick < next ) {
    #ifdef VVC_STM32
      __asm__( "WFI" );
    #elif VVC_GD32V
      __WFI();
    #endif
  }
}

/**
 * Main program.
 */
int main(void) {
  // Copy initialized data from .sidata (Flash) to .data (RAM)
  memcpy( &_sdata, &_sidata, ( ( void* )&_edata - ( void* )&_sdata ) );
  // Clear the .bss section in RAM.
  memset( &_sbss, 0x00, ( ( void* )&_ebss - ( void* )&_sbss ) );

  /* Clock configuration:
   * The LEDs expect an 800KHz transmission rate, but the duty cycle
   * for '1' and '0' varies. So, we'll use one byte per 'color bit' at
   * a baud rate of 6.4MHz. We can stretch the tailing 'low' periods
   * a bit though, so 6MHz is much easier to achieve. So we need a
   * core clock rate of (6MHz * 2^N). At the maximum 72MHz clock rate,
   * N is not an integer. So let's use 48MHz where N = 3.
   */
  // Set 1 wait state in flash and enable the prefetch buffer.
  FLASH->ACR &= ~(FLASH_ACR_LATENCY);
  #ifdef VVC_STM32
    FLASH->ACR |=  (0x1 << FLASH_ACR_LATENCY_Pos |
                    FLASH_ACR_PRFTBE);
  #elif  VVC_GD32V
    FLASH->ACR |=  (0x2 << FLASH_ACR_LATENCY_Pos);
  #endif
  // Enable the 8MHz external crystal oscillator.
  RCC->CR    |=  (RCC_CR_HSEON);
  while (!(RCC->CR & RCC_CR_HSERDY)) {};
  // Set the HSE oscillator as the system clock source.
  RCC->CFGR  &= ~(RCC_CFGR_SW);
  RCC->CFGR  |=  (RCC_CFGR_SW_HSE);
  // Set the PLL multiplication factor.
  RCC->CFGR  &= ~(RCC_CFGR_PLLMULL);
  #ifdef VVC_STM32
    RCC->CFGR  |=  (RCC_CFGR_PLLMULL6);
  #elif  VVC_GD32V
    RCC->CFGR  |=  (RCC_CFGR_PLLMULL12);
  #endif
  // Set the PLL to use the HSE oscillator.
  RCC->CFGR  |=  (RCC_CFGR_PLLSRC);
  // Enable the PLL.
  RCC->CR    |=  (RCC_CR_PLLON);
  while (!(RCC->CR & RCC_CR_PLLRDY)) {};
  // Set the PLL as the system clock source.
  RCC->CFGR  &= ~(RCC_CFGR_SW);
  RCC->CFGR  |=  (RCC_CFGR_SW_PLL);
  // The system clock is now 48MHz or 96MHz.
  #ifdef VVC_STM32
    SystemCoreClock = 48000000;
  #elif  VVC_GD32V
    SystemCoreClock = 96000000;
  #endif

  #ifdef VVC_STM32
    // Setup the SysTick peripheral to 1ms ticks.
    SysTick_Config( SystemCoreClock / 1000 );
  #elif  VVC_GD32V
    // Set up the global timer to generate an interrupt every ms.
    // Figure out how many interrupts are available.
    uint32_t max_irqn = *( volatile uint32_t * )( ECLIC_ADDR_BASE + ECLIC_INFO_OFFSET );
    max_irqn &= ( 0x00001FFF );
    // Initialize the 'ECLIC' interrupt controller.
    eclic_init( max_irqn );
    eclic_mode_enable();
    // Set 'vector mode' so the timer interrupt uses the vector table.
    eclic_set_vmode( CLIC_INT_TMR );
    // Enable the timer interrupt (#7) with low priority and 'level'.
    eclic_enable_interrupt( CLIC_INT_TMR );
    eclic_set_irq_lvl_abs( CLIC_INT_TMR, 1 );
    eclic_set_irq_priority( CLIC_INT_TMR, 1 );
    // Set the timer's comparison value to (frequency / 1000).
    *( volatile uint64_t * )( TIMER_CTRL_ADDR + TIMER_MTIMECMP ) = ( TIMER_FREQ / 1000 );
    // Reset the timer value to zero.
    *( volatile uint64_t * )( TIMER_CTRL_ADDR + TIMER_MTIME ) = 0;
    // Re-enable interrupts globally.
    set_csr( mstatus, MSTATUS_MIE );
  #endif

  // Zero out the colors array, to ensure latching period is all low.
  memset( &colors, 0x00, COLOR_ARRAY_LEN );
  // Fill the 'colors' buffer with 'off' colors.
  for ( int i = 0; i < ( NUM_STARS * STAR_LEDS ); ++i ) {
    set_px_rgb( &( colors[ i * 24 ] ), 0x00, 0x00, 0x00 );
  }
  // Initialize the star structs.
  for ( int i = 0; i < NUM_STARS; ++i ) {
    stars[ i ].my_colors = &( colors[ i * STAR_LEDS * 24 ] );
    stars[ i ].last_step = 0;
    stars[ i ].next_step = STEP_DUR;
    //stars[ i ].cur_pattern = rainbow;
    stars[ i ].cur_pattern = rainbow_lp;
    /* Set initial patterns to 'breathing RGB'?
    if ( i % 3 == 0 ) {
      stars[ i ].cur_pattern = breathe_r;
    }
    else if ( i % 3 == 1 ) {
      stars[ i ].cur_pattern = breathe_g;
    }
    else {
      stars[ i ].cur_pattern = breathe_b;
    }
    // ...or with red/green christmas colors?
    if ( i % 2 == 0 ) {
      stars[ i ].cur_pattern = xmas_odd;
    }
    else {
      stars[ i ].cur_pattern = xmas_even;
    }
    */
  }

  // Enable peripheral clocks: AFIO, GPIOA/B/C, DMA1, SPI1.
  RCC->APB2ENR  |=  ( RCC_APB2ENR_AFIOEN |
                      RCC_APB2ENR_IOPAEN |
                      RCC_APB2ENR_IOPBEN |
                      RCC_APB2ENR_IOPCEN |
                      RCC_APB2ENR_SPI1EN );
  RCC->AHBENR   |=  ( RCC_AHBENR_DMA1EN );

  // PB5 SPI1 MOSI pin setup: push-pull output, alt. func., mid-speed.
  GPIOB->CRL &= ~( 0xF << 20 );
  GPIOB->CRL |=  ( 0x9 << 20 );
  AFIO->MAPR |=  ( AFIO_MAPR_SPI1_REMAP );
  // LED pin(s) setup: output, push-pull, low-speed.
  #ifdef VVC_STM32
    GPIOB->CRH &= ~( 0xF << 16 );
    GPIOB->CRH |=  ( 0x2 << 16 );
  #elif  VVC_GD32V
    GPIOA->CRL &= ~( ( 0xF << 4 ) | ( 0xF << 8 ) );
    GPIOA->CRL |=  ( ( 0x2 << 4 ) | ( 0x2 << 8 ) );
    GPIOA->ODR &= ~( 0x1 << 1 );
    GPIOA->ODR |=  ( 0x1 << 2 );
  #endif

  // DMA setup: on STM32F103s, SPI1_TX is mapped to DMA1, Channel 3.
  // - Memory-to-peripheral mode.
  // - Circular mode enabled for continuous transfer.
  // - Increment source ptr, don't increment destination ptr.
  // - 8-bit transfer length.
  // - High-priority. Not that priority matters; it's the only one.
  DMA1_Channel3->CCR &= ~( DMA_CCR_MEM2MEM |
                           DMA_CCR_PL |
                           DMA_CCR_MSIZE |
                           DMA_CCR_PSIZE |
                           DMA_CCR_PINC |
                           DMA_CCR_EN );
  DMA1_Channel3->CCR |=  ( ( 0x2 << DMA_CCR_PL_Pos ) |
                           DMA_CCR_MINC |
                           DMA_CCR_CIRC |
                           DMA_CCR_DIR );
  // Set source memory address to the 'colors' array.
  DMA1_Channel3->CMAR  = ( uint32_t )&( colors );
  // Set destination peripheral address to the SPI1 data register.
  DMA1_Channel3->CPAR  = ( uint32_t )&( SPI1->DR );
  // Set the number of color bits. The maximum is 64K, but it's safe
  // to assume we will be in that range since there's only 20K of RAM.
  DMA1_Channel3->CNDTR = ( uint16_t )( COLOR_ARRAY_LEN );
  // Enable the DMA channel.
  DMA1_Channel3->CCR  |= ( DMA_CCR_EN );

  // SPI1 setup: host mode, 6MHz baud rate, sw cs pin control,
  // TX DMA enabled, 8-bit frames, msb-first, enable the peripheral.
  // Some of those settings are the default state after a reset.
  SPI1->CR2  |=  ( SPI_CR2_TXDMAEN );
  SPI1->CR1  |=  ( SPI_CR1_SSM |
                   SPI_CR1_SSI |
                   SPI_CR1_MSTR |
                   SPI_CR1_SPE |
                 #ifdef VVC_STM32
                   0x2 << SPI_CR1_BR_Pos );
                 #elif  VVC_GD32V
                   0x3 << SPI_CR1_BR_Pos );
                 #endif

  // Send new colors and blink the on-board LED at an interval.
  while ( 1 ) {
    // Delay briefly.
    delay_ms( 50 );
    // Step the lighting display.
    for ( int i = 0; i < NUM_STARS; ++i ) {
      step_star( &( stars[ i ] ) );
    }
    // Toggle on-board LED.
    #ifdef VVC_STM32
      GPIOB->ODR ^=  ( 1 << 12 );
    #elif  VVC_GD32V
      GPIOA->ODR ^=  ( ( 0x1 << 1 ) | ( 0x1 << 2 ) );
    #endif
  }

  return 0; // lol
}

#ifdef VVC_STM32
  // SysTick interrupt handler: increment the global 'systick' value.
  void SysTick_IRQn_handler( void ) {
    ++systick;
  }
#elif VVC_GD32V
  // System timer interrupt.
  __attribute__( ( interrupt ) )
  void eclic_mtip_handler( void ) {
    // Increment the global 'tick' value.
    systick++;
    // Reset the 'mtime' value to zero.
    *( volatile uint64_t * )( TIMER_CTRL_ADDR + TIMER_MTIME ) = 0;
  }
#endif
