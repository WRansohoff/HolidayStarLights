#include "main.h"

// Default core system clock frequency.
uint32_t core_clock_hz = 8000000;

// SysTick counter definition.
volatile uint32_t systick = 0;

// Systime to advance to the next 'substep' or 'cycle'.
uint32_t next_cyc = 0;
// Systime to advance to the next lighting pattern.
uint32_t next_pat = 0;

// Reset handler: set the stack pointer and branch to main().
__attribute__( ( naked ) ) void reset_handler( void ) {
  // Set the stack pointer to the 'end of stack' value.
  __asm__( "LDR r0, =_estack\n\t"
           "MOV sp, r0" );
  // Branch to main().
  __asm__( "B main" );
}

// Delay for a specified number of milliseconds.
// TODO: Prevent rollover bug on the 'systick' value.
void delay_ms( uint32_t ms ) {
  // Calculate the tick value when the system should stop delaying.
  uint32_t next = systick + ms;

  // Wait until the system reaches that tick value.
  // Use the 'wait for interrupt' instruction to save power.
  while ( systick < next ) { __asm__( "WFI" ); }
}

// Get a 'cycling rainbow' color, given a maximum and 'progress' time
// value. The result is returned as a 32-bit int; 0xRRGGBB00
// TODO: There are faster ways to do this, but...meh. It's a one-off.
uint32_t rainbow_cycle( uint32_t prg, uint32_t max ) {
  // No color if the value is out of range.
  if ( prg > max ) { return 0x00000000; }
  uint8_t r, g, b;
  r = g = b = 0x00;
  int step = max / 6;
  // Red color.
  if ( ( ( prg > 0 ) && ( prg < step ) ) || ( prg > ( step * 5 ) ) ) {
    r = 0xFF;
  }
  else if ( ( prg > ( step * 2 ) && ( prg < ( step * 4 ) ) ) ) {
    r = 0x00;
  }
  else if ( prg < ( step * 2 ) ) {
    r = 0xFF - ( ( ( prg - step ) * 0xFF ) / step );
  }
  else {
    r = ( ( prg - ( step * 4 ) ) * 0xFF ) / step;
  }
  // Green color.
  if ( ( ( prg > step ) && ( prg < ( step * 3 ) ) ) ) {
    g = 0xFF;
  }
  else if ( ( prg > ( step * 4 ) ) ) {
    g = 0x00;
  }
  else if ( ( prg > ( step * 3 ) ) && ( prg < ( step * 4 ) ) ) {
    g = 0xFF - ( ( ( prg - ( step * 3 ) ) * 0xFF ) / step );
  }
  else {
    g = ( prg * 0xFF ) / step;
  }
  // Blue color.
  if ( ( prg > ( step * 3 ) && prg < ( step * 5 ) ) ) {
    b = 0xFF;
  }
  else if ( ( prg < ( step * 2 ) ) ) {
    b = 0x00;
  }
  else if ( ( prg > ( step * 5 ) ) ) {
    b = 0xFF - ( ( ( prg - ( step * 5 ) ) * 0xFF ) / step );
  }
  else {
    b = ( ( prg - ( step * 2 ) ) * 0xFF ) / step;
  }
  return ( uint32_t )( ( r << 24 ) | ( g << 16 ) | ( b << 8 ) );
}

// Set a 24-byte GRB pixel color from 3 RGB bytes.
void set_px_rgb( uint8_t* px, uint8_t r, uint8_t g, uint8_t b ) {
  // Green color.
  for ( int i = 0; i < 8; ++i ) {
    if ( g & ( 1 << ( 7 - i ) ) ) { px[ i ] = WS2812_ON; }
    else { px[ i ] = WS2812_OFF; }
  }
  // Red color.
  for ( int i = 0; i < 8; ++i ) {
    if ( r & ( 1 << ( 7 - i ) ) ) { px[ i + 8 ] = WS2812_ON; }
    else { px[ i + 8 ] = WS2812_OFF; }
  }
  // Blue color.
  for ( int i = 0; i < 8; ++i ) {
    if ( b & ( 1 << ( 7 - i ) ) ) { px[ i + 16 ] = WS2812_ON; }
    else { px[ i + 16 ] = WS2812_OFF; }
  }
}

// Step one star's lighting pattern.
void step_star( star_t* star ) {
  // Use the same definition of 'now' for the whole function.
  int this_tick = systick;
  // Move to the next pattern if necessary.
  if ( star->next_step < this_tick ) {
    star->last_step = this_tick;
    star->next_step = this_tick + STEP_DUR;
    ++star->cur_pattern;
    if ( star->cur_pattern == ls_max ) { star->cur_pattern = ls_min; }
  }

  // Update colors to match the current pattern.
  // The downside of using an ancient STM32F1 chip is that it
  // lacks floating-point hardware, so integer math is much faster.
  uint8_t step_brightness = ( ( this_tick - star->last_step ) * 0xFF ) / ( STEP_DUR / 2 );
  if ( ( this_tick - star->last_step ) > ( STEP_DUR / 2 ) ) {
    step_brightness = 0xFF - ( step_brightness - 0xFF );
  }
  if ( star->cur_pattern == breathe_r ) {
    for ( int i = 0; i < STAR_LEDS; ++i )  {
      set_px_rgb( &( star->my_colors[ i * 24 ] ), step_brightness, 0x00, 0x00 );
    }
  }
  else if ( star->cur_pattern == breathe_g ) {
    for ( int i = 0; i < STAR_LEDS; ++i )  {
      set_px_rgb( &( star->my_colors[ i * 24 ] ), 0x00, step_brightness, 0x00 );
    }
  }
  else if ( star->cur_pattern == breathe_b ) {
    for ( int i = 0; i < STAR_LEDS; ++i )  {
      set_px_rgb( &( star->my_colors[ i * 24 ] ), 0x00, 0x00, step_brightness );
    }
  }
  else if ( star->cur_pattern == xmas_odd ) {
    for ( int i = 0; i < STAR_LEDS; ++i )  {
      if ( i % 2 == 0 ) {
        set_px_rgb( &( star->my_colors[ i * 24 ] ), step_brightness, 0x00, 0x00 );
      }
      else {
        set_px_rgb( &( star->my_colors[ i * 24 ] ), 0x00, step_brightness, 0x00 );
      }
    }
  }
  else if ( star->cur_pattern == xmas_even ) {
    for ( int i = 0; i < STAR_LEDS; ++i )  {
      if ( i % 2 == 1 ) {
        set_px_rgb( &( star->my_colors[ i * 24 ] ), step_brightness, 0x00, 0x00 );
      }
      else {
        set_px_rgb( &( star->my_colors[ i * 24 ] ), 0x00, step_brightness, 0x00 );
      }
    }
  }
  else if ( star->cur_pattern == rainbow ) {
    uint32_t r_step = STEP_DUR / STAR_LEDS;
    uint32_t r_prg = star->next_step - this_tick;
    for ( int i = 0; i < STAR_LEDS; ++i ) {
      uint32_t r_col = rainbow_cycle( r_prg, STEP_DUR );
      set_px_rgb( &( star->my_colors[ i * 24 ] ),
                  ( ( r_col >> 24 ) & 0xFF ),
                  ( ( r_col >> 16 ) & 0xFF ),
                  ( ( r_col >>  8 ) & 0xFF ) );
      r_prg += r_step;
      if ( r_prg > STEP_DUR ) { r_prg -= STEP_DUR; }
    }
  }
  else {
    for ( int i = 0; i < STAR_LEDS; ++i )  {
      set_px_rgb( &( star->my_colors[ i * 24 ] ), 0x00, 0x00, 0x00 );
    }
  }
}

// Increment the lighting patterns.
void step_lights() {
  for ( int i = 0; i < NUM_STARS; ++i ) {
    step_star( &( stars[ i ] ) );
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
  FLASH->ACR |=  (FLASH_ACR_LATENCY_1 |
                  FLASH_ACR_PRFTBE);
  // Enable the 8MHz external crystal oscillator.
  RCC->CR    |=  (RCC_CR_HSEON);
  while (!(RCC->CR & RCC_CR_HSERDY)) {};
  // Set the HSE oscillator as the system clock source.
  RCC->CFGR  &= ~(RCC_CFGR_SW);
  RCC->CFGR  |=  (RCC_CFGR_SW_HSE);
  // Set the PLL multiplication factor to 6, for 8*6=48MHz.
  RCC->CFGR  &= ~(RCC_CFGR_PLLMULL);
  RCC->CFGR  |=  (RCC_CFGR_PLLMULL6);
  // Set the PLL to use the HSE oscillator.
  RCC->CFGR  |=  (RCC_CFGR_PLLSRC);
  // Enable the PLL.
  RCC->CR    |=  (RCC_CR_PLLON);
  while (!(RCC->CR & RCC_CR_PLLRDY)) {};
  // Set the PLL as the system clock source.
  RCC->CFGR  &= ~(RCC_CFGR_SW);
  RCC->CFGR  |=  (RCC_CFGR_SW_PLL);
  // The system clock is now 48MHz.
  core_clock_hz = 48000000;

  // Setup the SysTick peripheral to 1ms ticks.
  SysTick_Config( core_clock_hz / 1000 );

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
    stars[ i ].cur_pattern = rainbow;
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

  // Enable peripheral clocks: AFIO, GPIOB, DMA1, SPI1.
  RCC->APB2ENR  |=  ( RCC_APB2ENR_AFIOEN |
                      RCC_APB2ENR_IOPBEN |
                      RCC_APB2ENR_SPI1EN );
  RCC->AHBENR   |=  ( RCC_AHBENR_DMA1EN );

  // PB5 SPI1 MOSI pin setup: push-pull output, alt. func., mid-speed.
  GPIOB->CRL &= ~( 0xF << 20 );
  GPIOB->CRL |=  ( 0x9 << 20 );
  AFIO->MAPR |=  ( AFIO_MAPR_SPI1_REMAP );
  // PB12 LED pin setup: output, push-pull, low-speed.
  GPIOB->CRH &= ~( 0xF << 16 );
  GPIOB->CRH |=  ( 0x2 << 16 );

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

  // SPI1 setup: host mode, /8 baud rate division, sw cs pin control,
  // TX DMA enabled, 8-bit frames, msb-first, enable the peripheral.
  // Some of those settings are the default state after a reset.
  SPI1->CR2  |=  ( SPI_CR2_TXDMAEN );
  SPI1->CR1  |=  ( SPI_CR1_SSM |
                   SPI_CR1_SSI |
                   SPI_CR1_MSTR |
                   SPI_CR1_SPE |
                   0x2 << SPI_CR1_BR_Pos );

  // Send new colors and blink the on-board LED every second.
  while ( 1 ) {
    // Wait a sec...
    delay_ms( 50 );
    // Step the lighting display.
    step_lights();
    // Toggle on-board LED.
    GPIOB->ODR ^=  ( 1 << 12 );
  }

  return 0; // lol
}

// SysTick interrupt handler: increment the global 'systick' value.
void SysTick_IRQn_handler( void ) {
  ++systick;
}
