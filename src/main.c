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

void set_target_rgb( uint8_t* px, uint8_t r, uint8_t g, uint8_t b ) {
  px[ 0 ] = r;
  px[ 1 ] = g;
  px[ 2 ] = b;
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

// Increment the lighting patterns.
void step_lights() {
}

// Send colors to the LED strip. TODO: Use DMA.
void send_colors() {
  // Send colors one by one.
  for ( int i = 0; i < ( NUM_COLOR_BITS ); ++i ) {
    while ( !( SPI1->SR & SPI_SR_TXE ) ) {};
    *( uint8_t* )&( SPI1->DR ) = colors[ i ];
  }
  // Latch.
  for ( int i = 0; i < 20; ++i ) {
    while ( !( SPI1->SR & SPI_SR_TXE ) ) {};
     *( uint8_t* )&( SPI1->DR ) = 0x00;
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
  // Set 2 wait states in flash and enable the prefetch buffer.
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
  /*
  // Set target colors.
  set_target_rgb( &( target_colors[ 0 ] ), 0x10, 0x10, 0x10 );
  set_target_rgb( &( target_colors[ 24 ] ), 0x40, 0x00, 0x00 );
  set_target_rgb( &( target_colors[ 48 ] ), 0x00, 0x40, 0x00 );
  set_target_rgb( &( target_colors[ 72 ] ), 0x40, 0x00, 0x40 );
  set_target_rgb( &( target_colors[ 96 ] ), 0x40, 0x40, 0x00 );
  set_target_rgb( &( target_colors[ 120 ] ), 0x00, 0x40, 0x40 );
  */
  for ( int i = 0; i < STAR_LEDS / 6; ++i ) {
    set_px_rgb( &( colors[ i * 144 ] ), 0x10, 0x10, 0x10 );
    set_px_rgb( &( colors[ i * 144 + 24 ] ), 0x10, 0x00, 0x00 );
    set_px_rgb( &( colors[ i * 144 + 48 ] ), 0x00, 0x10, 0x00 );
    set_px_rgb( &( colors[ i * 144 + 72 ] ), 0x10, 0x00, 0x10 );
    set_px_rgb( &( colors[ i * 144 + 96 ] ), 0x10, 0x10, 0x00 );
    set_px_rgb( &( colors[ i * 144 + 120 ] ), 0x00, 0x10, 0x10 );
    /*
    set_px_rgb( &( colors[ i * 144 ] ), 0xFF, 0xFF, 0xFF );
    set_px_rgb( &( colors[ i * 144 + 24 ] ), 0xFF, 0x00, 0x00 );
    set_px_rgb( &( colors[ i * 144 + 48 ] ), 0x00, 0xFF, 0x00 );
    set_px_rgb( &( colors[ i * 144 + 72 ] ), 0xFF, 0x00, 0xFF );
    set_px_rgb( &( colors[ i * 144 + 96 ] ), 0xFF, 0xFF, 0x00 );
    set_px_rgb( &( colors[ i * 144 + 120 ] ), 0x00, 0xFF, 0xFF );
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
    delay_ms( 1000 );
    // Step the lighting display.
    step_lights();
    // TODO: DMA instead of polling.
    //send_colors();
    // Toggle on-board LED.
    GPIOB->ODR ^=  ( 1 << 12 );
  }

  return 0; // lol
}

// SysTick interrupt handler: increment the global 'systick' value.
void SysTick_IRQn_handler( void ) {
  ++systick;
}
