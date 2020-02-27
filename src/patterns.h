#ifndef _VVC_PATTERNS_H
#define _VVC_PATTERNS_H

#include <stdint.h>
#include <string.h>

#ifdef VVC_STM32
  #include "stm32f1xx.h"
#elif  VVC_GD32V
  #include "gd32vf103.h"
  #include "n200_func.h"
  #include "riscv_encoding.h"
#endif

// SPI timing values for color bits. (One SPI byte = one color bit)
#define WS2812_ON  ( 0xFC )
#define WS2812_OFF ( 0xC0 )

// Number of LEDs per star.
#define STAR_LEDS ( 24 )
// Number of stars in the string.
#define NUM_STARS ( 8 )
// Duration of each lighting 'step', in ms.
#define STEP_DUR  ( 2000 )
// Number of 'substeps' or 'cycles' in each lighting 'step'
#define STEP_CYC  ( 5 )

// Values declared elsewhere.
extern volatile uint32_t systick;

// Enumeration for lighting patterns.
typedef enum {
  ls_min = 0,
  rainbow_lp = 0,
  ls_max, /* (Move pattern names below this line to disable them.) */
  rainbow,
  xmas_odd,
  xmas_even,
  breathe_r,
  breathe_g,
  breathe_b,
} light_steps;

// Star struct.
typedef struct {
  uint8_t* my_colors;
  int cur_pattern;
  int last_step;
  int next_step;
} star_t;

// Helper methods for getting / setting colors.
// Get a 'pulsing rainbow' color based on temporal progress.
uint32_t rainbow_cycle( uint32_t prg, uint32_t max );
// Lower-power 'pulsing rainbow' color getter.
uint32_t rainbow_lp_cycle( uint32_t prg, uint32_t max );
// Set a 24-byte GRB pixel color from 3 RGB bytes.
void set_px_rgb( uint8_t* px, uint8_t r, uint8_t g, uint8_t b );
// Step one star's lighting pattern.
void step_star( star_t* star );

#endif
