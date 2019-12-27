#ifndef _VVC_MAIN_H
#define _VVC_MAIN_H

#include <stdint.h>
#include <string.h>

#include "stm32f1xx.h"

// SPI timing values for color bits. (One SPI byte = one color bit)
#define WS2812_ON  ( 0xFC )
#define WS2812_OFF ( 0xC0 )

// Number of LEDs per star.
#define STAR_LEDS ( 23 )
// Number of stars in the string.
#define NUM_STARS ( 7 )
// Duration of each lighting 'step', in ms.
#define STEP_DUR  ( 2000 )
// Number of 'substeps' or 'cycles' in each lighting 'step'
#define STEP_CYC  ( 5 )

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

// Array of star structs.
star_t stars[ NUM_STARS ];

// LED colors buffer. (GRB*8)
// This is sort of a waste of RAM, but it makes DMA easier.
// You could use less memory by setting a faster SPI speed and
// stuffing more than one color bit into each byte, but this is easy.
#define NUM_COLOR_BYTES ( NUM_STARS * STAR_LEDS * 3 )
#define NUM_COLOR_BITS  ( NUM_COLOR_BYTES * 8 )
// (Add 20 cycles to hold a latching sequence)
#define COLOR_ARRAY_LEN ( NUM_COLOR_BITS + 64 )
uint8_t colors[ COLOR_ARRAY_LEN ];

// Pre-defined memory locations for program initialization.
extern uint32_t _sidata, _sdata, _edata, _sbss, _ebss;

// Core system clock speed, in Hertz.
uint32_t core_clock_hz;
// 'tick' variable which is incremented every ms by the SysTick.
volatile uint32_t systick;
// Systime to advance to the next 'substep' or 'cycle'.
uint32_t next_cyc;
// Systime to advance to the next lighting pattern.
uint32_t next_pat;

#endif
