#ifndef _VVC_MAIN_H
#define _VVC_MAIN_H

#include <stdint.h>
#include <string.h>

#include "stm32f1xx.h"

#include "patterns.h"

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
