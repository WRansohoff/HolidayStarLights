#ifndef _VVC_MAIN_H
#define _VVC_MAIN_H

#include <stdint.h>
#include <string.h>

#include "stm32f1xx.h"

// SPI timing values for color bits. (One SPI byte = one color bit)
#define WS2812_ON  ( 0xFC )
#define WS2812_OFF ( 0xC0 )

// Number of LEDs per star.
#define STAR_LEDS ( 4 )
// Number of stars in the string.
#define NUM_STARS ( 1 )
// LED colors buffer. (GRB*8)
// This is sort of a waste of RAM, but it makes DMA easier.
// You could use less memory by setting a faster SPI speed and
// stuffing more than one color bit into each byte, but this is easy.
uint8_t colors[ NUM_STARS * STAR_LEDS * 24 ];

// Pre-defined memory locations for program initialization.
extern uint32_t _sidata, _sdata, _edata, _sbss, _ebss;

uint32_t core_clock_hz;
volatile uint32_t systick;

#endif
