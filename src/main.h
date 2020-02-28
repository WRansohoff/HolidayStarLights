#ifndef _VVC_MAIN_H
#define _VVC_MAIN_H

// Standard library includes.
#include <stdint.h>
#include <string.h>

// Device header files.
#ifdef VVC_STM32
  #include "stm32f1xx.h"
#elif  VVC_GD32V
  #include "gd32vf103.h"
  #include "n200_func.h"
  #include "riscv_encoding.h"
#endif

// Project includes.
#include "patterns.h"

// Platform-dependent values.
#ifdef VVC_STM32
  #define CFG_FLASH_LATENCY ( 0x1 << FLASH_ACR_LATENCY_Pos )
  #define CFG_PLLMULL       ( RCC_CFGR_PLLMULL6 )
  #define CFG_SYSCLK        ( 48000000 )
  #define CFG_SPI_BR_PSC    ( 0x2 << SPI_CR1_BR_Pos )
#elif  VVC_GD32V
  #define CFG_FLASH_LATENCY ( 0x2 << FLASH_ACR_LATENCY_Pos )
  #define CFG_PLLMULL       ( RCC_CFGR_PLLMULL12 )
  #define CFG_SYSCLK        ( 96000000 )
  #define CFG_SPI_BR_PSC    ( 0x3 << SPI_CR1_BR_Pos )
#endif

// Array of star structs.
star_t stars[ NUM_STARS ];

// LED colors buffer. (GRB*8)
// This is sort of a waste of RAM, but it makes DMA easier.
// You could use less memory by setting a faster SPI speed and
// stuffing more than one color bit into each byte, but this is easy.
#define NUM_COLOR_BYTES ( NUM_STARS * STAR_LEDS * 3 )
#define NUM_COLOR_BITS  ( NUM_COLOR_BYTES * 8 )
// (Add 64 cycles to hold a latching sequence)
#define COLOR_ARRAY_LEN ( NUM_COLOR_BITS + 64 )
// Memory buffer holding the LED colors. The DMA channel will use
// this buffer as its 'source address' for data to send over SPI.
uint8_t colors[ COLOR_ARRAY_LEN ];

// Pre-defined memory locations for program initialization.
extern uint32_t _sidata, _sdata, _edata, _sbss, _ebss;

// Core system clock speed, in Hertz.
uint32_t core_clock_hz;
// 'tick' variable which is incremented every ms by the SysTick.
volatile uint32_t systick;

#endif
