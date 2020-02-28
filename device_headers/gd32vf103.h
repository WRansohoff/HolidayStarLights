#ifndef GD32VF103xB_PERIPHS
#define GD32VF103xB_PERIPHS
/*
 * GD32VF103CB device header file.
 * This is written to be as compatible as possible with STM32F1 code,
 * and differences between these chips' peripherals are commented.
 * Most peripherals are nearly identical, but there are plenty of
 * small differences. Many of the changes seem to revolve around the
 * fact that higher core clock speeds are supported (108MHz vs 72MHz).
 *
 * Comments about STM32F103 chips are based on the STM32F103xB
 * reference manual, so they may not be comprehensive.
 *
 * These definitions are still very incomplete.
 */
#include "riscv_encoding.h"
#include <stdint.h>

/**
 * Compatibility definitions and CPU-specific values.
 */
#define __MPU_PRESENT          ( 0 )
#define __NVIC_PRIO_BITS       ( 4 )
#define __Vendor_SysTickConfig ( 0 )
extern volatile uint32_t SystemCoreClock;

/**
 * Interrupt handler enumeration.
 */
typedef enum IRQn {
  CLIC_INT_RESERVED = 0,
  CLIC_INT_SFT = 3,
  CLIC_INT_TMR = 7,
  CLIC_INT_BWEI = 17,
  CLIC_INT_PMOVI = 18,
  WWDG_IRQn = 19,
  PVD_IRQn = 20,
  TAMPER_IRQn = 21,
  RTC_IRQn = 22,
  FLASH_IRQn = 23,
  RCC_IRQn = 24,
  EXTI0_IRQn = 25,
  EXTI1_IRQn = 26,
  EXTI2_IRQn = 27,
  EXTI3_IRQn = 28,
  EXTI4_IRQn = 29,
  DMA1_Channel1_IRQn = 30,
  DMA1_Channel2_IRQn = 31,
  DMA1_Channel3_IRQn = 32,
  DMA1_Channel4_IRQn = 33,
  DMA1_Channel5_IRQn = 34,
  DMA1_Channel6_IRQn = 35,
  DMA1_Channel7_IRQn = 36,
  ADC1_2_IRQn = 37,
  // These two interrupts aren't connected to the USB peripheral
  // on GD32 chips, but names are maintained for STM32F1 compatibility.
  USB_HP_CAN1_TX_IRQn = 38,
  CAN1_TX_IRQn = 38,
  USB_LP_CAN1_RX0_IRQn = 39,
  CAN1_RX0_IRQn = 39,
  CAN1_RX1_IRQn = 40,
  CAN1_SCE_IRQn = 41,
  EXTI9_5_IRQn = 42,
  TIM1_BRK_IRQn = 43,
  TIM1_UP_IRQn = 44,
  TIM1_TRG_COM_IRQn = 45,
  TIM1_CC_IRQn = 46,
  TIM2_IRQn = 47,
  TIM3_IRQn = 48,
  TIM4_IRQn = 49,
  I2C1_EV_IRQn = 50,
  I2C1_ER_IRQn = 51,
  I2C2_EV_IRQn = 52,
  I2C2_ER_IRQn = 53,
  SPI1_IRQn = 54,
  SPI2_IRQn = 55,
  USART1_IRQn = 56,
  USART2_IRQn = 57,
  USART3_IRQn = 58,
  EXTI15_10_IRQn = 59,
  RTC_Alarm_IRQn = 60,
  USBWakeUp_IRQn = 61,
  // Extra interrupts not available on STM32F103xB chips:
  // (Numbered peripherals are still 1-indexed)
  EXMC_IRQn = 67,
  TIM5_IRQn = 69,
  SPI3_IRQn = 70,
  UART4_IRQn = 71,
  UART5_IRQn = 72,
  TIM6_IRQn = 73,
  TIM7_IRQn = 74,
  DMA2_Channel1_IRQn = 75,
  DMA2_Channel2_IRQn = 76,
  DMA2_Channel3_IRQn = 77,
  DMA2_Channel4_IRQn = 78,
  DMA2_Channel5_IRQn = 79,
  CAN2_TX_IRQn = 82,
  CAN2_RX0_IRQn = 83,
  CAN2_RX1_IRQn = 84,
  CAN2_SCE_IRQn = 85,
  USBFS_IRQn = 86,
  ECLIC_NUM_INTERRUPTS
} IRQn_Type;

/**
 * Flash peripheral registers.
 */
typedef struct
{
  volatile uint32_t ACR;
  volatile uint32_t KEYR;
  volatile uint32_t OPTKEYR;
  volatile uint32_t SR;
  volatile uint32_t CR;
  volatile uint32_t AR;
  volatile uint32_t RESERVED;
  volatile uint32_t OBR;
  volatile uint32_t WRPR;
  // Note: Product ID register is not included in STM32F103xB docs.
  volatile uint32_t PID;
} FLASH_TypeDef;

/**
 * TODO: Power management peripheral registers.
 */

/**
 * TODO: Backup registers peripheral registers.
 */

/**
 * RCC / RCU peripheral registers.
 */
typedef struct
{
  volatile uint32_t CR;
  volatile uint32_t CFGR;
  volatile uint32_t CIR;
  volatile uint32_t APB2RSTR;
  volatile uint32_t APB1RSTR;
  volatile uint32_t AHBENR;
  volatile uint32_t APB2ENR;
  volatile uint32_t APB1ENR;
  volatile uint32_t BDCR;
  /* The following registers differ from STM32F103xB: */
  volatile uint32_t RSTSCK;
  volatile uint32_t AHBRSTR;
  volatile uint32_t CFGR1;
  volatile uint32_t DSV;
} RCC_TypeDef;

/**
 * TODO: EXTI peripheral registers.
 */

/**
 * GPIO peripheral registers.
 */
typedef struct
{
  volatile uint32_t CRL;
  volatile uint32_t CRH;
  volatile uint32_t IDR;
  volatile uint32_t ODR;
  volatile uint32_t BSRR;
  volatile uint32_t BRR;
  volatile uint32_t LCKR;
} GPIO_TypeDef;

/**
 * AFIO peripheral registers.
 */
typedef struct
{
  volatile uint32_t EVCR;
  volatile uint32_t MAPR;
  volatile uint32_t EXTICR[ 4 ];
  uint32_t RESERVED0;
  volatile uint32_t MAPR2;
} AFIO_TypeDef;

/**
 * TODO: CRC peripheral registers.
 */

/**
 * DMA peripheral registers (global).
 */
typedef struct
{
  volatile uint32_t ISR;
  volatile uint32_t IFCR;
} DMA_TypeDef;

/**
 * DMA peripheral registers (per-channel).
 */
typedef struct
{
  volatile uint32_t CCR;
  volatile uint32_t CNDTR;
  volatile uint32_t CPAR;
  volatile uint32_t CMAR;
} DMA_Channel_TypeDef;

/**
 * TODO: Debugging peripheral registers.
 */

/**
 * TODO: ADC peripheral registers.
 */

/**
 * TODO: DAC peripheral registers.
 */

/**
 * TODO: Watchdog peripheral registers.
 */

/**
 * TODO: RTC peripheral registers.
 */

/**
 * TODO: Timer peripheral registers.
 */

/**
 * TODO: UART / USART peripheral registers.
 */

/**
 * TODO: I2C peripheral registers.
 */

/**
 * SPI / I2S peripheral registers.
 */
typedef struct
{
  volatile uint32_t CR1;
  volatile uint32_t CR2;
  volatile uint32_t SR;
  volatile uint32_t DR;
  volatile uint32_t CRCPR;
  volatile uint32_t RXCRCR;
  volatile uint32_t TXCRCR;
  volatile uint32_t I2SCFGR;
  volatile uint32_t I2SPR;
} SPI_TypeDef;

/**
 * TODO: FSMC / EXMC peripheral registers.
 */

/**
 * TODO: CAN peripheral registers.
 */

/**
 * TODO: USBFS peripheral registers.
 */

/**
 * TODO: Option byte registers.
 */

/* Important global memory addresses. */
#define FLASH_BASE      ( 0x08000000 )
#define FLASH_BANK1_END ( 0x0801FFFF )
#define SRAM_BASE       ( 0x20000000 )
#define PERIPH_BASE     ( 0x40000000 )
// I don't think that the Nuclei N200 RISC-V CPUs support bit-banding,
// so there are no `SRAM_BB_BASE` or `PERIPH_BB_BASE` definitions.
#define APB1PERIPH_BASE ( 0x40000000 )
#define APB2PERIPH_BASE ( 0x40010000 )
#define AHBPERIPH_BASE  ( 0x40020000 )

/* Global register block address definitions. */
#define RCC           ( ( RCC_TypeDef * )         0x40021000 )
#define GPIOA         ( ( GPIO_TypeDef * )        0x40010800 )
#define GPIOB         ( ( GPIO_TypeDef * )        0x40010C00 )
#define GPIOC         ( ( GPIO_TypeDef * )        0x40011000 )
#define GPIOD         ( ( GPIO_TypeDef * )        0x40011400 )
#define GPIOE         ( ( GPIO_TypeDef * )        0x40011800 )
#define AFIO          ( ( AFIO_TypeDef * )        0x40010000 )
#define DMA1          ( ( DMA_TypeDef * )         0x40020000 )
#define DMA2          ( ( DMA_TypeDef * )         0x40020400 )
#define DMA1_Channel1 ( ( DMA_Channel_TypeDef * ) 0x40020008 )
#define DMA1_Channel2 ( ( DMA_Channel_TypeDef * ) 0x4002001C )
#define DMA1_Channel3 ( ( DMA_Channel_TypeDef * ) 0x40020030 )
#define DMA1_Channel4 ( ( DMA_Channel_TypeDef * ) 0x40020044 )
#define DMA1_Channel5 ( ( DMA_Channel_TypeDef * ) 0x40020058 )
#define DMA1_Channel6 ( ( DMA_Channel_TypeDef * ) 0x4002006C )
#define DMA1_Channel7 ( ( DMA_Channel_TypeDef * ) 0x40020080 )
#define DMA2_Channel1 ( ( DMA_Channel_TypeDef * ) 0x40020408 )
#define DMA2_Channel2 ( ( DMA_Channel_TypeDef * ) 0x4002041C )
#define DMA2_Channel3 ( ( DMA_Channel_TypeDef * ) 0x40020430 )
#define DMA2_Channel4 ( ( DMA_Channel_TypeDef * ) 0x40020444 )
#define DMA2_Channel5 ( ( DMA_Channel_TypeDef * ) 0x40020458 )
#define FLASH         ( ( FLASH_TypeDef * )       0x40022000 )
#define SPI1          ( ( SPI_TypeDef * )         0x40013000 )
#define SPI2          ( ( SPI_TypeDef * )         0x40003800 )
#define SPI3          ( ( SPI_TypeDef * )         0x40003C00 )

/* Flash register bit definitions. */
/* ACR */
#define FLASH_ACR_LATENCY_Pos ( 0 )
#define FLASH_ACR_LATENCY_Msk ( 0x7 << FLASH_ACR_LATENCY_Pos )
#define FLASH_ACR_LATENCY     ( FLASH_ACR_LATENCY_Msk )
// Note: The GD32VF103xB seems to lack some of the Flash optimizations
// which the STM32F103xB comes with. There is no half-cycle access
// or prefetch buffer in the GD32VF103xB.
#define FLASH_ACR_HLFCYA      ( 0 )
#define FLASH_ACR_PRFTBE      ( 0 )
#define FLASH_ACR_PRFTBS      ( 0 )

/* RCC register bit definitions. */
/* CR */
#define RCC_CR_HSION_Pos           ( 0 )
#define RCC_CR_HSION_Msk           ( 0x1 << RCC_CR_HSION_Pos )
#define RCC_CR_HSION               ( RCC_CR_HSION_Msk )
#define RCC_CR_HSIRDY_Pos          ( 1 )
#define RCC_CR_HSIRDY_Msk          ( 0x1 << RCC_CR_HSIRDY_Pos )
#define RCC_CR_HSIRDY              ( RCC_CR_HSIRDY_Msk )
#define RCC_CR_HSITRIM_Pos         ( 3 )
#define RCC_CR_HSITRIM_Msk         ( 0x1F << RCC_CR_HSITRIM_Pos )
#define RCC_CR_HSITRIM             ( RCC_CR_HSITRIM_Msk )
#define RCC_CR_HSICAL_Pos          ( 8 )
#define RCC_CR_HSICAL_Msk          ( 0xFF << RCC_CR_HSICAL_Pos )
#define RCC_CR_HSICAL              ( RCC_CR_HSICAL_Msk )
#define RCC_CR_HSEON_Pos           ( 16 )
#define RCC_CR_HSEON_Msk           ( 0x1 << RCC_CR_HSEON_Pos )
#define RCC_CR_HSEON               ( RCC_CR_HSEON_Msk )
#define RCC_CR_HSERDY_Pos          ( 17 )
#define RCC_CR_HSERDY_Msk          ( 0x1 << RCC_CR_HSERDY_Pos )
#define RCC_CR_HSERDY              ( RCC_CR_HSERDY_Msk )
#define RCC_CR_HSEBYP_Pos          ( 18 )
#define RCC_CR_HSEBYP_Msk          ( 0x1 << RCC_CR_HSEBYP_Pos )
#define RCC_CR_HSEBYP              ( RCC_CR_HSEBYP_Msk )
#define RCC_CR_CSSON_Pos           ( 19 )
#define RCC_CR_CSSON_Msk           ( 0x1 << RCC_CR_CSSON_Pos )
#define RCC_CR_CSSON               ( RCC_CR_CSSON_Msk )
#define RCC_CR_PLLON_Pos           ( 24 )
#define RCC_CR_PLLON_Msk           ( 0x1 << RCC_CR_PLLON_Pos )
#define RCC_CR_PLLON               ( RCC_CR_PLLON_Msk )
#define RCC_CR_PLLRDY_Pos          ( 25 )
#define RCC_CR_PLLRDY_Msk          ( 0x1 << RCC_CR_PLLRDY_Pos )
#define RCC_CR_PLLRDY              ( RCC_CR_PLLRDY_Msk )
// Note: PLL1 and PLL2 are not present on STM32F103xB chips.
#define RCC_CR_PLL1ON_Pos          ( 26 )
#define RCC_CR_PLL1ON_Msk          ( 0x1 << RCC_CR_PLL1ON_Pos )
#define RCC_CR_PLL1ON              ( RCC_CR_PLL1ON_Msk )
#define RCC_CR_PLL1RDY_Pos         ( 27 )
#define RCC_CR_PLL1RDY_Msk         ( 0x1 << RCC_CR_PLL1RDY_Pos )
#define RCC_CR_PLL1RDY             ( RCC_CR_PLL1RDY_Msk )
#define RCC_CR_PLL2ON_Pos          ( 28 )
#define RCC_CR_PLL2ON_Msk          ( 0x1 << RCC_CR_PLL2ON_Pos )
#define RCC_CR_PLL2ON              ( RCC_CR_PLL2ON_Msk )
#define RCC_CR_PLL2RDY_Pos         ( 29 )
#define RCC_CR_PLL2RDY_Msk         ( 0x1 << RCC_CR_PLL2RDY_Pos )
#define RCC_CR_PLL2RDY             ( RCC_CR_PLL2RDY_Msk )
/* CFGR */
#define RCC_CFGR_SW_Pos            ( 0 )
#define RCC_CFGR_SW_Msk            ( 0x3 << RCC_CFGR_SW_Pos )
#define RCC_CFGR_SW                ( RCC_CFGR_SW_Msk )
#define RCC_CFGR_SW_HSI            ( 0x0 << RCC_CFGR_SW_Pos )
#define RCC_CFGR_SW_HSE            ( 0x1 << RCC_CFGR_SW_Pos )
#define RCC_CFGR_SW_PLL            ( 0x2 << RCC_CFGR_SW_Pos )
#define RCC_CFGR_SWS_Pos           ( 2 )
#define RCC_CFGR_SWS_Msk           ( 0x3 << RCC_CFGR_SWS_Pos )
#define RCC_CFGR_SWS               ( RCC_CFGR_SWS_Msk )
#define RCC_CFGR_SWS_HSI           ( 0x0 << RCC_CFGR_SWS_Pos )
#define RCC_CFGR_SWS_HSE           ( 0x1 << RCC_CFGR_SWS_Pos )
#define RCC_CFGR_SWS_PLL           ( 0x2 << RCC_CFGR_SWS_Pos )
#define RCC_CFGR_HPRE_Pos          ( 4 )
#define RCC_CFGR_HPRE_Msk          ( 0xF << RCC_CFGR_HPRE_Pos )
#define RCC_CFGR_HPRE              ( RCC_CFGR_HPRE_Msk )
#define RCC_CFGR_HPRE_DIV1         ( 0x0 << RCC_CFGR_HPRE_Pos )
#define RCC_CFGR_HPRE_DIV2         ( 0x8 << RCC_CFGR_HPRE_Pos )
#define RCC_CFGR_HPRE_DIV4         ( 0x9 << RCC_CFGR_HPRE_Pos )
#define RCC_CFGR_HPRE_DIV8         ( 0xA << RCC_CFGR_HPRE_Pos )
#define RCC_CFGR_HPRE_DIV16        ( 0xB << RCC_CFGR_HPRE_Pos )
#define RCC_CFGR_HPRE_DIV64        ( 0xC << RCC_CFGR_HPRE_Pos )
#define RCC_CFGR_HPRE_DIV128       ( 0xD << RCC_CFGR_HPRE_Pos )
#define RCC_CFGR_HPRE_DIV256       ( 0xE << RCC_CFGR_HPRE_Pos )
#define RCC_CFGR_HPRE_DIV512       ( 0xF << RCC_CFGR_HPRE_Pos )
#define RCC_CFGR_PPRE1_Pos         ( 8 )
#define RCC_CFGR_PPRE1_Msk         ( 0x7 << RCC_CFGR_PPRE1_Pos )
#define RCC_CFGR_PPRE1             ( RCC_CFGR_PPRE1_Msk )
#define RCC_CFGR_PPRE1_DIV1        ( 0x0 << RCC_CFGR_PPRE1_Pos )
#define RCC_CFGR_PPRE1_DIV2        ( 0x4 << RCC_CFGR_PPRE1_Pos )
#define RCC_CFGR_PPRE1_DIV4        ( 0x5 << RCC_CFGR_PPRE1_Pos )
#define RCC_CFGR_PPRE1_DIV8        ( 0x6 << RCC_CFGR_PPRE1_Pos )
#define RCC_CFGR_PPRE1_DIV16       ( 0x7 << RCC_CFGR_PPRE1_Pos )
#define RCC_CFGR_PPRE2_Pos         ( 11 )
#define RCC_CFGR_PPRE2_Msk         ( 0x7 << RCC_CFGR_PPRE2_Pos )
#define RCC_CFGR_PPRE2             ( RCC_CFGR_PPRE2_Msk )
#define RCC_CFGR_PPRE2_DIV1        ( 0x0 << RCC_CFGR_PPRE2_Pos )
#define RCC_CFGR_PPRE2_DIV2        ( 0x4 << RCC_CFGR_PPRE2_Pos )
#define RCC_CFGR_PPRE2_DIV4        ( 0x5 << RCC_CFGR_PPRE2_Pos )
#define RCC_CFGR_PPRE2_DIV8        ( 0x6 << RCC_CFGR_PPRE2_Pos )
#define RCC_CFGR_PPRE2_DIV16       ( 0x7 << RCC_CFGR_PPRE2_Pos )
#define RCC_CFGR_ADCPRE_Pos        ( 14 )
#define RCC_CFGR_ADCPRE_Msk        ( 0x3 << RCC_CFGR_ADCPRE_Pos )
#define RCC_CFGR_ADCPRE            ( RCC_CFGR_ADCPRE_Msk )
// Note: These divisions can change based on the ADCPRE2 bit below,
// which is not present in STM32F103xB chips.
#define RCC_CFGR_ADCPRE_DIV2       ( 0x0 << RCC_CFGR_ADCPRE_Pos )
#define RCC_CFGR_ADCPRE_DIV4       ( 0x1 << RCC_CFGR_ADCPRE_Pos )
#define RCC_CFGR_ADCPRE_DIV6       ( 0x2 << RCC_CFGR_ADCPRE_Pos )
#define RCC_CFGR_ADCPRE_DIV8       ( 0x3 << RCC_CFGR_ADCPRE_Pos )
// Note: There is an extra ADC prescaler bit which is not present on
// STM32F103xB chips, possibly to account for a 108MHz core freq?
#define RCC_CFGR_ADCPRE_DIV12      ( 0x10004000 )
#define RCC_CFGR_ADCPRE_DIV16      ( 0x1000C000 )
#define RCC_CFGR_PLLSRC_Pos        ( 16 )
#define RCC_CFGR_PLLSRC_Msk        ( 0x1 << RCC_CFGR_PLLSRC_Pos )
#define RCC_CFGR_PLLSRC            ( RCC_CFGR_PLLSRC_Msk )
#define RCC_CFGR_PLLXTPRE_Pos      ( 17 )
#define RCC_CFGR_PLLXTPRE_Msk      ( 0x1 << RCC_CFGR_PLLXTPRE_Pos )
#define RCC_CFGR_PLLXTPRE          ( RCC_CFGR_PLLXTPRE_Msk )
#define RCC_CFGR_PLLXTPRE_HSE      ( 0x0 << RCC_CFGR_PLLXTPRE_Pos )
#define RCC_CFGR_PLLXTPRE_HSE_DIV2 ( 0x1 << RCC_CFGR_PLLXTPRE_Pos )
#define RCC_CFGR_PLLMULL_Pos       ( 18 )
#define RCC_CFGR_PLLMULL_Msk       ( 0xF << RCC_CFGR_PLLMULL_Pos )
#define RCC_CFGR_PLLMULL           ( RCC_CFGR_PLLMULL_Msk )
#define RCC_CFGR_PLLMULL2          ( 0x0 << RCC_CFGR_PLLMULL_Pos )
#define RCC_CFGR_PLLMULL3          ( 0x1 << RCC_CFGR_PLLMULL_Pos )
#define RCC_CFGR_PLLMULL4          ( 0x2 << RCC_CFGR_PLLMULL_Pos )
#define RCC_CFGR_PLLMULL5          ( 0x3 << RCC_CFGR_PLLMULL_Pos )
#define RCC_CFGR_PLLMULL6          ( 0x4 << RCC_CFGR_PLLMULL_Pos )
#define RCC_CFGR_PLLMULL7          ( 0x5 << RCC_CFGR_PLLMULL_Pos )
#define RCC_CFGR_PLLMULL8          ( 0x6 << RCC_CFGR_PLLMULL_Pos )
#define RCC_CFGR_PLLMULL9          ( 0x7 << RCC_CFGR_PLLMULL_Pos )
#define RCC_CFGR_PLLMULL10         ( 0x8 << RCC_CFGR_PLLMULL_Pos )
#define RCC_CFGR_PLLMULL11         ( 0x9 << RCC_CFGR_PLLMULL_Pos )
#define RCC_CFGR_PLLMULL12         ( 0xA << RCC_CFGR_PLLMULL_Pos )
#define RCC_CFGR_PLLMULL13         ( 0xB << RCC_CFGR_PLLMULL_Pos )
#define RCC_CFGR_PLLMULL14         ( 0xC << RCC_CFGR_PLLMULL_Pos )
// Note: This is marked as "x6.5" instead of "x15" in the GD32VF103xB
// reference manual, but I'm pretty sure that's a typo.
#define RCC_CFGR_PLLMULL15         ( 0xD << RCC_CFGR_PLLMULL_Pos )
#define RCC_CFGR_PLLMULL16         ( 0xE << RCC_CFGR_PLLMULL_Pos )
// Note: There is an extra PLL multiplication bit which is not present
// on STM32F103xB chips, to allow for the higher 108MHz speed limit.
#define RCC_CFGR_PLLMULL17         ( 0x800 << RCC_CFGR_PLLMULL_Pos )
#define RCC_CFGR_PLLMULL18         ( 0x801 << RCC_CFGR_PLLMULL_Pos )
#define RCC_CFGR_PLLMULL19         ( 0x802 << RCC_CFGR_PLLMULL_Pos )
#define RCC_CFGR_PLLMULL20         ( 0x803 << RCC_CFGR_PLLMULL_Pos )
#define RCC_CFGR_PLLMULL21         ( 0x804 << RCC_CFGR_PLLMULL_Pos )
#define RCC_CFGR_PLLMULL22         ( 0x805 << RCC_CFGR_PLLMULL_Pos )
#define RCC_CFGR_PLLMULL23         ( 0x806 << RCC_CFGR_PLLMULL_Pos )
#define RCC_CFGR_PLLMULL24         ( 0x807 << RCC_CFGR_PLLMULL_Pos )
#define RCC_CFGR_PLLMULL25         ( 0x808 << RCC_CFGR_PLLMULL_Pos )
#define RCC_CFGR_PLLMULL26         ( 0x809 << RCC_CFGR_PLLMULL_Pos )
#define RCC_CFGR_PLLMULL27         ( 0x80A << RCC_CFGR_PLLMULL_Pos )
#define RCC_CFGR_PLLMULL28         ( 0x80B << RCC_CFGR_PLLMULL_Pos )
#define RCC_CFGR_PLLMULL29         ( 0x80C << RCC_CFGR_PLLMULL_Pos )
#define RCC_CFGR_PLLMULL30         ( 0x80D << RCC_CFGR_PLLMULL_Pos )
#define RCC_CFGR_PLLMULL31         ( 0x80E << RCC_CFGR_PLLMULL_Pos )
#define RCC_CFGR_PLLMULL32         ( 0x80F << RCC_CFGR_PLLMULL_Pos )
// Note: The `USBPRE` setting spans two bits instead of one, compared
// to the STM32F103xB. Again, this allows 108MHz clock speeds.
#define RCC_CFGR_USBPRE_Pos        ( 22 )
#define RCC_CFGR_USBPRE_Msk        ( 0x3 << RCC_CFGR_USBPRE_Pos )
#define RCC_CFGR_USBPRE            ( RCC_CFGR_USBPRE_Msk )
#define RCC_CFGR_USBPRE_DIV1_5     ( 0x0 << RCC_CFGR_USBPRE_Pos )
#define RCC_CFGR_USBPRE_DIV1       ( 0x1 << RCC_CFGR_USBPRE_Pos )
#define RCC_CFGR_USBPRE_DIV2_5     ( 0x2 << RCC_CFGR_USBPRE_Pos )
#define RCC_CFGR_USBPRE_DIV2       ( 0x3 << RCC_CFGR_USBPRE_Pos )
#define RCC_CFGR_MCO_Pos           ( 24 )
#define RCC_CFGR_MCO_Msk           ( 0xF << RCC_CFGR_MCO_Pos )
#define RCC_CFGR_MCO               ( RCC_CFGR_MCO_Msk )
#define RCC_CFGR_MCO_NOCLOCK       ( 0x0 << RCC_CFGR_MCO_Pos )
#define RCC_CFGR_MCO_SYSCLK        ( 0x4 << RCC_CFGR_MCO_Pos )
#define RCC_CFGR_MCO_HSI           ( 0x5 << RCC_CFGR_MCO_Pos )
#define RCC_CFGR_MCO_HSE           ( 0x6 << RCC_CFGR_MCO_Pos )
#define RCC_CFGR_MCO_PLLCLK_DIV2   ( 0x7 << RCC_CFGR_MCO_Pos )
// Note: The following MCO configurations don't seeem to be
// available on STM32F103xB chips.
#define RCC_CFGR_MCO_PLL1CLK       ( 0x8 << RCC_CFGR_MCO_Pos )
#define RCC_CFGR_MCO_PLL2CLK_DIV2  ( 0x9 << RCC_CFGR_MCO_Pos )
#define RCC_CFGR_MCO_EXTI          ( 0xA << RCC_CFGR_MCO_Pos )
#define RCC_CFGR_MCO_PLL2CLK       ( 0xB << RCC_CFGR_MCO_Pos )
/* APB2RSTR */
#define RCC_APB2RSTR_AFIORST_Pos   ( 0 )
#define RCC_APB2RSTR_AFIORST_Msk   ( 0x1 << RCC_APB2RSTR_AFIORST_Pos )
#define RCC_APB2RSTR_AFIORST       ( RCC_APB2RSTR_AFIORST_Msk )
#define RCC_APB2RSTR_IOPARST_Pos   ( 2 )
#define RCC_APB2RSTR_IOPARST_Msk   ( 0x1 << RCC_APB2RSTR_IOPARST_Pos )
#define RCC_APB2RSTR_IOPARST       ( RCC_APB2RSTR_IOPARST_Msk )
#define RCC_APB2RSTR_IOPBRST_Pos   ( 3 )
#define RCC_APB2RSTR_IOPBRST_Msk   ( 0x1 << RCC_APB2RSTR_IOPBRST_Pos )
#define RCC_APB2RSTR_IOPBRST       ( RCC_APB2RSTR_IOPBRST_Msk )
#define RCC_APB2RSTR_IOPCRST_Pos   ( 4 )
#define RCC_APB2RSTR_IOPCRST_Msk   ( 0x1 << RCC_APB2RSTR_IOPCRST_Pos )
#define RCC_APB2RSTR_IOPCRST       ( RCC_APB2RSTR_IOPCRST_Msk )
#define RCC_APB2RSTR_IOPDRST_Pos   ( 5 )
#define RCC_APB2RSTR_IOPDRST_Msk   ( 0x1 << RCC_APB2RSTR_IOPDRST_Pos )
#define RCC_APB2RSTR_IOPDRST       ( RCC_APB2RSTR_IOPDRST_Msk )
#define RCC_APB2RSTR_IOPERST_Pos   ( 6 )
#define RCC_APB2RSTR_IOPERST_Msk   ( 0x1 << RCC_APB2RSTR_IOPERST_Pos )
#define RCC_APB2RSTR_IOPERST       ( RCC_APB2RSTR_IOPERST_Msk )
#define RCC_APB2RSTR_ADC1RST_Pos   ( 9 )
#define RCC_APB2RSTR_ADC1RST_Msk   ( 0x1 << RCC_APB2RSTR_ADC1RST_Pos )
#define RCC_APB2RSTR_ADC1RST       ( RCC_APB2RSTR_ADC1RST_Msk )
#define RCC_APB2RSTR_ADC2RST_Pos   ( 10 )
#define RCC_APB2RSTR_ADC2RST_Msk   ( 0x1 << RCC_APB2RSTR_ADC2RST_Pos )
#define RCC_APB2RSTR_ADC2RST       ( RCC_APB2RSTR_ADC2RST_Msk )
#define RCC_APB2RSTR_TIM1RST_Pos   ( 11 )
#define RCC_APB2RSTR_TIM1RST_Msk   ( 0x1 << RCC_APB2RSTR_TIM1RST_Pos )
#define RCC_APB2RSTR_TIM1RST       ( RCC_APB2RSTR_TIM1RST_Msk )
#define RCC_APB2RSTR_SPI1RST_Pos   ( 12 )
#define RCC_APB2RSTR_SPI1RST_Msk   ( 0x1 << RCC_APB2RSTR_SPI1RST_Pos )
#define RCC_APB2RSTR_SPI1RST       ( RCC_APB2RSTR_SPI1RST_Msk )
#define RCC_APB2RSTR_USART1RST_Pos ( 14 )
#define RCC_APB2RSTR_USART1RST_Msk ( 0x1 << RCC_APB2RSTR_USART1RST_Pos )
#define RCC_APB2RSTR_USART1RST     ( RCC_APB2RSTR_USART1RST_Msk )
/* AHBENR */
#define RCC_AHBENR_DMA1EN_Pos    ( 0 )
#define RCC_AHBENR_DMA1EN_Msk    ( 0x1 << RCC_AHBENR_DMA1EN_Pos )
#define RCC_AHBENR_DMA1EN        ( RCC_AHBENR_DMA1EN_Msk )
// Note: DMA2 not present on STM32F103xB chips.
#define RCC_AHBENR_DMA2EN_Pos    ( 1 )
#define RCC_AHBENR_DMA2EN_Msk    ( 0x1 << RCC_AHBENR_DMA2EN_Pos )
#define RCC_AHBENR_DMA2EN        ( RCC_AHBENR_DMA2EN_Msk )
#define RCC_AHBENR_SRAMEN_Pos    ( 2 )
#define RCC_AHBENR_SRAMEN_Msk    ( 0x1 << RCC_AHBENR_SRAMEN_Pos )
#define RCC_AHBENR_SRAMEN        ( RCC_AHBENR_SRAMEN_Msk )
#define RCC_AHBENR_FLITFEN_Pos   ( 4 )
#define RCC_AHBENR_FLITFEN_Msk   ( 0x1 << RCC_AHBENR_FLITFEN_Pos )
#define RCC_AHBENR_FLITFEN       ( RCC_AHBENR_FLITFEN_Msk )
#define RCC_AHBENR_CRCEN_Pos     ( 6 )
#define RCC_AHBENR_CRCEN_Msk     ( 0x1 << RCC_AHBENR_CRCENEN_Pos )
#define RCC_AHBENR_CRCEN         ( RCC_AHBENR_CRCENEN_Msk )
// Note: FSMC / EXMC enable bit not present on STM32F103xB chips.
#define RCC_AHBENR_EXMCEN_Pos    ( 8 )
#define RCC_AHBENR_EXMCEN_Msk    ( 0x1 << RCC_AHBENR_EXMCEN_Pos )
#define RCC_AHBENR_EXMCEN        ( RCC_AHBENR_EXMCEN_Msk )
// Note: USB handled differently on STM32F103xB chips.
#define RCC_AHBENR_USBFSEN_Pos   ( 12 )
#define RCC_AHBENR_USBFSEN_Msk   ( 0x1 << RCC_AHBENR_USBFSEN_Pos )
#define RCC_AHBENR_USBFSEN       ( RCC_AHBENR_USBFSEN_Msk )
/* APB2ENR */
#define RCC_APB2ENR_AFIOEN_Pos   ( 0 )
#define RCC_APB2ENR_AFIOEN_Msk   ( 0x1 << RCC_APB2ENR_AFIOEN_Pos )
#define RCC_APB2ENR_AFIOEN       ( RCC_APB2ENR_AFIOEN_Msk )
#define RCC_APB2ENR_IOPAEN_Pos   ( 2 )
#define RCC_APB2ENR_IOPAEN_Msk   ( 0x1 << RCC_APB2ENR_IOPAEN_Pos )
#define RCC_APB2ENR_IOPAEN       ( RCC_APB2ENR_IOPAEN_Msk )
#define RCC_APB2ENR_IOPBEN_Pos   ( 3 )
#define RCC_APB2ENR_IOPBEN_Msk   ( 0x1 << RCC_APB2ENR_IOPBEN_Pos )
#define RCC_APB2ENR_IOPBEN       ( RCC_APB2ENR_IOPBEN_Msk )
#define RCC_APB2ENR_IOPCEN_Pos   ( 4 )
#define RCC_APB2ENR_IOPCEN_Msk   ( 0x1 << RCC_APB2ENR_IOPCEN_Pos )
#define RCC_APB2ENR_IOPCEN       ( RCC_APB2ENR_IOPCEN_Msk )
#define RCC_APB2ENR_IOPDEN_Pos   ( 5 )
#define RCC_APB2ENR_IOPDEN_Msk   ( 0x1 << RCC_APB2ENR_IOPDEN_Pos )
#define RCC_APB2ENR_IOPDEN       ( RCC_APB2ENR_IOPDEN_Msk )
#define RCC_APB2ENR_IOPEEN_Pos   ( 6 )
#define RCC_APB2ENR_IOPEEN_Msk   ( 0x1 << RCC_APB2ENR_IOPEEN_Pos )
#define RCC_APB2ENR_IOPEEN       ( RCC_APB2ENR_IOPEEN_Msk )
#define RCC_APB2ENR_ADC1EN_Pos   ( 9 )
#define RCC_APB2ENR_ADC1EN_Msk   ( 0x1 << RCC_APB2ENR_ADC1EN_Pos )
#define RCC_APB2ENR_ADC1EN       ( RCC_APB2ENR_ADC1EN_Msk )
#define RCC_APB2ENR_ADC2EN_Pos   ( 10 )
#define RCC_APB2ENR_ADC2EN_Msk   ( 0x1 << RCC_APB2ENR_ADC2EN_Pos )
#define RCC_APB2ENR_ADC2EN       ( RCC_APB2ENR_ADC2EN_Msk )
#define RCC_APB2ENR_TIM1EN_Pos   ( 11 )
#define RCC_APB2ENR_TIM1EN_Msk   ( 0x1 << RCC_APB2ENR_TIM1EN_Pos )
#define RCC_APB2ENR_TIM1EN       ( RCC_APB2ENR_TIM1EN_Msk )
#define RCC_APB2ENR_SPI1EN_Pos   ( 12 )
#define RCC_APB2ENR_SPI1EN_Msk   ( 0x1 << RCC_APB2ENR_SPI1EN_Pos )
#define RCC_APB2ENR_SPI1EN       ( RCC_APB2ENR_SPI1EN_Msk )
#define RCC_APB2ENR_USART1EN_Pos ( 14 )
#define RCC_APB2ENR_USART1EN_Msk ( 0x1 << RCC_APB2ENR_USART1EN_Pos )
#define RCC_APB2ENR_USART1EN     ( RCC_APB2ENR_USART1EN_Msk )

/* AFIO register bit definitions. */
/* EVCR */
#define AFIO_EVCR_PIN_Pos   ( 0 )
#define AFIO_EVCR_PIN_Msk   ( 0xF << AFIO_EVCR_PIN_Pos )
#define AFIO_EVCR_PIN       ( AFIO_EVCR_PIN_Msk )
#define AFIO_EVCR_PIN_PX0   ( 0x0 << AFIO_EVCR_PIN_Pos )
#define AFIO_EVCR_PIN_PX1   ( 0x1 << AFIO_EVCR_PIN_Pos )
#define AFIO_EVCR_PIN_PX2   ( 0x2 << AFIO_EVCR_PIN_Pos )
#define AFIO_EVCR_PIN_PX3   ( 0x3 << AFIO_EVCR_PIN_Pos )
#define AFIO_EVCR_PIN_PX4   ( 0x4 << AFIO_EVCR_PIN_Pos )
#define AFIO_EVCR_PIN_PX5   ( 0x5 << AFIO_EVCR_PIN_Pos )
#define AFIO_EVCR_PIN_PX6   ( 0x6 << AFIO_EVCR_PIN_Pos )
#define AFIO_EVCR_PIN_PX7   ( 0x7 << AFIO_EVCR_PIN_Pos )
#define AFIO_EVCR_PIN_PX8   ( 0x8 << AFIO_EVCR_PIN_Pos )
#define AFIO_EVCR_PIN_PX9   ( 0x9 << AFIO_EVCR_PIN_Pos )
#define AFIO_EVCR_PIN_PX10  ( 0xA << AFIO_EVCR_PIN_Pos )
#define AFIO_EVCR_PIN_PX11  ( 0xB << AFIO_EVCR_PIN_Pos )
#define AFIO_EVCR_PIN_PX12  ( 0xC << AFIO_EVCR_PIN_Pos )
#define AFIO_EVCR_PIN_PX13  ( 0xD << AFIO_EVCR_PIN_Pos )
#define AFIO_EVCR_PIN_PX14  ( 0xE << AFIO_EVCR_PIN_Pos )
#define AFIO_EVCR_PIN_PX15  ( 0xF << AFIO_EVCR_PIN_Pos )
#define AFIO_EVCR_PORT_Pos  ( 4 )
#define AFIO_EVCR_PORT_Msk  ( 0x7 << AFIO_EVCR_PORT_Pos )
#define AFIO_EVCR_PORT      ( AFIO_EVCR_PORT_Msk )
#define AFIO_EVCR_PORT_PA   ( 0x0 << AFIO_EVCR_PORT_Pos )
#define AFIO_EVCR_PORT_PB   ( 0x1 << AFIO_EVCR_PORT_Pos )
#define AFIO_EVCR_PORT_PC   ( 0x2 << AFIO_EVCR_PORT_Pos )
#define AFIO_EVCR_PORT_PD   ( 0x3 << AFIO_EVCR_PORT_Pos )
#define AFIO_EVCR_PORT_PE   ( 0x4 << AFIO_EVCR_PORT_Pos )
#define AFIO_EVCR_EVOE_Pos  ( 7 )
#define AFIO_EVCR_EVOE_Msk  ( 0x1 << AFIO_EVCR_EVOE_Pos )
#define AFIO_EVCR_EVOE      ( AFIO_EVCR_EVOE_Msk )
/* MAPR */
#define AFIO_MAPR_SPI1_REMAP_Pos ( 0 )
#define AFIO_MAPR_SPI1_REMAP_Msk ( 0x1 << AFIO_MAPR_SPI1_REMAP_Pos )
#define AFIO_MAPR_SPI1_REMAP     ( AFIO_MAPR_SPI1_REMAP_Msk )
#define AFIO_MAPR_I2C1_REMAP_Pos ( 1 )
#define AFIO_MAPR_I2C1_REMAP_Msk ( 0x1 << AFIO_MAPR_I2C1_REMAP_Pos )
#define AFIO_MAPR_I2C1_REMAP     ( AFIO_MAPR_I2C1_REMAP_Msk )
#define AFIO_MAPR_USART1_REMAP_Pos ( 2 )
#define AFIO_MAPR_USART1_REMAP_Msk ( 0x1 << AFIO_MAPR_USART1_REMAP_Pos )
#define AFIO_MAPR_USART1_REMAP     ( AFIO_MAPR_USART1_REMAP_Msk )
#define AFIO_MAPR_USART2_REMAP_Pos ( 3 )
#define AFIO_MAPR_USART2_REMAP_Msk ( 0x1 << AFIO_MAPR_USART2_REMAP_Pos )
#define AFIO_MAPR_USART2_REMAP     ( AFIO_MAPR_USART2_REMAP_Msk )
#define AFIO_MAPR_USART3_REMAP_Pos ( 4 )
#define AFIO_MAPR_USART3_REMAP_Msk ( 0x3 << AFIO_MAPR_USART3_REMAP_Pos )
#define AFIO_MAPR_USART3_REMAP     ( AFIO_MAPR_USART3_REMAP_Msk )
#define AFIO_MAPR_USART3_REMAP_NOREMAP ( 0x0 << AFIO_MAPR_USART3_REMAP_Pos )
#define AFIO_MAPR_USART3_REMAP_PARTIALREMAP ( 0x1 << AFIO_MAPR_USART3_REMAP_Pos )
#define AFIO_MAPR_USART3_REMAP_FULLREMAP ( 0x3 << AFIO_MAPR_USART3_REMAP_Pos )
#define AFIO_MAPR_TIM1_REMAP_Pos ( 6 )
#define AFIO_MAPR_TIM1_REMAP_Msk ( 0x3 << AFIO_MAPR_TIM1_REMAP_Pos )
#define AFIO_MAPR_TIM1_REMAP     ( AFIO_MAPR_TIM1_REMAP_Msk )
#define AFIO_MAPR_TIM1_REMAP_NOREMAP ( 0x0 << AFIO_MAPR_TIM1_REMAP_Pos )
#define AFIO_MAPR_TIM1_REMAP_PARTIALREMAP ( 0x1 << AFIO_MAPR_TIM1_REMAP_Pos )
#define AFIO_MAPR_TIM1_REMAP_FULLREMAP ( 0x3 << AFIO_MAPR_TIM1_REMAP_Pos )
#define AFIO_MAPR_TIM2_REMAP_Pos ( 8 )
#define AFIO_MAPR_TIM2_REMAP_Msk ( 0x3 << AFIO_MAPR_TIM2_REMAP_Pos )
#define AFIO_MAPR_TIM2_REMAP     ( AFIO_MAPR_TIM2_REMAP_Msk )
#define AFIO_MAPR_TIM2_REMAP_NOREMAP ( 0x0 << AFIO_MAPR_TIM2_REMAP_Pos )
#define AFIO_MAPR_TIM2_REMAP_PARTIALREMAP1 ( 0x1 << AFIO_MAPR_TIM2_REMAP_Pos )
#define AFIO_MAPR_TIM2_REMAP_PARTIALREMAP2 ( 0x2 << AFIO_MAPR_TIM2_REMAP_Pos )
#define AFIO_MAPR_TIM2_REMAP_FULLREMAP ( 0x3 << AFIO_MAPR_TIM2_REMAP_Pos )
#define AFIO_MAPR_TIM3_REMAP_Pos ( 10 )
#define AFIO_MAPR_TIM3_REMAP_Msk ( 0x3 << AFIO_MAPR_TIM3_REMAP_Pos )
#define AFIO_MAPR_TIM3_REMAP     ( AFIO_MAPR_TIM3_REMAP_Msk )
#define AFIO_MAPR_TIM3_REMAP_NOREMAP ( 0x0 << AFIO_MAPR_TIM3_REMAP_Pos )
#define AFIO_MAPR_TIM3_REMAP_PARTIALREMAP ( 0x2 << AFIO_MAPR_TIM3_REMAP_Pos )
#define AFIO_MAPR_TIM3_REMAP_FULLREMAP ( 0x3 << AFIO_MAPR_TIM3_REMAP_Pos )
#define AFIO_MAPR_TIM4_REMAP_Pos ( 12 )
#define AFIO_MAPR_TIM4_REMAP_Msk ( 0x1 << AFIO_MAPR_TIM4_REMAP_Pos )
#define AFIO_MAPR_TIM4_REMAP     ( AFIO_MAPR_TIM4_REMAP_Msk )
// Note: STM32F103xB chips only have one CAN peripheral.
#define AFIO_MAPR_CAN1_REMAP_Pos ( 13 )
#define AFIO_MAPR_CAN1_REMAP_Msk ( 0x3 << AFIO_MAPR_CAN1_REMAP_Pos )
#define AFIO_MAPR_CAN1_REMAP     ( AFIO_MAPR_CAN1_REMAP_Msk )
#define AFIO_MAPR_CAN1_REMAP_REMAP1 ( 0x0 << AFIO_MAPR_CAN1_REMAP_Pos )
#define AFIO_MAPR_CAN1_REMAP_REMAP2 ( 0x2 << AFIO_MAPR_CAN1_REMAP_Pos )
#define AFIO_MAPR_CAN1_REMAP_REMAP3 ( 0x3 << AFIO_MAPR_CAN1_REMAP_Pos )
#define AFIO_MAPR_CAN_REMAP_Pos ( AFIO_MAPR_CAN1_REMAP_Pos )
#define AFIO_MAPR_CAN_REMAP_Msk ( AFIO_MAPR_CAN1_REMAP_Msk )
#define AFIO_MAPR_CAN_REMAP     ( AFIO_MAPR_CAN1_REMAP )
#define AFIO_MAPR_CAN_REMAP_REMAP1 ( AFIO_MAPR_CAN1_REMAP_REMAP1 )
#define AFIO_MAPR_CAN_REMAP_REMAP2 ( AFIO_MAPR_CAN1_REMAP_REMAP2 )
#define AFIO_MAPR_CAN_REMAP_REMAP3 ( AFIO_MAPR_CAN1_REMAP_REMAP3 )
#define AFIO_MAPR_PD01_REMAP_Pos ( 15 )
#define AFIO_MAPR_PD01_REMAP_Msk ( 0x1 << AFIO_MAPR_PD01_REMAP_Pos )
#define AFIO_MAPR_PD01_REMAP     ( AFIO_MAPR_PD01_REMAP_Msk )
// Note: TIM5CH3 remap is not present in STM32F103xB docs.
#define AFIO_MAPR_TIM5CH3_REMAP_Pos ( 16 )
#define AFIO_MAPR_TIM5CH3_REMAP_Msk ( 0x1 << AFIO_MAPR_TIM5CH3_REMAP_Pos )
#define AFIO_MAPR_TIM5CH3_REMAP     ( AFIO_MAPR_TIM5CH3_REMAP_Msk )
// Note: CAN2 is not present in STM32F103xB docs.
#define AFIO_MAPR_CAN2_REMAP_Pos ( 22 )
#define AFIO_MAPR_CAN2_REMAP_Msk ( 0x1 << AFIO_MAPR_CAN2_REMAP_Pos )
#define AFIO_MAPR_CAN2_REMAP     ( AFIO_MAPR_CAN2_REMAP_Msk )
#define AFIO_MAPR_SWJ_CFG_Pos ( 24 )
#define AFIO_MAPR_SWJ_CFG_Msk ( 0x7 << AFIO_MAPR_SWJ_CFG_REMAP_Pos )
#define AFIO_MAPR_SWJ_CFG     ( AFIO_MAPR_SWJ_CFG_REMAP_Msk )
#define AFIO_MAPR_SWJ_CFG_RESET ( 0x0 << AFIO_MAPR_SWJ_CFG_REMAP_Pos )
#define AFIO_MAPR_SWJ_CFG_NOJNTRST ( 0x1 << AFIO_MAPR_SWJ_CFG_REMAP_Pos )
#define AFIO_MAPR_SWJ_CFG_JTAGDISABLE ( 0x2 << AFIO_MAPR_SWJ_CFG_REMAP_Pos )
// Note: No 'AFIO_MAPR_SWJ_CFG_DISABLE' option in GD32VF103 chips.
#define AFIO_MAPR_SWJ_CFG_DISABLE ( AFIO_MAPR_SWJ_CFG_JTAGDISABLE )
// Note: SPI3 is not present in STM32F103xB docs.
#define AFIO_MAPR_SPI3_REMAP_Pos ( 28 )
#define AFIO_MAPR_SPI3_REMAP_Msk ( 0x1 << AFIO_MAPR_SPI3_REMAP_Pos )
#define AFIO_MAPR_SPI3_REMAP     ( AFIO_MAPR_SPI3_REMAP_Msk )
// Note: TIM1ITI1 remap is not present in STM32F103xB docs.
#define AFIO_MAPR_TIM1ITI1_REMAP_Pos ( 29 )
#define AFIO_MAPR_TIM1ITI1_REMAP_Msk ( 0x1 << AFIO_MAPR_TIM1ITI1_REMAP_Pos )
#define AFIO_MAPR_TIM1ITI1_REMAP     ( AFIO_MAPR_TIM1ITI1_REMAP_Msk )

/* GPIO register bit definitions. */
/* CRL */
#define GPIO_CRL_MODE_Pos   ( 0 )
#define GPIO_CRL_MODE_Msk   ( 0x33333333 << GPIO_CRL_MODE_Pos )
#define GPIO_CRL_MODE       ( GPIO_CRL_MODE_Msk )
#define GPIO_CRL_MODE0_Pos  ( 0 )
#define GPIO_CRL_MODE0_Msk  ( 0x3 << GPIO_CRL_MODE0_Pos )
#define GPIO_CRL_MODE0      ( GPIO_CRL_MODE0_Msk )
#define GPIO_CRL_MODE1_Pos  ( 4 )
#define GPIO_CRL_MODE1_Msk  ( 0x3 << GPIO_CRL_MODE1_Pos )
#define GPIO_CRL_MODE1      ( GPIO_CRL_MODE1_Msk )
#define GPIO_CRL_MODE2_Pos  ( 8 )
#define GPIO_CRL_MODE2_Msk  ( 0x3 << GPIO_CRL_MODE2_Pos )
#define GPIO_CRL_MODE2      ( GPIO_CRL_MODE2_Msk )
#define GPIO_CRL_MODE3_Pos  ( 12 )
#define GPIO_CRL_MODE3_Msk  ( 0x3 << GPIO_CRL_MODE3_Pos )
#define GPIO_CRL_MODE3      ( GPIO_CRL_MODE3_Msk )
#define GPIO_CRL_MODE4_Pos  ( 16 )
#define GPIO_CRL_MODE4_Msk  ( 0x3 << GPIO_CRL_MODE4_Pos )
#define GPIO_CRL_MODE4      ( GPIO_CRL_MODE4_Msk )
#define GPIO_CRL_MODE5_Pos  ( 20 )
#define GPIO_CRL_MODE5_Msk  ( 0x3 << GPIO_CRL_MODE5_Pos )
#define GPIO_CRL_MODE5      ( GPIO_CRL_MODE5_Msk )
#define GPIO_CRL_MODE6_Pos  ( 24 )
#define GPIO_CRL_MODE6_Msk  ( 0x3 << GPIO_CRL_MODE6_Pos )
#define GPIO_CRL_MODE6      ( GPIO_CRL_MODE6_Msk )
#define GPIO_CRL_MODE7_Pos  ( 28 )
#define GPIO_CRL_MODE7_Msk  ( 0x3 << GPIO_CRL_MODE7_Pos )
#define GPIO_CRL_MODE7      ( GPIO_CRL_MODE7_Msk )
#define GPIO_CRL_CNF_Pos    ( 2 )
#define GPIO_CRL_CNF_Msk    ( 0x33333333 << GPIO_CRL_CNF_Pos )
#define GPIO_CRL_CNF        ( GPIO_CRL_CNF_Msk )
#define GPIO_CRL_CNF0_Pos   ( 2 )
#define GPIO_CRL_CNF0_Msk   ( 0x3 << GPIO_CRL_CNF0_Pos )
#define GPIO_CRL_CNF0       ( GPIO_CRL_CNF0_Msk )
#define GPIO_CRL_CNF1_Pos   ( 6 )
#define GPIO_CRL_CNF1_Msk   ( 0x3 << GPIO_CRL_CNF1_Pos )
#define GPIO_CRL_CNF1       ( GPIO_CRL_CNF1_Msk )
#define GPIO_CRL_CNF2_Pos   ( 10 )
#define GPIO_CRL_CNF2_Msk   ( 0x3 << GPIO_CRL_CNF2_Pos )
#define GPIO_CRL_CNF2       ( GPIO_CRL_CNF2_Msk )
#define GPIO_CRL_CNF3_Pos   ( 14 )
#define GPIO_CRL_CNF3_Msk   ( 0x3 << GPIO_CRL_CNF3_Pos )
#define GPIO_CRL_CNF3       ( GPIO_CRL_CNF3_Msk )
#define GPIO_CRL_CNF4_Pos   ( 18 )
#define GPIO_CRL_CNF4_Msk   ( 0x3 << GPIO_CRL_CNF4_Pos )
#define GPIO_CRL_CNF4       ( GPIO_CRL_CNF4_Msk )
#define GPIO_CRL_CNF5_Pos   ( 22 )
#define GPIO_CRL_CNF5_Msk   ( 0x3 << GPIO_CRL_CNF5_Pos )
#define GPIO_CRL_CNF5       ( GPIO_CRL_CNF5_Msk )
#define GPIO_CRL_CNF6_Pos   ( 26 )
#define GPIO_CRL_CNF6_Msk   ( 0x3 << GPIO_CRL_CNF6_Pos )
#define GPIO_CRL_CNF6       ( GPIO_CRL_CNF6_Msk )
#define GPIO_CRL_CNF7_Pos   ( 30 )
#define GPIO_CRL_CNF7_Msk   ( 0x3 << GPIO_CRL_CNF7_Pos )
#define GPIO_CRL_CNF7       ( GPIO_CRL_CNF7_Msk )
/* CRH */
#define GPIO_CRH_MODE_Pos   ( 0 )
#define GPIO_CRH_MODE_Msk   ( 0x33333333 << GPIO_CRH_MODE_Pos )
#define GPIO_CRH_MODE       ( GPIO_CRH_MODE_Msk )
#define GPIO_CRH_MODE8_Pos  ( 0 )
#define GPIO_CRH_MODE8_Msk  ( 0x3 << GPIO_CRH_MODE8_Pos )
#define GPIO_CRH_MODE8      ( GPIO_CRH_MODE8_Msk )
#define GPIO_CRH_MODE9_Pos  ( 4 )
#define GPIO_CRH_MODE9_Msk  ( 0x3 << GPIO_CRH_MODE9_Pos )
#define GPIO_CRH_MODE9      ( GPIO_CRH_MODE9_Msk )
#define GPIO_CRH_MODE10_Pos ( 8 )
#define GPIO_CRH_MODE10_Msk ( 0x3 << GPIO_CRH_MODE10_Pos )
#define GPIO_CRH_MODE10     ( GPIO_CRH_MODE10_Msk )
#define GPIO_CRH_MODE11_Pos ( 12 )
#define GPIO_CRH_MODE11_Msk ( 0x11 << GPIO_CRH_MODE11_Pos )
#define GPIO_CRH_MODE11     ( GPIO_CRH_MODE11_Msk )
#define GPIO_CRH_MODE12_Pos ( 16 )
#define GPIO_CRH_MODE12_Msk ( 0x3 << GPIO_CRH_MODE12_Pos )
#define GPIO_CRH_MODE12     ( GPIO_CRH_MODE12_Msk )
#define GPIO_CRH_MODE13_Pos ( 20 )
#define GPIO_CRH_MODE13_Msk ( 0x3 << GPIO_CRH_MODE13_Pos )
#define GPIO_CRH_MODE13     ( GPIO_CRH_MODE13_Msk )
#define GPIO_CRH_MODE14_Pos ( 24 )
#define GPIO_CRH_MODE14_Msk ( 0x3 << GPIO_CRH_MODE14_Pos )
#define GPIO_CRH_MODE14     ( GPIO_CRH_MODE14_Msk )
#define GPIO_CRH_MODE15_Pos ( 28 )
#define GPIO_CRH_MODE15_Msk ( 0x3 << GPIO_CRH_MODE15_Pos )
#define GPIO_CRH_MODE15     ( GPIO_CRH_MODE15_Msk )
#define GPIO_CRH_CNF_Pos    ( 2 )
#define GPIO_CRH_CNF_Msk    ( 0x33333333 << GPIO_CRH_CNF_Pos )
#define GPIO_CRH_CNF        ( GPIO_CRH_CNF_Msk )
#define GPIO_CRH_CNF8_Pos   ( 2 )
#define GPIO_CRH_CNF8_Msk   ( 0x3 << GPIO_CRH_CNF8_Pos )
#define GPIO_CRH_CNF8       ( GPIO_CRH_CNF8_Msk )
#define GPIO_CRH_CNF9_Pos   ( 6 )
#define GPIO_CRH_CNF9_Msk   ( 0x3 << GPIO_CRH_CNF9_Pos )
#define GPIO_CRH_CNF9       ( GPIO_CRH_CNF9_Msk )
#define GPIO_CRH_CNF10_Pos  ( 10 )
#define GPIO_CRH_CNF10_Msk  ( 0x3 << GPIO_CRH_CNF10_Pos )
#define GPIO_CRH_CNF10      ( GPIO_CRH_CNF10_Msk )
#define GPIO_CRH_CNF11_Pos  ( 14 )
#define GPIO_CRH_CNF11_Msk  ( 0x11 << GPIO_CRH_CNF11_Pos )
#define GPIO_CRH_CNF11      ( GPIO_CRH_CNF11_Msk )
#define GPIO_CRH_CNF12_Pos  ( 18 )
#define GPIO_CRH_CNF12_Msk  ( 0x3 << GPIO_CRH_CNF12_Pos )
#define GPIO_CRH_CNF12      ( GPIO_CRH_CNF12_Msk )
#define GPIO_CRH_CNF13_Pos  ( 22 )
#define GPIO_CRH_CNF13_Msk  ( 0x3 << GPIO_CRH_CNF13_Pos )
#define GPIO_CRH_CNF13      ( GPIO_CRH_CNF13_Msk )
#define GPIO_CRH_CNF14_Pos  ( 214 )
#define GPIO_CRH_CNF14_Msk  ( 0x3 << GPIO_CRH_CNF14_Pos )
#define GPIO_CRH_CNF14      ( GPIO_CRH_CNF14_Msk )
#define GPIO_CRH_CNF15_Pos  ( 30 )
#define GPIO_CRH_CNF15_Msk  ( 0x3 << GPIO_CRH_CNF15_Pos )
#define GPIO_CRH_CNF15      ( GPIO_CRH_CNF15_Msk )

/* Global DMA register bit definitions. */

/* Per-channel DMA register bit definitions. */
/* CCR / CHCTL */
#define DMA_CCR_EN_Pos      ( 0 )
#define DMA_CCR_EN_Msk      ( 0x1 << DMA_CCR_EN_Pos )
#define DMA_CCR_EN          ( DMA_CCR_EN_Msk )
#define DMA_CCR_TCIE_Pos    ( 1 )
#define DMA_CCR_TCIE_Msk    ( 0x1 << DMA_CCR_TCIE_Pos )
#define DMA_CCR_TCIE        ( DMA_CCR_TCIE_Msk )
#define DMA_CCR_HTIE_Pos    ( 2 )
#define DMA_CCR_HTIE_Msk    ( 0x1 << DMA_CCR_HTIE_Pos )
#define DMA_CCR_HTIE        ( DMA_CCR_HTIE_Msk )
#define DMA_CCR_TEIE_Pos    ( 3 )
#define DMA_CCR_TEIE_Msk    ( 0x1 << DMA_CCR_TEIE_Pos )
#define DMA_CCR_TEIE        ( DMA_CCR_TEIE_Msk )
#define DMA_CCR_DIR_Pos     ( 4 )
#define DMA_CCR_DIR_Msk     ( 0x1 << DMA_CCR_DIR_Pos )
#define DMA_CCR_DIR         ( DMA_CCR_DIR_Msk )
#define DMA_CCR_CIRC_Pos    ( 5 )
#define DMA_CCR_CIRC_Msk    ( 0x1 << DMA_CCR_CIRC_Pos )
#define DMA_CCR_CIRC        ( DMA_CCR_CIRC_Msk )
#define DMA_CCR_PINC_Pos    ( 6 )
#define DMA_CCR_PINC_Msk    ( 0x1 << DMA_CCR_PINC_Pos )
#define DMA_CCR_PINC        ( DMA_CCR_PINC_Msk )
#define DMA_CCR_MINC_Pos    ( 7 )
#define DMA_CCR_MINC_Msk    ( 0x1 << DMA_CCR_MINC_Pos )
#define DMA_CCR_MINC        ( DMA_CCR_MINC_Msk )
#define DMA_CCR_PSIZE_Pos   ( 8 )
#define DMA_CCR_PSIZE_Msk   ( 0x3 << DMA_CCR_PSIZE_Pos )
#define DMA_CCR_PSIZE       ( DMA_CCR_PSIZE_Msk )
#define DMA_CCR_MSIZE_Pos   ( 10 )
#define DMA_CCR_MSIZE_Msk   ( 0x3 << DMA_CCR_MSIZE_Pos )
#define DMA_CCR_MSIZE       ( DMA_CCR_MSIZE_Msk )
#define DMA_CCR_PL_Pos      ( 12 )
#define DMA_CCR_PL_Msk      ( 0x3 << DMA_CCR_PL_Pos )
#define DMA_CCR_PL          ( DMA_CCR_PL_Msk )
#define DMA_CCR_MEM2MEM_Pos ( 14 )
#define DMA_CCR_MEM2MEM_Msk ( 0x1 << DMA_CCR_MEM2MEM_Pos )
#define DMA_CCR_MEM2MEM     ( DMA_CCR_MEM2MEM_Msk )

/* SPI register bit definitions. */
/* CR1 */
#define SPI_CR1_CPHA_Pos     ( 0 )
#define SPI_CR1_CPHA_Msk     ( 0x1 << SPI_CR1_CPHA_Pos )
#define SPI_CR1_CPHA         ( SPI_CR1_CPHA_Msk )
#define SPI_CR1_CPOL_Pos     ( 1 )
#define SPI_CR1_CPOL_Msk     ( 0x1 << SPI_CR1_CPOL_Pos )
#define SPI_CR1_CPOL         ( SPI_CR1_CPOL_Msk )
#define SPI_CR1_MSTR_Pos     ( 2 )
#define SPI_CR1_MSTR_Msk     ( 0x1 << SPI_CR1_MSTR_Pos )
#define SPI_CR1_MSTR         ( SPI_CR1_MSTR_Msk )
#define SPI_CR1_BR_Pos       ( 3 )
#define SPI_CR1_BR_Msk       ( 0x7 << SPI_CR1_BR_Pos )
#define SPI_CR1_BR           ( SPI_CR1_BR_Msk )
#define SPI_CR1_SPE_Pos      ( 6 )
#define SPI_CR1_SPE_Msk      ( 0x1 << SPI_CR1_SPE_Pos )
#define SPI_CR1_SPE          ( SPI_CR1_SPE_Msk )
#define SPI_CR1_LSBFIRST_Pos ( 7 )
#define SPI_CR1_LSBFIRST_Msk ( 0x1 << SPI_CR1_LSBFIRST_Pos )
#define SPI_CR1_LSBFIRST     ( SPI_CR1_LSBFIRST_Msk )
#define SPI_CR1_SSI_Pos      ( 8 )
#define SPI_CR1_SSI_Msk      ( 0x1 << SPI_CR1_SSI_Pos )
#define SPI_CR1_SSI          ( SPI_CR1_SSI_Msk )
#define SPI_CR1_SSM_Pos      ( 9 )
#define SPI_CR1_SSM_Msk      ( 0x1 << SPI_CR1_SSM_Pos )
#define SPI_CR1_SSM          ( SPI_CR1_SSM_Msk )
#define SPI_CR1_RXONLY_Pos   ( 10 )
#define SPI_CR1_RXONLY_Msk   ( 0x1 << SPI_CR1_RXONLY_Pos )
#define SPI_CR1_RXONLY       ( SPI_CR1_RXONLY_Msk )
#define SPI_CR1_DFF_Pos      ( 11 )
#define SPI_CR1_DFF_Msk      ( 0x1 << SPI_CR1_DFF_Pos )
#define SPI_CR1_DFF          ( SPI_CR1_DFF_Msk )
#define SPI_CR1_CRCNEXT_Pos  ( 12 )
#define SPI_CR1_CRCNEXT_Msk  ( 0x1 << SPI_CR1_CRCNEXT_Pos )
#define SPI_CR1_CRCNEXT      ( SPI_CR1_CRCNEXT_Msk )
#define SPI_CR1_CRCEN_Pos    ( 13 )
#define SPI_CR1_CRCEN_Msk    ( 0x1 << SPI_CR1_CRCEN_Pos )
#define SPI_CR1_CRCEN        ( SPI_CR1_CRCEN_Msk )
#define SPI_CR1_BIDIOE_Pos   ( 14 )
#define SPI_CR1_BIDIOE_Msk   ( 0x1 << SPI_CR1_BIDIOE_Pos )
#define SPI_CR1_BIDIOE       ( SPI_CR1_BIDIOE_Msk )
#define SPI_CR1_BIDIMODE_Pos ( 15 )
#define SPI_CR1_BIDIMODE_Msk ( 0x1 << SPI_CR1_BIDIMODE_Pos )
#define SPI_CR1_BIDIMODE     ( SPI_CR1_BIDIMODE_Msk )
/* CR2 */
#define SPI_CR2_RXDMAEN_Pos ( 0 )
#define SPI_CR2_RXDMAEN_Msk ( 0x1 << SPI_CR2_RXDMAEN_Pos )
#define SPI_CR2_RXDMAEN     ( SPI_CR2_RXDMAEN_Msk )
#define SPI_CR2_TXDMAEN_Pos ( 1 )
#define SPI_CR2_TXDMAEN_Msk ( 0x1 << SPI_CR2_TXDMAEN_Pos )
#define SPI_CR2_TXDMAEN     ( SPI_CR2_TXDMAEN_Msk )
#define SPI_CR2_SSOE_Pos    ( 2 )
#define SPI_CR2_SSOE_Msk    ( 0x1 << SPI_CR2_SSOE_Pos )
#define SPI_CR2_SSOE        ( SPI_CR2_SSOE_Msk )
// Note: 'NSS pulse mode' and 'TI mode' bits don't show up in
// STM32F103xB reference documents.
#define SPI_CR2_NSSP_Pos    ( 3 )
#define SPI_CR2_NSSP_Msk    ( 0x1 << SPI_CR2_NSSP_Pos )
#define SPI_CR2_NSSP        ( SPI_CR2_NSSP_Msk )
#define SPI_CR2_TIMOD_Pos   ( 4 )
#define SPI_CR2_TIMOD_Msk   ( 0x1 << SPI_CR2_TIMOD_Pos )
#define SPI_CR2_TIMOD       ( SPI_CR2_TIMOD_Msk )
#define SPI_CR2_ERRIE_Pos   ( 5 )
#define SPI_CR2_ERRIE_Msk   ( 0x1 << SPI_CR2_ERRIE_Pos )
#define SPI_CR2_ERRIE       ( SPI_CR2_ERRIE_Msk )
#define SPI_CR2_RXNEIE_Pos  ( 6 )
#define SPI_CR2_RXNEIE_Msk  ( 0x1 << SPI_CR2_RXNEIE_Pos )
#define SPI_CR2_RXNEIE      ( SPI_CR2_RXNEIE_Msk )
#define SPI_CR2_TXEIE_Pos   ( 7 )
#define SPI_CR2_TXEIE_Msk   ( 0x1 << SPI_CR2_TXEIE_Pos )
#define SPI_CR2_TXEIE       ( SPI_CR2_TXEIE_Msk )
/* SR / STAT */
#define SPI_SR_RXNE_Pos   ( 0 )
#define SPI_SR_RXNE_Msk   ( 0x1 << SPI_SR_RXNE_Pos )
#define SPI_SR_RXNE       ( SPI_SR_RXNE_Msk )
#define SPI_SR_TXE_Pos    ( 1 )
#define SPI_SR_TXE_Msk    ( 0x1 << SPI_SR_TXE_Pos )
#define SPI_SR_TXE        ( SPI_SR_TXE_Msk )
#define SPI_SR_CHSIDE_Pos ( 2 )
#define SPI_SR_CHSIDE_Msk ( 0x1 << SPI_SR_CHSIDE_Pos )
#define SPI_SR_CHSIDE     ( SPI_SR_CHSIDE_Msk )
#define SPI_SR_UDR_Pos    ( 3 )
#define SPI_SR_UDR_Msk    ( 0x1 << SPI_SR_UDR_Pos )
#define SPI_SR_UDR        ( SPI_SR_UDR_Msk )
#define SPI_SR_CRCERR_Pos ( 4 )
#define SPI_SR_CRCERR_Msk ( 0x1 << SPI_SR_CRCERR_Pos )
#define SPI_SR_CRCERR     ( SPI_SR_CRCERR_Msk )
#define SPI_SR_MODF_Pos   ( 5 )
#define SPI_SR_MODF_Msk   ( 0x1 << SPI_SR_MODF_Pos )
#define SPI_SR_MODF       ( SPI_SR_MODF_Msk )
#define SPI_SR_OVR_Pos    ( 6 )
#define SPI_SR_OVR_Msk    ( 0x1 << SPI_SR_OVR_Pos )
#define SPI_SR_OVR        ( SPI_SR_OVR_Msk )
#define SPI_SR_BSY_Pos    ( 7 )
#define SPI_SR_BSY_Msk    ( 0x1 << SPI_SR_BSY_Pos )
#define SPI_SR_BSY        ( SPI_SR_BSY_Msk )
// Note: "Format error" bit doesn't show up in STM32F103xB docs.
#define SPI_SR_FERR_Pos   ( 8 )
#define SPI_SR_FERR_Msk   ( 0x1 << SPI_SR_FERR_Pos )
#define SPI_SR_FERR       ( SPI_SR_FERR_Msk )

#endif
