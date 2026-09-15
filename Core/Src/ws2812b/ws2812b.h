/*

  WS2812B CPU and memory efficient library

  Date: 28.9.2016

  Author: Martin Hubacek
                  http://www.martinhubacek.cz
                  @hubmartin

  Licence: MIT License

*/

#ifndef WS2812B_H_
#define WS2812B_H_

#include "stm32h7xx_hal.h"

// GPIO enable command
#define WS2812B_GPIO_CLK_ENABLE() __HAL_RCC_GPIOC_CLK_ENABLE()
// LED output port
#define WS2812B_PORT GPIOC
// LED output pins
#define WS2812B_PINS (GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11)
// How many LEDs are in the series - only valid multiples by two
#define WS2812B_NUMBER_OF_LEDS 60

// Number of paralel output LED strips. Each has its own buffer.
// Supports up to 16 outputs on a single GPIO port
#define WS2812_BUFFER_COUNT 4

// Choose one of the bit-juggling setpixel implementation
// *******************************************************
#define SETPIX_1 // For loop, works everywhere, slow
//#define SETPIX_2	// Bit band in a loop
//#define SETPIX_3	// Like SETPIX_1 but with unrolled loop
//#define SETPIX_4	// Fastest copying using bit-banding (NOT available on
//Cortex-M7)

// DEBUG OUTPUT
// ********************
// Nucleo-H723ZG onboard LEDs (LD1 green = PB0, LD3 red = PB14)

// Set during DMA Half and Full transfer IRQ to debug how long IRQ is processing
#define LED_BLUE_PORT GPIOB
#define LED_BLUE_PIN GPIO_PIN_0

// Set during full transfer DMA and TIM IRQ
#define LED_ORANGE_PORT GPIOB
#define LED_ORANGE_PIN GPIO_PIN_14

// Public functions
// ****************
void ws2812b_init();
void ws2812b_handle();

// Library structures
// ******************
// This value sets number of periods to generate 50uS Treset signal
#define WS2812_RESET_PERIOD 50

typedef struct WS2812_BufferItem {
  uint8_t *frameBufferPointer;
  uint32_t frameBufferSize;
  uint32_t frameBufferCounter;
  uint8_t channel; // digital output pin/channel
} WS2812_BufferItem;

typedef struct WS2812_Struct {
  WS2812_BufferItem item[WS2812_BUFFER_COUNT];
  uint8_t transferComplete;
  uint8_t startTransfer;
  uint32_t timerPeriodCounter;
  uint32_t repeatCounter;
} WS2812_Struct;

extern WS2812_Struct ws2812b;

void DMA_TransferCompleteHandler(DMA_HandleTypeDef *DmaHandle);
void DMA_TransferHalfHandler(DMA_HandleTypeDef *DmaHandle);
void DMA_TransferError(DMA_HandleTypeDef *DmaHandle);

#endif /* WS2812B_H_ */
