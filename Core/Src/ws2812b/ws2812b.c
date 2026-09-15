/*

  WS2812B CPU and memory efficient library

  Date: 28.9.2016

  Author: Martin Hubacek
                  http://www.martinhubacek.cz
                  @hubmartin

  Licence: MIT License

  Ported to STM32H723 (Nucleo-H723ZG):
    - TIM2 (APB1) PWM CH1/CH2 + Update, DMA via DMAMUX
    - Uses CubeMX-generated handles htim2, hdma_tim2_up/ch1/ch2
    - IRQs dispatched by generated stm32h7xx_it.c

*/

#include <string.h>

#include "stm32h7xx_hal.h"
#include "ws2812b.h"

// TIM2 kernel clock (APB1 timer clock). At SYSCLK = 400 MHz this is 200 MHz.
#define WS2812B_TIMER_CLOCK 200000000UL

// Peripheral handles are provided by CubeMX-generated code (tim.c)
extern TIM_HandleTypeDef htim2;
extern DMA_HandleTypeDef hdma_tim2_up;
extern DMA_HandleTypeDef hdma_tim2_ch1;
extern DMA_HandleTypeDef hdma_tim2_ch2;

// Actual definition of the global driver state (declared extern in ws2812b.h)
WS2812_Struct ws2812b;

// Forward declaration (implementation at the bottom of this file)
static void ws2812b_set_pixel(uint8_t row, uint16_t column, uint8_t red,
                              uint8_t green, uint8_t blue);

// DMA source arrays. These MUST live in a DMA-accessible memory region
// (AXI SRAM / RAM_D1), because DMA1/DMA2 cannot access DTCM on STM32H7.
// The .dma_buffer section is NOLOAD, so values are assigned at runtime.
uint32_t WS2812_IO_High[1] __attribute__((section(".dma_buffer")));
uint32_t WS2812_IO_Low[1] __attribute__((section(".dma_buffer")));

// WS2812 framebuffer - buffer for 2 LEDs - two times 24 bits
uint16_t ws2812bDmaBitBuffer[24 * 2] __attribute__((section(".dma_buffer")));

// Gamma correction table
const uint8_t gammaTable[] = {
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   1,   1,
    1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   2,   2,   2,   2,
    2,   2,   2,   2,   3,   3,   3,   3,   3,   3,   3,   4,   4,   4,   4,
    4,   5,   5,   5,   5,   6,   6,   6,   6,   7,   7,   7,   7,   8,   8,
    8,   9,   9,   9,   10,  10,  10,  11,  11,  11,  12,  12,  13,  13,  13,
    14,  14,  15,  15,  16,  16,  17,  17,  18,  18,  19,  19,  20,  20,  21,
    21,  22,  22,  23,  24,  24,  25,  25,  26,  27,  27,  28,  29,  29,  30,
    31,  32,  32,  33,  34,  35,  35,  36,  37,  38,  39,  39,  40,  41,  42,
    43,  44,  45,  46,  47,  48,  49,  50,  50,  51,  52,  54,  55,  56,  57,
    58,  59,  60,  61,  62,  63,  64,  66,  67,  68,  69,  70,  72,  73,  74,
    75,  77,  78,  79,  81,  82,  83,  85,  86,  87,  89,  90,  92,  93,  95,
    96,  98,  99,  101, 102, 104, 105, 107, 109, 110, 112, 114, 115, 117, 119,
    120, 122, 124, 126, 127, 129, 131, 133, 135, 137, 138, 140, 142, 144, 146,
    148, 150, 152, 154, 156, 158, 160, 162, 164, 167, 169, 171, 173, 175, 177,
    180, 182, 184, 186, 189, 191, 193, 196, 198, 200, 203, 205, 208, 210, 213,
    215, 218, 220, 223, 225, 228, 231, 233, 236, 239, 241, 244, 247, 249, 252,
    255};

static void ws2812b_gpio_init(void) {
  // WS2812B outputs
  WS2812B_GPIO_CLK_ENABLE();
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = WS2812B_PINS;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(WS2812B_PORT, &GPIO_InitStruct);

// Enable output pins for debuging to see DMA Full and Half transfer interrupts
#if defined(LED_BLUE_PORT) && defined(LED_ORANGE_PORT)
  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

  GPIO_InitStruct.Pin = LED_BLUE_PIN;
  HAL_GPIO_Init(LED_BLUE_PORT, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = LED_ORANGE_PIN;
  HAL_GPIO_Init(LED_ORANGE_PORT, &GPIO_InitStruct);
#endif
}

uint32_t tim_period;
uint32_t timer_reset_pulse_period;

static void TIM2_init(void) {
  // This computation of pulse length works from the actual TIM2 kernel clock.
  tim_period =
      WS2812B_TIMER_CLOCK / 800000; // 1.25us bit period (250 counts @ 200MHz)
  timer_reset_pulse_period = (WS2812B_TIMER_CLOCK / (320 * 60)); // ~50us Treset

  uint32_t cc1 = (10 * tim_period) / 36; // WS2812B '0' high time (~0.35us)
  uint32_t cc2 = (10 * tim_period) / 15; // WS2812B '1' high time (~0.83us)

  // htim2 is base-initialized by MX_TIM2_Init(); set our period and compares.
  __HAL_TIM_SET_AUTORELOAD(&htim2, tim_period);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, cc1);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, cc2);

  // TIM2 NVIC interrupt is enabled by MX_TIM2_Init() (TIM2_IRQn)

  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

  __HAL_TIM_DISABLE(&htim2);
}

#define BUFFER_SIZE (sizeof(ws2812bDmaBitBuffer) / sizeof(uint16_t))

static void DMA_init(void) {
  // The .dma_buffer section is NOLOAD, so initialize the IO source arrays here.
  WS2812_IO_High[0] = WS2812B_PINS;
  WS2812_IO_Low[0] = WS2812B_PINS << 16;

  // The DMA streams are created and linked to htim2 by MX_TIM2_Init() /
  // HAL_TIM_PWM_MspInit(). Here we override the per-stream settings that this
  // driver needs and start the circular transfers.

  // TIM2 Update event -> set the WS2812 outputs HIGH (start of every bit)
  hdma_tim2_up.Init.Request = DMA_REQUEST_TIM2_UP;
  hdma_tim2_up.Init.Direction = DMA_MEMORY_TO_PERIPH;
  hdma_tim2_up.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_tim2_up.Init.MemInc = DMA_MINC_DISABLE;
  hdma_tim2_up.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  hdma_tim2_up.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
  hdma_tim2_up.Init.Mode = DMA_CIRCULAR;
  hdma_tim2_up.Init.Priority = DMA_PRIORITY_VERY_HIGH;
  hdma_tim2_up.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  HAL_DMA_Init(&hdma_tim2_up);
  HAL_DMA_Start(&hdma_tim2_up, (uint32_t)WS2812_IO_High,
                (uint32_t)(&WS2812B_PORT->BSRR), BUFFER_SIZE);

  // TIM2 CC1 event -> write the bit buffer into the reset half of BSRR (BR
  // bits)
  hdma_tim2_ch1.Init.Request = DMA_REQUEST_TIM2_CH1;
  hdma_tim2_ch1.Init.Direction = DMA_MEMORY_TO_PERIPH;
  hdma_tim2_ch1.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_tim2_ch1.Init.MemInc = DMA_MINC_ENABLE;
  hdma_tim2_ch1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  hdma_tim2_ch1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
  hdma_tim2_ch1.Init.Mode = DMA_CIRCULAR;
  hdma_tim2_ch1.Init.Priority = DMA_PRIORITY_VERY_HIGH;
  hdma_tim2_ch1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  HAL_DMA_Init(&hdma_tim2_ch1);
  HAL_DMA_Start(&hdma_tim2_ch1, (uint32_t)ws2812bDmaBitBuffer,
                (uint32_t)(&WS2812B_PORT->BSRR) + 2, BUFFER_SIZE);

  // TIM2 CC2 event -> set the WS2812 outputs LOW (end of every bit).
  // This stream carries the half/complete transfer interrupt.
  hdma_tim2_ch2.Init.Request = DMA_REQUEST_TIM2_CH2;
  hdma_tim2_ch2.Init.Direction = DMA_MEMORY_TO_PERIPH;
  hdma_tim2_ch2.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_tim2_ch2.Init.MemInc = DMA_MINC_DISABLE;
  hdma_tim2_ch2.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  hdma_tim2_ch2.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
  hdma_tim2_ch2.Init.Mode = DMA_CIRCULAR;
  hdma_tim2_ch2.Init.Priority = DMA_PRIORITY_VERY_HIGH;
  hdma_tim2_ch2.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  HAL_DMA_Init(&hdma_tim2_ch2);

  hdma_tim2_ch2.XferCpltCallback = DMA_TransferCompleteHandler;
  hdma_tim2_ch2.XferHalfCpltCallback = DMA_TransferHalfHandler;
  hdma_tim2_ch2.XferErrorCallback = DMA_TransferError;

  // DMA1_Stream1 (hdma_tim2_ch2) NVIC interrupt is enabled by MX_DMA_Init()
  HAL_DMA_Start_IT(&hdma_tim2_ch2, (uint32_t)WS2812_IO_Low,
                   (uint32_t)&WS2812B_PORT->BSRR, BUFFER_SIZE);
}

static void loadNextFramebufferData(WS2812_BufferItem *bItem, uint32_t row) {

  uint32_t r = bItem->frameBufferPointer[bItem->frameBufferCounter++];
  uint32_t g = bItem->frameBufferPointer[bItem->frameBufferCounter++];
  uint32_t b = bItem->frameBufferPointer[bItem->frameBufferCounter++];

  if (bItem->frameBufferCounter == bItem->frameBufferSize)
    bItem->frameBufferCounter = 0;

  ws2812b_set_pixel(bItem->channel, row, r, g, b);
}

// Transmit the framebuffer
static void WS2812_sendbuf() {
  // transmission complete flag
  ws2812b.transferComplete = 0;

  uint32_t i;

  for (i = 0; i < WS2812_BUFFER_COUNT; i++) {
    ws2812b.item[i].frameBufferCounter = 0;

    loadNextFramebufferData(&ws2812b.item[i], 0); // ROW 0
    loadNextFramebufferData(&ws2812b.item[i], 1); // ROW 0
  }

  // clear all DMA flags (portable flag-index macros)
  __HAL_DMA_CLEAR_FLAG(&hdma_tim2_up,
                       __HAL_DMA_GET_TC_FLAG_INDEX(&hdma_tim2_up) |
                           __HAL_DMA_GET_HT_FLAG_INDEX(&hdma_tim2_up) |
                           __HAL_DMA_GET_TE_FLAG_INDEX(&hdma_tim2_up));
  __HAL_DMA_CLEAR_FLAG(&hdma_tim2_ch1,
                       __HAL_DMA_GET_TC_FLAG_INDEX(&hdma_tim2_ch1) |
                           __HAL_DMA_GET_HT_FLAG_INDEX(&hdma_tim2_ch1) |
                           __HAL_DMA_GET_TE_FLAG_INDEX(&hdma_tim2_ch1));
  __HAL_DMA_CLEAR_FLAG(&hdma_tim2_ch2,
                       __HAL_DMA_GET_TC_FLAG_INDEX(&hdma_tim2_ch2) |
                           __HAL_DMA_GET_HT_FLAG_INDEX(&hdma_tim2_ch2) |
                           __HAL_DMA_GET_TE_FLAG_INDEX(&hdma_tim2_ch2));

  // configure the number of bytes to be transferred by the DMA controller
  // (Instance is void* on H7, cast to the stream type to reach NDTR)
  ((DMA_Stream_TypeDef *)hdma_tim2_up.Instance)->NDTR = BUFFER_SIZE;
  ((DMA_Stream_TypeDef *)hdma_tim2_ch1.Instance)->NDTR = BUFFER_SIZE;
  ((DMA_Stream_TypeDef *)hdma_tim2_ch2.Instance)->NDTR = BUFFER_SIZE;

  // clear all TIM2 flags
  __HAL_TIM_CLEAR_FLAG(&htim2,
                       TIM_FLAG_UPDATE | TIM_FLAG_CC1 | TIM_FLAG_CC2 |
                           TIM_FLAG_CC3 | TIM_FLAG_CC4);

  // enable DMA channels
  __HAL_DMA_ENABLE(&hdma_tim2_up);
  __HAL_DMA_ENABLE(&hdma_tim2_ch1);
  __HAL_DMA_ENABLE(&hdma_tim2_ch2);

  // IMPORTANT: enable the TIM2 DMA requests AFTER enabling the DMA channels!
  __HAL_TIM_ENABLE_DMA(&htim2, TIM_DMA_UPDATE);
  __HAL_TIM_ENABLE_DMA(&htim2, TIM_DMA_CC1);
  __HAL_TIM_ENABLE_DMA(&htim2, TIM_DMA_CC2);

  TIM2->CNT = tim_period - 1;

  // start TIM2
  __HAL_TIM_ENABLE(&htim2);
}

void DMA_TransferError(DMA_HandleTypeDef *DmaHandle) {
  volatile int i = 0;
  i++;
}

void DMA_TransferHalfHandler(DMA_HandleTypeDef *DmaHandle) {

  // Is this the last LED?
  if (ws2812b.repeatCounter == WS2812B_NUMBER_OF_LEDS) {

    // If this is the last pixel, set the next pixel value to zeros, because
    // the DMA would not stop exactly at the last bit.
    ws2812b_set_pixel(0, 0, 0, 0, 0);

  } else {
    uint32_t i;

    for (i = 0; i < WS2812_BUFFER_COUNT; i++) {
      loadNextFramebufferData(&ws2812b.item[i], 0);
    }

    ws2812b.repeatCounter++;
  }
}

void DMA_TransferCompleteHandler(DMA_HandleTypeDef *DmaHandle) {

#if defined(LED_ORANGE_PORT)
  LED_ORANGE_PORT->BSRR = LED_ORANGE_PIN;
#endif

  if (ws2812b.repeatCounter == WS2812B_NUMBER_OF_LEDS) {
    // Transfer of all LEDs is done, disable DMA but enable timer update IRQ to
    // stop the 50us pulse
    ws2812b.repeatCounter = 0;

    // Stop timer
    TIM2->CR1 &= ~TIM_CR1_CEN;

    // Disable DMA
    __HAL_DMA_DISABLE(&hdma_tim2_up);
    __HAL_DMA_DISABLE(&hdma_tim2_ch1);
    __HAL_DMA_DISABLE(&hdma_tim2_ch2);

    // Disable the DMA requests
    __HAL_TIM_DISABLE_DMA(&htim2, TIM_DMA_UPDATE);
    __HAL_TIM_DISABLE_DMA(&htim2, TIM_DMA_CC1);
    __HAL_TIM_DISABLE_DMA(&htim2, TIM_DMA_CC2);

    // Set 50us period for Treset pulse
    TIM2->ARR = timer_reset_pulse_period;
    // Reset the timer
    TIM2->CNT = 0;

    // Generate an update event to reload the prescaler value immediately
    TIM2->EGR = TIM_EGR_UG;
    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);

    // Enable TIM2 Update interrupt for 50us Treset signal
    __HAL_TIM_ENABLE_IT(&htim2, TIM_IT_UPDATE);
    // Enable timer
    TIM2->CR1 |= TIM_CR1_CEN;

    // Manually set outputs to low to generate 50us reset impulse
    WS2812B_PORT->BSRR = WS2812_IO_Low[0];
  } else {

    // Load bitbuffer with next RGB LED values
    uint32_t i;
    for (i = 0; i < WS2812_BUFFER_COUNT; i++) {
      loadNextFramebufferData(&ws2812b.item[i], 1);
    }

    ws2812b.repeatCounter++;
  }

#if defined(LED_ORANGE_PORT)
  LED_ORANGE_PORT->BSRR = LED_ORANGE_PIN << 16;
#endif
}

// TIM2 Interrupt Handler gets executed on every TIM2 Update if enabled
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  ws2812b.timerPeriodCounter = 0;
  TIM2->CR1 = 0; // disable timer

  // disable the TIM2 Update IRQ
  __HAL_TIM_DISABLE_IT(&htim2, TIM_IT_UPDATE);

  // Set back 1,25us period
  TIM2->ARR = tim_period;

  // Generate an update event to reload the Prescaler value immediatly
  TIM2->EGR = TIM_EGR_UG;
  __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);

  // set transfer_complete flag
  ws2812b.transferComplete = 1;
}

static void ws2812b_set_pixel(uint8_t row, uint16_t column, uint8_t red,
                              uint8_t green, uint8_t blue) {

  // Apply gamma
  red = gammaTable[red];
  green = gammaTable[green];
  blue = gammaTable[blue];

  uint32_t calcCol = (column * 24);
  uint32_t invRed = ~red;
  uint32_t invGreen = ~green;
  uint32_t invBlue = ~blue;

#if defined(SETPIX_1)
  uint8_t i;
  uint32_t calcClearRow = ~((0x01 << row) << 0);
  for (i = 0; i < 8; i++) {
    // clear the data for pixel

    ws2812bDmaBitBuffer[(calcCol + i)] &= calcClearRow;
    ws2812bDmaBitBuffer[(calcCol + 8 + i)] &= calcClearRow;
    ws2812bDmaBitBuffer[(calcCol + 16 + i)] &= calcClearRow;

    // write new data for pixel
    ws2812bDmaBitBuffer[(calcCol + i)] |=
        (((((invGreen) << i) & 0x80) >> 7) << (row + 0));
    ws2812bDmaBitBuffer[(calcCol + 8 + i)] |=
        (((((invRed) << i) & 0x80) >> 7) << (row + 0));
    ws2812bDmaBitBuffer[(calcCol + 16 + i)] |=
        (((((invBlue) << i) & 0x80) >> 7) << (row + 0));
  }
#elif defined(SETPIX_3)
  ws2812bDmaBitBuffer[(calcCol + 0)] |=
      (((((invGreen) << 0) & 0x80) >> 7) << row);
  ws2812bDmaBitBuffer[(calcCol + 8 + 0)] |=
      (((((invRed) << 0) & 0x80) >> 7) << row);
  ws2812bDmaBitBuffer[(calcCol + 16 + 0)] |=
      (((((invBlue) << 0) & 0x80) >> 7) << row);

  ws2812bDmaBitBuffer[(calcCol + 1)] |=
      (((((invGreen) << 1) & 0x80) >> 7) << row);
  ws2812bDmaBitBuffer[(calcCol + 8 + 1)] |=
      (((((invRed) << 1) & 0x80) >> 7) << row);
  ws2812bDmaBitBuffer[(calcCol + 16 + 1)] |=
      (((((invBlue) << 1) & 0x80) >> 7) << row);

  ws2812bDmaBitBuffer[(calcCol + 2)] |=
      (((((invGreen) << 2) & 0x80) >> 7) << row);
  ws2812bDmaBitBuffer[(calcCol + 8 + 2)] |=
      (((((invRed) << 2) & 0x80) >> 7) << row);
  ws2812bDmaBitBuffer[(calcCol + 16 + 2)] |=
      (((((invBlue) << 2) & 0x80) >> 7) << row);

  ws2812bDmaBitBuffer[(calcCol + 3)] |=
      (((((invGreen) << 3) & 0x80) >> 7) << row);
  ws2812bDmaBitBuffer[(calcCol + 8 + 3)] |=
      (((((invRed) << 3) & 0x80) >> 7) << row);
  ws2812bDmaBitBuffer[(calcCol + 16 + 3)] |=
      (((((invBlue) << 3) & 0x80) >> 7) << row);

  ws2812bDmaBitBuffer[(calcCol + 4)] |=
      (((((invGreen) << 4) & 0x80) >> 7) << row);
  ws2812bDmaBitBuffer[(calcCol + 8 + 4)] |=
      (((((invRed) << 4) & 0x80) >> 7) << row);
  ws2812bDmaBitBuffer[(calcCol + 16 + 4)] |=
      (((((invBlue) << 4) & 0x80) >> 7) << row);

  ws2812bDmaBitBuffer[(calcCol + 5)] |=
      (((((invGreen) << 5) & 0x80) >> 7) << row);
  ws2812bDmaBitBuffer[(calcCol + 8 + 5)] |=
      (((((invRed) << 5) & 0x80) >> 7) << row);
  ws2812bDmaBitBuffer[(calcCol + 16 + 5)] |=
      (((((invBlue) << 5) & 0x80) >> 7) << row);

  ws2812bDmaBitBuffer[(calcCol + 6)] |=
      (((((invGreen) << 6) & 0x80) >> 7) << row);
  ws2812bDmaBitBuffer[(calcCol + 8 + 6)] |=
      (((((invRed) << 6) & 0x80) >> 7) << row);
  ws2812bDmaBitBuffer[(calcCol + 16 + 6)] |=
      (((((invBlue) << 6) & 0x80) >> 7) << row);

  ws2812bDmaBitBuffer[(calcCol + 7)] |=
      (((((invGreen) << 7) & 0x80) >> 7) << row);
  ws2812bDmaBitBuffer[(calcCol + 8 + 7)] |=
      (((((invRed) << 7) & 0x80) >> 7) << row);
  ws2812bDmaBitBuffer[(calcCol + 16 + 7)] |=
      (((((invBlue) << 7) & 0x80) >> 7) << row);
#endif
}

void ws2812b_init() {
  ws2812b_gpio_init();

  DMA_init();
  TIM2_init();

  // Need to start the first transfer
  ws2812b.transferComplete = 1;
}

void ws2812b_handle() {
  if (ws2812b.startTransfer) {
    ws2812b.startTransfer = 0;
    WS2812_sendbuf();
  }
}
