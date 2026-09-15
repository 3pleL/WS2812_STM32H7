# WS2812B driver for STM32H723 (Nucleo-H723ZG)

A memory- and CPU-efficient WS2812B (NeoPixel) driver for the STM32H723,
driving multiple parallel LED strips from a single GPIO port using one timer
and three DMA streams (no per-bit CPU involvement).

This is a **port of Martin Hubáček's excellent STM32F4 implementation** to the
STM32H7 family.

- Original project (STM32F4): https://github.com/hubmartin/WS2812B_STM32F4
- Original author: Martin Hubáček — [@hubmartin](https://github.com/hubmartin)

All credit for the original design and the clever timer/DMA bit-generation
technique goes to Martin Hubáček. This repository only adapts that work to the
STM32H7.

## How it works (in brief)

A timer generates the WS2812 1.25 µs bit slots. Three DMA transfers, triggered
by the timer's Update / CC1 / CC2 events, write to the GPIO `BSRR` register:

1. **Update event** → drives all strip outputs **high** (start of each bit).
2. **CC1 event** → writes the encoded bit buffer, pulling the "0" bits **low**
   early (~0.35 µs).
3. **CC2 event** → drives all remaining outputs **low** (~0.83 µs, end of "1"
   bits).

Because everything is done by DMA to `BSRR`, up to 16 parallel strips can share
one GPIO port with almost no CPU cost.

## Hardware / configuration

Default configuration (Nucleo-H723ZG):

| Setting            | Value                                  |
| ------------------ | -------------------------------------- |
| System clock       | 400 MHz (SYSCLK)                       |
| Timer              | TIM2 (200 MHz APB1 timer clock)        |
| DMA                | DMA1 streams via DMAMUX (TIM2 UP/CH1/CH2) |
| LED output port    | GPIOC                                  |
| LED output pins    | PC8, PC9, PC10, PC11 (4 parallel strips) |
| LEDs per strip     | 60                                     |
| Debug LEDs         | LD1 (PB0), LD3 (PB14)                  |

Key defines live in `Core/Src/ws2812b/ws2812b.h`
(`WS2812B_PORT`, `WS2812B_PINS`, `WS2812B_NUMBER_OF_LEDS`, `WS2812_BUFFER_COUNT`).
Effects and framebuffer setup are in `Core/Src/visEffect.c`.

## Changes from the original STM32F4 version

- Retargeted **TIM1 + DMA2 (fixed channel)** → **TIM2 + DMA1 via DMAMUX** on the
  STM32H7 request-router architecture.
- Timing recomputed from the actual **200 MHz** TIM2 kernel clock (APB1 timer
  clock) instead of `SystemCoreClock`.
- **Removed Cortex-M4 bit-banding** in the pixel setter — the SRAM bit-band
  region does not exist on Cortex-M7. Uses the portable set-pixel implementation.
- **DMA buffers placed in AXI SRAM (RAM_D1)** via a dedicated `.dma_buffer`
  linker section, because the STM32H7 DMA1/DMA2 controllers cannot access the
  DTCM where `.data`/`.bss` are otherwise located.
- Driver reuses the **CubeMX-generated** peripheral handles (`htim2`,
  `hdma_tim2_up/ch1/ch2`) and the generated interrupt handlers.
- Project is a **STM32CubeMX / CMake** project with a VS Code build/flash/debug
  setup.

## Building

Requires the Arm GNU Toolchain, CMake, and Ninja (Ninja ships with STM32CubeCLT).

```sh
cmake --preset Debug
cmake --build build/Debug
```

Flashing (ST-Link):

```sh
STM32_Programmer_CLI -c port=SWD mode=UR -d build/Debug/WS2812_STM32H7.elf -rst
```

In VS Code, the included `.vscode` tasks provide **Build**, **Flash**, and an
**F5** debug configuration (Cortex-Debug + ST-Link).

## Wiring

- Connect each strip's data-in to the corresponding pin (PC8–PC11).
- Common ground between the board and the LED power supply.
- WS2812B expect ~5 V logic; a 3.3 V → 5 V level shifter on the data line is
  recommended for reliable operation.

## License

MIT — see [LICENSE](LICENSE).

The original STM32F4 implementation is Copyright (c) 2017 Martin Hub and remains
under the MIT License. The STM32H7 port is Copyright (c) 2026 3pleL, also
under the MIT License. The original copyright notice is retained as required.
