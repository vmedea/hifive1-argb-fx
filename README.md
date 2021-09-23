# HiFive1-based ARGB controller

This is a project to drive ARGB-based computer case lighting from a HiFive1 board. I am using this with a Silverstone SST-RVZ03B-ARGB case, but it will likely work with others. I ditched the built-in 8051-based LED controller board and replaced it with the HiFive1 board, which I had collecting dust anyway.

# Setup

This software drives a 16-LED (the number can be changed by changing `N_LEDS` in the source code) WS2812B (Neopixel-ish) LED strip using the SPI controller on a [HiFive1 evaluation board](https://www.sifive.com/products/hifive1/). The SoC on this board is a [SiFive Freedom E310](https://www.sifive.com/documentation/chips/freedom-e310-g000-manual/).

## SDK

First you need to install the Freedom-E-SDK as described [here](https://github.com/sifive/freedom-e-sdk/tree/v1_0). You need the `v1_0` branch, not Metal (`master` branch). This is an intentional choice because I prefer the low-level register abstractions. Build it from source or install pre-built binaries.

Next edit `config.mk` and set the `SDKDIR` variable to where you installed the SDK.

## Hardware

This uses the old HiFive1 board, not the HiFive1 Rev B. Some changes to the code, as well as a level-shifter, may be necessary to make it work on that.

Set the I/O voltage jumper on the HiFive1 board to 5V. It will likely not work at all at 3.3V because the signal voltage will be too low to register.

### Headers

There seem to be two types of ARGB headers:

- 5V RGB header three(of four)-wire "addressable RGB" (ARGB) light strip

```
    |     |     |     |     |
    | +5V |  D  | N/A | GND |
    |     |     |     |     |
```

- Three-pin connector "Mizucool Gigabyte VDG header", also just "VDG header":

```
         -       -
    -------------------
    |     |     |     |
    | +5V | D   | GND |
    |     |     |     |
    -------------------
    |-    -     -    -|
    -------------------
```

In either case, connect the `D` wire of the LED strip to PIN-3 (`SPI1_MOSI`) on the HiFive1, the `+5V` to a power supply, and the `GND` to the same ground as the HiFive1.

# Compile the code

Just type `make software` to compile the code.

# Upload and run the code

Connect your HiFive1 to the host computer in the usual way and type `make upload`. That should upload the code.

# UART protocol

A simple protocol is used at 115200 baud. First, a single-character effect identifier is sent from the host, the device replies with `C`, then the host sends optional parameters. After the parameters are received the effect starts the next frame. See `handle_uart_interrupt()`.

## Available effects

- `x` Reset to initial color (no parameters)
- `o` Turn all leds off (no parameters)
- `f` Manual color setting (follow with 3 * `N_LEDS` bytes for R, G, B for each led)
- `s` Simplex noise (a kind of lava lamp effect).
  - First byte is mode-
    - `0` FULLRGB: full RGB noise
    - `1` COLOR1: interpolate between black and one color
    - `2` COLOR2: interpolate between black and two colors (using two noise channels)
    - `3` COLORN: interpolate between black and white, multiplied with a color per led
  - Then `N_LEDS` times 3 bytes for RGB color parameters. These always need to be provided even for the modes that use no, or only one, color.
- `r` Rainbow (no parameters at this time)
- `p` Pulse all leds (no parameters at this time)
- `2` One or two moving dots (no parameters at this time)

# Credits

- Loosely based on hifive1-neopixel by Curt Brune, but it uses SPI instead of GPIO bit-banging, takes commands through the UART, and uses interrupts to drive timing. Also, the effects have been changed. Not much of the original code remains. But thanks a lot for getting me started.
