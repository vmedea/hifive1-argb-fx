/*
 * Copyright 2021 Mara Huldra <vmedea@protonmail.com>
 * All rights reserved
 *
 * Loosely based on hifive1-neopixel by Curt Brune.
 * Portions of this code and inspiration comes from the SiFive
 * Freedom-e-SDK.
 * Copyright 2016 SiFive, Inc.
 */

/* This program demonstrates driving Neopixel LEDs using a HiFive1
 * board from SiFive.  This board contains a SiFive Freedom E310
 * (FE310) microcontroller.
 */

#include <stdint.h>
#include <stdio.h>
#include <unistd.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "encoding.h"
#include "platform.h"
#include "plic/plic_driver.h"
#include "utility.h"
#include "opensimplex2d.h"

#include "gamma_table.h"

#define N_LEDS  (16)
#define MAYBE_UNUSED  __attribute__ ((unused))

const uint32_t USEC_PER_FRAME = 20000; /* Fixed frame rate of 50Hz */
const uint32_t TICKS_PER_FRAME = USEC_PER_FRAME * RTC_FREQ / 1000000;

/** Initial color. */
const uint8_t INIT_COL[3] = {128, 0, 128};

/**** SPI *******************************************************************/

/** For SPI need:
 * 400ns bit rate = 2 500 000 Hz
 * BIT-0 = 1000  0.4us high, 1.2us low
 * BIT-1 = 1100  0.8us high, 0.8us low
 */
#define SPI_HZ   (2500000)

/** Pattern per two bits. SPI is configured in MSB-first mode.
 *
 *  static const uint8_t spi_patterns[4] = {
 *      0x88, // 00
 *      0x8c, // 01
 *      0xc8, // 10
 *      0xcc, // 11
 *  };
 */
static inline uint8_t spi_patterns(uint8_t bits)
{
    return (0xccc88c88 >> (bits * 8)) & 0xff;
}

static void inline spi1_send(uint8_t value) {
    while ((SPI1_REG(SPI_REG_TXFIFO) & SPI_TXFIFO_FULL) != 0) ;
    SPI1_REG(SPI_REG_TXFIFO) = value;
}

static inline void send_byte(uint8_t byte) {
    spi1_send(spi_patterns((byte >> 6) & 3));
    spi1_send(spi_patterns((byte >> 4) & 3));
    spi1_send(spi_patterns((byte >> 2) & 3));
    spi1_send(spi_patterns((byte >> 0) & 3));
}

static inline void send_pixel(uint8_t r, uint8_t g, uint8_t b) {
    send_byte(g);
    send_byte(r);
    send_byte(b);
}

static inline void latch() {
    spi1_send(0x00);
    spi1_send(0x00);
    spi1_send(0x00);
}

/**** Interrupt handling ****************************************************/
/** Structures for registering different interrupt handlers
 * for different parts of the application.
 */
typedef void (*interrupt_handler_t) (void);

void no_interrupt_handler (void) {}

interrupt_handler_t g_ext_interrupt_handlers[PLIC_NUM_INTERRUPTS];

/** Instance data for the PLIC. */
plic_instance_t g_plic;

/** Predeclare handle UART command. */
static void handle_uart_interrupt();

/** Entry Point for PLIC Interrupt Handler. */
void handle_m_ext_interrupt()
{
    plic_source int_num  = PLIC_claim_interrupt(&g_plic);
    if ((int_num >= 1) && (int_num < PLIC_NUM_INTERRUPTS)) {
        g_ext_interrupt_handlers[int_num]();
    } else {
        exit(1 + (uintptr_t) int_num);
    }
    PLIC_complete_interrupt(&g_plic, int_num);
}

/** Reset the timer. This also clears the existing timer interrupt. */
static void set_frame_timer()
{
    volatile uint64_t * mtime       = (uint64_t*) (CLINT_CTRL_ADDR + CLINT_MTIME);
    volatile uint64_t * mtimecmp    = (uint64_t*) (CLINT_CTRL_ADDR + CLINT_MTIMECMP);
    uint64_t now = *mtime;
    uint64_t then = now + TICKS_PER_FRAME;
    *mtimecmp = then;
}

static void interrupt_setup()
{
    PLIC_init(&g_plic, PLIC_CTRL_ADDR, PLIC_NUM_INTERRUPTS, PLIC_NUM_PRIORITIES);

    // Disable the machine & timer interrupts until setup is done.
    clear_csr(mie, MIP_MEIP);
    clear_csr(mie, MIP_MTIP);

    for (int ii = 0; ii < PLIC_NUM_INTERRUPTS; ii ++){
        g_ext_interrupt_handlers[ii] = no_interrupt_handler;
    }

    // Enable UART receive interrupt.
    UART0_REG(UART_REG_IE) = UART_IP_RXWM;
    // Unnecessary: RXWM will be 0, which means to trigger on every character, what we want.
    // UART0_REG(UART_REG_RXCTRL) = (UART0_REG(UART_REG_RXCTRL) & 0xffff) | UART_RXWM(1);

    g_ext_interrupt_handlers[INT_UART0_BASE] = &handle_uart_interrupt;
    PLIC_enable_interrupt(&g_plic, INT_UART0_BASE);
    PLIC_set_priority(&g_plic, INT_UART0_BASE, 1);

    // Set frame timer to go off.
    set_frame_timer();

    // Enable the Machine-External bit in MIE
    set_csr(mie, MIP_MEIP);

    // Enable the Machine-Timer bit in MIE
    set_csr(mie, MIP_MTIP);

    // Enable interrupts in general.
    set_csr(mstatus, MSTATUS_MIE);
}

/**** Effects ***************************************************************/

#define CSTEP (28)
MAYBE_UNUSED static void rainbow_pixel(int idx, uint8_t *col_out) {
    int r = 0, g = 0, b = 0;

    idx = idx % 60;
    if (idx < 10) {
        r = 255;
        b = 0;
        g = (idx * CSTEP) > 255 ? 255 : idx * CSTEP;
    }
    else if (idx < 20) {
        g = 255;
        b = 0;
        r = ((255 - (idx-10)*CSTEP) < 0) ? 0 : (255 - (idx-10)*CSTEP);
    }
    else if (idx < 30) {
        r = 0;
        g = 255;
        b = ((idx-20) * CSTEP) > 255 ? 255 : (idx-20) * CSTEP;
    }
    else if (idx < 40) {
        r = 0;
        b = 255;
        g = ((255 - (idx-30)*CSTEP) < 0) ? 0 : (255 - (idx-30)*CSTEP);
    }
    else if (idx < 50) {
        b = 255;
        g = 0;
        r = ((idx-40) * CSTEP) > 255 ? 255 : (idx-40) * CSTEP;
    }
    else if (idx < 60) {
        g = 0;
        r = 255;
        b = ((255 - (idx-50)*CSTEP) < 0) ? 0 : (255 - (idx-50)*CSTEP);
    }
    col_out[0] = r;
    col_out[1] = g;
    col_out[2] = b;
}

/** -1..1 to 0..1. */
static osp_fixp rescale(osp_fixp v)
{
    return (v + OSP_PRECISION) / 2;
}

/** Simplex effect state. */
struct simplex_state {
    struct opensimplex2d s[3];
    osp_fixp xscale;
    osp_fixp yscale;
    osp_fixp yval;

    uint8_t mode;
    uint8_t col[N_LEDS][3];
};

void simplex_init(struct simplex_state *self)
{
    // A stack of noise generators
    opensimplex2d_init(&self->s[0], 1);
    opensimplex2d_init(&self->s[1], 2);
    opensimplex2d_init(&self->s[2], 3);

    self->xscale = OSP_FCONST(0.3 * 0.5);
    self->yscale = OSP_FCONST(0.04 * 0.3 * 0.7);

    self->mode = 0;
    memset(&self->col[0][0], 0, sizeof(self->col));

    self->yval = 0;
}

/** Gamma-correct fixed-point color component to LED PWM value. */
#define GAMMA_COR(val) (gamma_table[(val) >> (OSP_SHIFT - (GT_BITS - 8))])

void simplex_frame(struct simplex_state *self, uint8_t *data, bool *post_frame)
{
    osp_fixp xval = 0;
    switch (self->mode) {
    case 0: // FULLRGB
        for (int x = 0; x < N_LEDS; ++x) {
            for (int i = 0; i < 3; ++i) {
                osp_fixp val = rescale(opensimplex2d_noise(&self->s[i], xval, self->yval));
                data[x * 3 + i] = GAMMA_COR(255 * val);
            }
            xval += self->xscale;
        }
        break;
    case 1: // COLOR1
        for (int x = 0; x < N_LEDS; ++x) {
            osp_fixp val = rescale(opensimplex2d_noise(&self->s[0], xval, self->yval));
            for (int i = 0; i < 3; ++i) {
                data[x * 3 + i] = GAMMA_COR(self->col[0][i] * val);
            }
            xval += self->xscale;
        }
        break;
    case 2: // COLOR2
        for (int x = 0; x < N_LEDS; ++x) {
            osp_fixp val = rescale(opensimplex2d_noise(&self->s[0], xval, self->yval));
            osp_fixp tint = rescale(opensimplex2d_noise(&self->s[1], xval, self->yval));
            for (int i = 0; i < 3; ++i) {
                data[x * 3 + i] = GAMMA_COR(OSP_MULT(self->col[0][i] * (OSP_FIXED(1) - tint) + (self->col[1][i] * tint), val));
            }
            xval += self->xscale;
        }
        break;
    case 3: // COLORN
        for (int x = 0; x < N_LEDS; ++x) {
            osp_fixp val = rescale(opensimplex2d_noise(&self->s[0], xval, self->yval));
            for (int i = 0; i < 3; ++i) {
                data[x * 3 + i] = GAMMA_COR(self->col[x][i] * val);
            }
            xval += self->xscale;
        }
        break;
    }
    self->yval += self->yscale;
    self->yval &= OSP_FIXED(256) - 1;

    *post_frame = true;
}

/** Rainbow effect state. */
struct rainbow_state {
    uint8_t offset;
    bool direction;
    uint8_t speed;
    uint8_t counter;
};

void rainbow_init(struct rainbow_state *self)
{
    self->offset = 0;
    self->direction = false;
    self->speed = 5;
    self->counter = 0;
}

void rainbow_frame(struct rainbow_state *self, uint8_t *data, bool *post_frame)
{
    // TODO: transition to fixed-point arithmetic,
    // render every frame even on slow speed
    // for more smoothness
    if (self->counter == 0) {
        self->counter = self->speed;

        for (int x = 0; x < N_LEDS; ++x) {
            rainbow_pixel(self->offset + x, &data[x * 3]);
        }
        *post_frame = true;

        if (self->direction) { // Increasing
            self->offset += 1;
            if (self->offset == 60) {
                self->offset = 0;
            }
        } else { // Decreasing
            self->offset -= 1;
            if (self->offset == 255) {
                self->offset = 60;
            }
        }
    } else {
        self->counter -= 1;
    }
}

/** Pulse effect state. */
struct pulse_state {
    /** Current pulse number. */
    int32_t count;
    /** Number times to pulse. */
    int32_t total;
    /** Current brightness (fixp). */
    int32_t brightness;
    /** Brightness delta (fixp). */
    int32_t delta;
    /** Pulse color. */
    uint8_t col[3];
};

void pulse_init(struct pulse_state *self)
{
    self->count = 0;
    self->total = 3;
    self->brightness = OSP_FIXED(0);
    self->delta = OSP_PRECISION / 25;
    self->col[0] = 255;
    self->col[1] = 0;
    self->col[2] = 255;
}

void pulse_frame(struct pulse_state *self, uint8_t *data, bool *post_frame)
{
    if (self->count >= self->total) {
        return;
    }

    uint8_t col[3];
    for (int i = 0; i < 3; ++i) {
        col[i] = OSP_FLOOR(self->col[i] * self->brightness);
    }

    for (int x = 0; x < N_LEDS; ++x) {
        data[x * 3 + 0] = col[0];
        data[x * 3 + 1] = col[1];
        data[x * 3 + 2] = col[2];
    }
    *post_frame = true;

    self->brightness += self->delta;
    if (self->brightness <= OSP_FIXED(0)) {
        self->brightness = OSP_FIXED(0);
        self->delta = -self->delta;
        self->count += 1;
    }
    if (self->brightness >= OSP_FIXED(1)) {
        self->brightness = OSP_FIXED(1);
        self->delta = -self->delta;
    }
}

/** Dot effect state. */
struct dot2_state {
    /** Foreground color. */
    uint8_t col_fg[3];
    /** Background color. */
    uint8_t col_bg[3];
    /** Mirrored halves. */
    bool mirrored;
    /** Frames per movement step. */
    int speed;
    /** Current light position. */
    int pos;
    /** Current movement direction. */
    bool direction;
    /** Counter for speed. */
    int counter;
};

void dot2_init(struct dot2_state *self)
{
    self->col_fg[0] = 200;
    self->col_fg[1] = 0;
    self->col_fg[2] = 255;
    self->col_bg[0] = 32;
    self->col_bg[1] = 32;
    self->col_bg[2] = 32;
    self->mirrored = true;
    self->speed = 5;
    self->pos = 0;
    self->direction = false;
    self->counter = 0;
}

void dot2_frame(struct dot2_state *self, uint8_t *data, bool *post_frame)
{
    if (self->counter == 0) {
        self->counter = self->speed;

        for (int x = 0; x < N_LEDS; ++x) {
            data[x * 3 + 0] = self->col_bg[0];
            data[x * 3 + 1] = self->col_bg[1];
            data[x * 3 + 2] = self->col_bg[2];
        }
        int pos1 = self->pos;
        if (pos1 >= 0 && pos1 < N_LEDS) {
            data[pos1 * 3 + 0] = self->col_fg[0];
            data[pos1 * 3 + 1] = self->col_fg[1];
            data[pos1 * 3 + 2] = self->col_fg[2];
        }

        int pos2 = N_LEDS - self->pos - 1;
        if (self->mirrored && pos2 >= 0 && pos2 < N_LEDS) {
            data[pos2 * 3 + 0] = self->col_fg[0];
            data[pos2 * 3 + 1] = self->col_fg[1];
            data[pos2 * 3 + 2] = self->col_fg[2];
        }

        *post_frame = true;

        if (self->direction) { // Increasing
            self->pos += 1;
            if (self->pos >= N_LEDS - 1) {
                self->pos = N_LEDS - 1;
                self->direction = false;
            }
        } else { // Decreasing
            self->pos -= 1;
            if (self->pos < 0) {
                self->pos = 0;
                self->direction = true;
            }
        }
    } else {
        self->counter -= 1;
    }
}

/**** Command handling ******************************************************/

/** Effect frame callback function type. */
typedef void (*mode_frame_func)(void *self, uint8_t *data, bool *post_frame);

// Current led state ("frame buffer").
uint8_t data[N_LEDS * 3] = {0};

// Current global mode.
bool post_frame = false;
mode_frame_func mode_frame = 0;
void *mode_state = 0;


static struct simplex_state simplex;
static struct rainbow_state rainbow;
static struct pulse_state pulse;
static struct dot2_state dot2;

/** Receive bytes from UART. Blocks until everything has been received.
 */
MAYBE_UNUSED static void uart_recv(uint8_t *data, uint32_t size)
{
    for (uint32_t i = 0; i < size; ++i) {
        while (!_getc(&data[i])) ;
    }
}

/** Entry Point for UART interrupt handler. */
static void handle_uart_interrupt()
{
    uint8_t cmd;
    if (!(UART0_REG(UART_REG_IP) & UART_IP_RXWM)) {
        // Only handle RX interrupt.
        return;
    }
    // TODO: When preemption is enabled, make sure to disable the timer
    // interrupt while updating effect state to avoid race behavior. Read all
    // data first, then update the fx state in one go. This will reduce the
    // time frame during which the timer interrupt is disabled, avoiding
    // visible interruption.
    // Currently this is not a problem: preemption is disabled unless the
    // interrupt handler explicitly enables it.
    //
    // To enable pre-emption on RISC-V, in the handler:
    //   - Save mpie and mpp in mstatus
    //   - Set mie
    //   - Execute preemptable part of handler
    //   - Before returning: disable mie, restore mpie and mpp
    //
    // The handler can explicitly choose which interrupts can
    // preempt it, by choosing which ones to enable. In this case,
    // be sure to restore MIP_??IP state at the end of the handler as well.
    //
    // I'm not sure this is worth it. Could structure this asynchronously to
    // minimize time in the handler.
    while (_getc(&cmd)) {
        write(1, "C", 1);
        switch(cmd) {
        case 'x': // Reset to initial color
            for (int i = 0; i < N_LEDS; i++) {
                data[i * 3 + 0] = INIT_COL[0];
                data[i * 3 + 1] = INIT_COL[1];
                data[i * 3 + 2] = INIT_COL[2];
            }
            mode_frame = 0;
            break;
        case 'o': // Off
            for (int i = 0; i < N_LEDS; i++) {
                data[i * 3 + 0] = 0;
                data[i * 3 + 1] = 0;
                data[i * 3 + 2] = 0;
            }
            mode_frame = 0;
            break;
        case 'f': {
            uart_recv(data, N_LEDS * 3);
            mode_frame = 0;
            } break;
        case 's': // Simplex plasma
            // TODO: could allocate the effect state here, there's no real reason
            // to keep it around in other modes. Or share a single block between modes.
            // On the other hand it would be nice to be able to compose (mix)
            // multiple effects in some cases, in which case they do need to be able
            // to run in parallel.
            uart_recv(&simplex.mode, 1);
            uart_recv(&simplex.col[0][0], N_LEDS * 3);
            mode_frame = (mode_frame_func)&simplex_frame;
            mode_state = (void*)&simplex;
            break;
        case 'r': // Rainbow
            // TODO: parameters?
            mode_frame = (mode_frame_func)&rainbow_frame;
            mode_state = (void*)&rainbow;
            break;
        case 'p': // Pulse
            // TODO: parameters?
            // pulse (once, N times, infinite), speed, color
            pulse_init(&pulse);
            mode_frame = (mode_frame_func)&pulse_frame;
            mode_state = (void*)&pulse;
            break;
        case '2': // Dot2
            // TODO: parameters?
            // dot2 color, bgcolor, speed
            pulse_init(&pulse);
            mode_frame = (mode_frame_func)&dot2_frame;
            mode_state = (void*)&dot2;
            break;
        }
    }
    post_frame = true;
}

/** Entry Point for Machine Timer Interrupt Handler. */
void handle_m_time_interrupt()
{
    clear_csr(mie, MIP_MTIP);

    set_frame_timer();

    if (mode_frame) {
        mode_frame(mode_state, data, &post_frame);
    }

    if (post_frame) {
        // Finally, actually update the leds.
        for (int i = 0; i < N_LEDS; ++i) {
            send_pixel(data[i * 3 + 0],  data[i * 3 + 1], data[i * 3 + 2]);
        }
        latch();

        post_frame = false;
    }

    // Re-enable the timer interrupt.
    set_csr(mie, MIP_MTIP);
}

int main(void)
{
    board_init();

    printf("Starting program...\n");
    printf("Core CPU freq: %ld Hz\n", get_cpu_freq());
    printf("Timer freq   : %ld Hz\n", get_timer_freq());

    printf("Case led control\n");
    printf("2021 Mara Huldra, loosely based on hifive1-neopixel by Curt Brune\n");

    // Configure SPI1 output function
    GPIO_REG(GPIO_IOF_SEL) &= ~IOF0_SPI1_MASK;
    GPIO_REG(GPIO_IOF_EN) |= IOF0_SPI1_MASK;

    // Configure SPI1
    SPI1_REG(SPI_REG_SCKDIV) = (get_cpu_freq() / SPI_HZ) / 2 - 1; // 0x00 Serial clock Divisor
    SPI1_REG(SPI_REG_SCKMODE) = 0; // Don't care about SCLK
    SPI1_REG(SPI_REG_CSMODE) = SPI_CSMODE_OFF; // Disable chip select
    SPI1_REG(SPI_REG_DCSSCK) = 0; // No extra delays
    SPI1_REG(SPI_REG_DINTERCS) = 0; // No extra delays
    SPI1_REG(SPI_REG_FMT) = SPI_FMT_PROTO(SPI_PROTO_S) // Single SPI through MOSI/MISO
                          | SPI_FMT_ENDIAN(SPI_ENDIAN_MSB) // Top bit first
                          | SPI_FMT_DIR(SPI_DIR_TX) // Transmit only
                          | SPI_FMT_LEN(8); // 8-bit units

    interrupt_setup();

    // Effect states.
    simplex_init(&simplex);
    rainbow_init(&rainbow);
    pulse_init(&pulse);
    dot2_init(&dot2);

    // Initial color.
    for (int i = 0; i < N_LEDS; i++) {
        data[i * 3 + 0] = INIT_COL[0];
        data[i * 3 + 1] = INIT_COL[1];
        data[i * 3 + 2] = INIT_COL[2];
    }
    post_frame = true;

    while (true) {
        // Everything happens in interrupts.
        __asm__ volatile ("wfi");
    }
}
