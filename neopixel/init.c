#include <stdio.h>

#include "encoding.h"

/*
 *
 * Copyright 2018 Curt Brune <curt@brune.net>
 * All rights reserved.
 *
 * Most of this was lifted from the Freedom-E-SDK init code.
 * Copyright 2016 SiFive, Inc.
 *
 */

#include "platform.h"

static void uart_init(size_t baud_rate)
{
    GPIO_REG(GPIO_OUTPUT_VAL) |= IOF0_UART0_MASK;
    GPIO_REG(GPIO_OUTPUT_EN)  |= IOF0_UART0_MASK;
    GPIO_REG(GPIO_IOF_SEL) &= ~IOF0_UART0_MASK;
    GPIO_REG(GPIO_IOF_EN) |= IOF0_UART0_MASK;
    UART0_REG(UART_REG_DIV) = get_cpu_freq() / baud_rate - 1;
    UART0_REG(UART_REG_TXCTRL) = UART_TXEN;
    UART0_REG(UART_REG_RXCTRL) = UART_RXEN;

    // Wait a bit to avoid corruption on the UART.
    // (In some cases, switching to the IOF can lead
    // to output glitches, so need to let the UART
    // reciever time out and resynchronize to the real
    // start of the stream.
    volatile int i=0;
    while(i < 10000) {i++;}
}

int _getc(uint8_t *c)
{
    uint32_t val = (uint32_t) UART0_REG(UART_REG_RXFIFO);
    if (val & 0x80000000) {
        return 0;
    } else {
        *c =  val & 0xFF;
        return 1;
    }
}

/** Trap entry point (provided by BSP). */
extern void trap_entry();

void board_init()
{
    // Make sure traps and interrupts go to the handler. Do this as first thing
    // possible, to prevent unexpected traps from jumping into the woods.
    write_csr(mtvec, &trap_entry);

    // Make sure the HFROSC is on before the next line:
    PRCI_REG(PRCI_HFROSCCFG) |= ROSC_EN(1);
    // Run off 16 MHz Crystal for accuracy. Note that the
    // first line is
    PRCI_REG(PRCI_PLLCFG) = (PLL_REFSEL(1) | PLL_BYPASS(1));
    PRCI_REG(PRCI_PLLCFG) |= (PLL_SEL(1));
    // Turn off HFROSC to save power
    PRCI_REG(PRCI_HFROSCCFG) &= ~(ROSC_EN(1));

    uart_init(115200);
}
