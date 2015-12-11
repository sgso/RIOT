/*
 * Copyright (C) 2015 Sebastian Sontberg <sebastian@sontberg.de>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     cpu_xmc1000
 * @{
 *
 * @file
 * @brief       Low-level UART driver implementation
 *
 * @author      Sebastian Sontberg <sebastian@sontberg.de>
 *
 * @}
 */

#include <string.h>
#include "mutex.h"

#include "periph_conf.h"

#include "periph/gating.h"
#include "periph/gpio.h"
#include "periph/usic.h"
#include "periph/uart.h"

#include "sched.h"

#define ENABLE_DEBUG (0)
#include "debug.h"

/**
 * @brief Allocate memory to store the rx callback.
 */
static uart_isr_ctx_t uart_ctx[UART_NUMOF];

#if UART_ENABLE_FIFOS

static mutex_t tx_mutex[UART_NUMOF] = {MUTEX_INIT};

#endif /* UART_ENABLE_FIFOS */

void uart_isr_rx(uint8_t);

int uart_init(uart_t dev, uint32_t baudrate, uart_rx_cb_t rx_cb, void *arg)
{
    const usic_channel_t *usic_ch = (usic_channel_t *)&uart_instance[dev];

    /* gives 0 for USIC0_CH0 & 1 for USIC0_CH1 */
    const uint8_t u_index = usic_index(usic_ch) * 3;
        
    const usic_fdr_t fdr = {
        /* STEP value: (944/1024) * 32Mhz = 29.5Mhz (16 x 1.8432 Mhz) */
        .field.step = 944,
        /* DM: fractional divider mode */
        .field.dm	= 2
    };

    const usic_brg_t brg = {
        /* CTQIN: values below relate to f[PDIV] */
        .field.ctqsel	= USIC_CTQIN_PDIV,
        /* DCTQ: denominator for time quanta counter (8 pulses) */
        .field.dctq		= 7,
        /* PCTQ: pre-divider for time quanta counter (1) */
        .field.pctq		= 0,
        /* PDIV: divider factor to generate f[PDIV] */
        .field.pdiv		= ((((DCO1_FREQUENCY / 2) / 1024) * fdr.field.step) /
                                   (baudrate * (brg.field.dctq + 1)))
    };

    /* register receive callback */
    uart_ctx[dev].rx_cb = rx_cb;
    uart_ctx[dev].arg   = usic_ch->usic;

    /* register uart_isr_rx for receiving and enable it */
    usic_mplex[u_index] = &uart_isr_rx;
    NVIC_SetPriority(USIC0_0_IRQn + u_index, UART_IRQ_PRIO);
    NVIC_EnableIRQ(USIC0_0_IRQn + u_index);

    /* setup & start the USIC channel */
    usic_init(usic_ch, brg, fdr);

#if UART_ENABLE_FIFOS

    uart_instance[dev].usic->TBCTR =
        (uart_instance[dev].fifo.tx_size << USIC_CH_TBCTR_SIZE_Pos) |
        (uart_instance[dev].fifo.tx_dptr << USIC_CH_TBCTR_DPTR_Pos) |
        (1 << USIC_CH_TBCTR_LIMIT_Pos)  |
        /* A standard transmit buffer event fires when TRBSR.TBFLVL ==
         * TBCTR.LIMIT && TRBSR.TBFLVL decreases due to transmission:
         * which means, the FIFO is empty ;) */
        (0 << USIC_CH_TBCTR_LOF_Pos)  |
        /* STBT is cleared by HW when TRBSR.TBFLVL == TBCTR.LIMIT:
         *  which means, we filled it with at least 1 word after the
         *  event. */
        (0 << USIC_CH_TBCTR_STBTM_Pos)  |
        /* standard TxFIFO event trigger: activated */
        (1 << USIC_CH_TBCTR_STBTEN_Pos) |
        /* routed to isr_usic0 */
        (0 << USIC_CH_TBCTR_STBINP_Pos) |
        /* interrupt initially disabled */
        (0 << USIC_CH_TBCTR_STBIEN_Pos);

    uart_instance[dev].usic->RBCTR =
        /* RxFIFO size configurable */
        (uart_instance[dev].fifo.rx_size << USIC_CH_RBCTR_SIZE_Pos) |
        (uart_instance[dev].fifo.rx_dptr << USIC_CH_RBCTR_DPTR_Pos) |
        (3 << USIC_CH_RBCTR_RCIM_Pos) |
        (1 << USIC_CH_RBCTR_LIMIT_Pos) |
        (1 << USIC_CH_RBCTR_SRBTM_Pos) |
        (1 << USIC_CH_RBCTR_SRBIEN_Pos) |
        (1 << USIC_CH_RBCTR_SRBTEN_Pos) |
        (5 << USIC_CH_RBCTR_SRBINP_Pos) |
        /* RCI mode: enabled */
        (1 << USIC_CH_RBCTR_RNM_Pos) |
        (0 << USIC_CH_RBCTR_LOF_Pos);

#endif /* UART_ENABLE_FIFOS */

    return 0;
}

void uart_write(uart_t dev, const uint8_t *data, size_t len)
{
    USIC_CH_TypeDef *usic = uart_instance[dev].usic;

#if UART_ENABLE_FIFOS

    const unsigned fifo_size = (1 << uart_instance[dev].fifo.tx_size);

    while (len) {
        size_t nbytes = len > fifo_size ? fifo_size : len;

        mutex_lock(&tx_mutex[dev]);

        /* disable interrupt for standard transmit buffer event */
        USIC0_CH0->TBCTR &= ~USIC_CH_TBCTR_STBIEN_Msk;

        /* write up to fifo_size bytes into transmit FIFO */
        for (int i = 0; i < nbytes; i++) {
            usic->IN[0] = data[i];
        }

        /* enable interrupt for standard transmit buffer event */
        uart_instance[dev].usic->TBCTR |= (1 << USIC_CH_TBCTR_STBIEN_Pos);

        data += nbytes;
        len -= nbytes;
    }

#else

    for (int i = 0; i < len; i++) {
        while (usic->TCSR & USIC_CH_TCSR_TDV_Msk) {
            /* wait for transmission to be ready */
        }

        usic->TBUF[0] = data[i];
    }

#endif /* UART_ENABLE_FIFOS */
}

void uart_poweron(uart_t uart)
{
    USIC_CH_TypeDef *usic = uart_instance[uart].usic;

    /* clear clock */
    GATING_CLEAR(USIC0);

    /* enable module & set normal mode 0 */
    usic->KSCFG = ((usic->KSCFG & ~(USIC_CH_KSCFG_NOMCFG_Msk | USIC_CH_KSCFG_MODEN_Pos)) |
                   (USIC_CH_KSCFG_BPNOM_Msk | USIC_CH_KSCFG_BPMODEN_Msk));
}

void uart_poweroff(uart_t uart)
{
    USIC_CH_TypeDef *usic = uart_instance[uart].usic;

    /* disable BUSY reporting for receive */
    usic->PCR &= ~USIC_CH_PCR_ASCMode_RSTEN_Msk;
    /* enable BUSY reporting for transmit */
    usic->PCR |=  USIC_CH_PCR_ASCMode_TSTEN_Msk;
    /* request stop mode 1 */
    usic->KSCFG |= ((0x3 << USIC_CH_KSCFG_NOMCFG_Pos) |
                    USIC_CH_KSCFG_BPNOM_Msk);
    /* wait for transmissions to finish */
    while (usic->PSR & USIC_CH_PSR_ASCMode_BUSY_Msk) ;
    /* disable BUSY reporting for transmit */
    usic->PCR &= ~USIC_CH_PCR_ASCMode_TSTEN_Msk;
    /* disable USIC module */
    usic->KSCFG |= ((0 << USIC_CH_KSCFG_MODEN_Pos) |
                    USIC_CH_KSCFG_BPMODEN_Msk);
    /* gate clock */
    GATING_SET(USIC0);
}

void uart_isr_rx(uint8_t channel)
{
    USIC_CH_TypeDef *usic = uart_instance[UART_DEV(0)].usic;

#if UART_ENABLE_FIFOS

    if (usic->TRBSR & USIC_CH_TRBSR_STBI_Msk) {
        mutex_unlock(&tx_mutex[UART_DEV(0)]);
    }

    if (usic->TRBSR & USIC_CH_TRBSR_RBFLVL_Msk) {
        uart_ctx[UART_DEV(0)].rx_cb(uart_ctx[UART_DEV(0)].arg, (char)usic->OUTR);
    }

#else

    if (usic->PSR & USIC_CH_PSR_RIF_Msk) {
        uart_ctx[UART_DEV(0)].rx_cb(uart_ctx[UART_DEV(0)].arg, (char)usic->RBUF);
        usic->PSCR = USIC_CH_PSCR_CRIF_Msk;
    }

#endif /* UART_ENABLE_FIFOS */
    if (sched_context_switch_request) {
        thread_yield();
    }
}
