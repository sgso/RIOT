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

void uart_isr_rx(uint8_t);

int uart_init(uart_t dev, uint32_t baudrate, uart_rx_cb_t rx_cb, void *arg)
{
    const uint8_t u_index = uart_instance[dev].channel * 3;

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
    uart_ctx[dev].arg   = (void*)&uart_instance[dev].channel;
    
    /* setup & start the USIC channel */
    USIC_CH_TypeDef *usic = usic_init((usic_channel_t *)&uart_instance[dev],
                                      brg, fdr,
                                      &uart_isr_rx, NULL, NULL, UART_IRQ_PRIO);
    
    usic->TBCTR =
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
        (u_index << USIC_CH_TBCTR_STBINP_Pos) |
        /* interrupt initially disabled */
        (0 << USIC_CH_TBCTR_STBIEN_Pos);

    usic->RBCTR =
        /* RxFIFO size configurable */
        (uart_instance[dev].fifo.rx_size << USIC_CH_RBCTR_SIZE_Pos) |
        (uart_instance[dev].fifo.rx_dptr << USIC_CH_RBCTR_DPTR_Pos) |
        (1 << USIC_CH_RBCTR_LIMIT_Pos) |
        (0 << USIC_CH_RBCTR_LOF_Pos) |
        (3 << USIC_CH_RBCTR_RCIM_Pos) |

        (1 << USIC_CH_RBCTR_SRBTM_Pos) |
        (1 << USIC_CH_RBCTR_SRBIEN_Pos) |
        (1 << USIC_CH_RBCTR_SRBTEN_Pos) |
        (u_index << USIC_CH_RBCTR_SRBINP_Pos) |
        /* RCI mode: enabled */
        (1 << USIC_CH_RBCTR_RNM_Pos);

    return 0;
}

void uart_write(uart_t dev, const uint8_t *data, size_t len)
{
    USIC_CH_TypeDef *usic = usic_get(uart_instance[dev].channel);

    const unsigned fifo_size = (1 << uart_instance[dev].fifo.tx_size);

    while (len) {
        size_t nbytes = len > fifo_size ? fifo_size : len;

        usic_lock(uart_instance[dev].channel);

        /* disable interrupt for standard transmit buffer event */
        usic->TBCTR &= ~USIC_CH_TBCTR_STBIEN_Msk;

        /* write up to fifo_size bytes into transmit FIFO */
        for (int i = 0; i < nbytes; i++) {
            usic->IN[0] = data[i];
        }

        /* enable interrupt for standard transmit buffer event */
        usic->TBCTR |= (1 << USIC_CH_TBCTR_STBIEN_Pos);

        data += nbytes;
        len -= nbytes;
    }
}

void uart_poweron(uart_t dev)
{
    USIC_CH_TypeDef *usic = usic_get(uart_instance[dev].channel);

    /* clear clock */
    GATING_CLEAR(USIC0);

    /* enable module & set normal mode 0 */
    usic->KSCFG = ((usic->KSCFG & ~(USIC_CH_KSCFG_NOMCFG_Msk | USIC_CH_KSCFG_MODEN_Pos)) |
                   (USIC_CH_KSCFG_BPNOM_Msk | USIC_CH_KSCFG_BPMODEN_Msk));
}

void uart_poweroff(uart_t dev)
{
    USIC_CH_TypeDef *usic = usic_get(uart_instance[dev].channel);

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
    USIC_CH_TypeDef *usic = usic_get(channel);

    if (usic->TRBSR & USIC_CH_TRBSR_STBI_Msk) {
        usic_unlock(channel);
    }

    if (usic->TRBSR & USIC_CH_TRBSR_RBFLVL_Msk) {
        uart_ctx[channel].rx_cb(uart_ctx[channel].arg, (char)usic->OUTR);
    }

    if (sched_context_switch_request) {
        thread_yield();
    }
}
