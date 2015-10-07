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

#include "periph/gating.h"
#include "periph/gpio.h"
#include "periph/uart.h"
#include "periph_conf.h"
#include "sched.h"
#include "board.h"

#define ENABLE_DEBUG (0)
#include "debug.h"

/* guard file in case no UART device was specified */
#if UART_NUMOF

/**
 * @brief Allocate memory to store the rx callback.
 */
static uart_isr_ctx_t uart_ctx[UART_NUMOF];

static inline USIC_CH_TypeDef *_usic(uart_t uart)
{
    return uart_dev[uart].usic;
}

int uart_init(uart_t uart, uint32_t baudrate,
              uart_rx_cb_t rx_cb, void *arg)
{
    USIC_CH_TypeDef *usic = _usic(uart);
    gpio_t uart_tx = uart_dev[uart].tx;
    gpio_t uart_rx = uart_dev[uart].rx;

    /* clear gating, enable clock */
    GATING_CLEAR(USIC0);

    /* enable the USIC module */
    usic->KSCFG |= USIC_CH_KSCFG_MODEN_Msk | USIC_CH_KSCFG_BPMODEN_Msk;

    /* TODO: Support more rates */
    if ((115200 % baudrate) != 0 || (baudrate < 28800)) {
        return -2;
    }

    /* "The fractional divider register FDR allows the generation of the
       internal frequency f_FD, that is derived from the system clock f_PB." */
    /* uint32_t fdr_step = 590; */
    uint32_t fdr_step = 811;

    /* clear register */
    usic->FDR &= ~(USIC_CH_FDR_DM_Msk | USIC_CH_FDR_STEP_Msk);

    /* enable fractional divider mode (2) and set step value. */
    /* f_FD = f_PB * (frd_step / 1024) */
    usic->FDR |= (2 << USIC_CH_FDR_DM_Pos) | (fdr_step << USIC_CH_FDR_STEP_Pos);

    /* Pre-Divider for Time Quanta Counter (2 Bits) */
    uint8_t brg_pctq = (115200 / baudrate);
    /* Denominator for Time Quanta Counter (5 Bits) */
    uint8_t brg_dctq = 10;
    /* Divider Factor to Generate f_pdiv */
    /* uint8_t brg_pdiv = 16; */
    uint8_t brg_pdiv = 22;

    /* clear CLKSEL -> fractional divider is used */
    /* clear PPEN   -> 2:1 divider is disabled */
    usic->BRG &= ~(USIC_CH_BRG_PCTQ_Msk | USIC_CH_BRG_DCTQ_Msk  |
                   USIC_CH_BRG_PDIV_Msk | USIC_CH_BRG_PPPEN_Msk |
                   USIC_CH_BRG_CLKSEL_Msk);

    usic->BRG |= (((brg_pctq - 1) << USIC_CH_BRG_PCTQ_Pos) |
                  ((brg_dctq - 1) << USIC_CH_BRG_DCTQ_Pos) |
                  ((brg_pdiv - 1) << USIC_CH_BRG_PDIV_Pos));

    /* baudrate = f_FD / brg_pdiv / brg_pctq / brg_dctq */

    /* clear shift control register */
    usic->SCTR &= ~(USIC_CH_SCTR_TRM_Msk | USIC_CH_SCTR_FLE_Msk |
                    USIC_CH_SCTR_WLE_Msk);

    /* configure the shift register: FLE = frame length, WLE = word
       length, TRM = transmission mode, PDL = passive data level. */
    usic->SCTR |= ((1 << USIC_CH_SCTR_TRM_Pos) |
                   (7 << USIC_CH_SCTR_FLE_Pos) |
                   (7 << USIC_CH_SCTR_WLE_Pos) |
                   (1 << USIC_CH_SCTR_PDL_Pos));

    /* clear transmit control/status register */
    usic->TCSR &= ~(USIC_CH_TCSR_TDEN_Msk);

    /* configure the transmit control register */
    /* TDEN: "This bit field controls the gating of the transmission start of
       the data word in the transmit buffer TBUF." 1 -> "A transmission of the
       data word in TBUF can be started if TDV = 1" */
    /* TDSSM: 1 -> "The data word in TBUF is considered as invalid after it has
       been loaded into the shift register." */
    usic->TCSR |= USIC_CH_TCSR_TDSSM_Msk | (1 << USIC_CH_TCSR_TDEN_Pos);

    /* clear protocol control register */
    usic->PCR &= ~(USIC_CH_PCR_ASCMode_SP_Msk |
                   USIC_CH_PCR_ASCMode_PL_Msk |
                   USIC_CH_PCR_ASCMode_STPB_Msk);

    /* configure the protocol control register */
    /* SMD: 1 -> "Three samples are taken per bit time and a majority decision
       is made" */
    /* STPB: 0 -> one stop bit */
    /* SP: "This bit field defines the sample point of the bit value. The sample
       point must not be located outside the programmed bit timing (PCR.SP ≤
       BRG.DCTQ). */
    /* PL: 0 -> "Pulse Length is equal to the bit length" */
    usic->PCR |= ((USIC_CH_PCR_ASCMode_SMD_Msk) |
                  (4 << USIC_CH_PCR_ASCMode_SP_Pos));

    /* clear the channel control register */
    /* PM: 0 -> no parity */
    usic->CCR &= ~(USIC_CH_CCR_PM_Msk | USIC_CH_CCR_MODE_Msk);

    /* configure the channel control register */
    /* MODE: 2 -> ASC mode */
    usic->CCR |= (2 << USIC_CH_CCR_MODE_Pos);

    /* Data selection for input signal */
    usic->DX0CR |= uart_dev[uart].dsel;

    /* P2.2 as input */
    gpio_init(uart_rx, GPIO_DIR_IN, GPIO_NOPULL);

    /* Initialize UART_TX */
    gpio_init(uart_tx, (GPIO_DIR_OUT | uart_dev[uart].asel), GPIO_NOPULL);

    /* register callback */
    uart_ctx[uart].rx_cb = rx_cb;
    uart_ctx[uart].arg = usic;

    /* enable corresponding IRQ (both USICs use the same set of 6
     * IRQs, so we could route each UART receive interrupt to its own
     * ISR but they're precious and SPI might need some as well */
    NVIC_SetPriority(USIC0_5_IRQn, UART_IRQ_PRIO);
    NVIC_EnableIRQ(USIC0_5_IRQn);

    /* set receive interrupt pointer to SR5 */
    usic->INPR &= ~USIC_CH_INPR_RINP_Msk;
    usic->INPR |= 0x5 << USIC_CH_INPR_RINP_Pos;

    /* set transmit shift interrupt pointer to SR5 */
    usic->INPR &= ~USIC_CH_INPR_TSINP_Msk;
    usic->INPR |= 0x5 << USIC_CH_INPR_TSINP_Pos;

    /* enable receive interrupt on peripheral */
    usic->CCR |= 1 << USIC_CH_CCR_RIEN_Pos;

    return 0;
}

void uart_tx(uart_t uart)
{
    _usic(uart)->CCR |= 1 << USIC_CH_CCR_TSIEN_Pos;
}

void uart_write(uart_t uart, const char *data, size_t len)
{
    USIC_CH_TypeDef *usic = _usic(uart);
    for (int i = 0; i < len; i++) {
        /* wait for transmission to be ready */
        while (_usic(uart)->TCSR & USIC_CH_TCSR_TDV_Msk);
        usic->TBUF[0] = data[i];
    }
}

int uart_read_blocking(uart_t uart, char *data)
{
    /* wait for data to be available */
    while (!(_usic(uart)->RBUFSR & USIC_CH_RBUFSR_RDV0_Msk) &&
           !(_usic(uart)->RBUFSR & USIC_CH_RBUFSR_RDV1_Msk));

    *(data) = (char)_usic(uart)->RBUF;
    return 1;
}

void uart_write_blocking(uart_t uart, char data)
{
    uart_write(uart, (const char *)&data, 1);
}

void uart_poweron(uart_t uart)
{
    /* clear clock */
    GATING_CLEAR(USIC0);

    /* enable module & set normal mode 0 */
    _usic(uart)->KSCFG =
        ((_usic(uart)->KSCFG & ~(USIC_CH_KSCFG_NOMCFG_Msk |
                                 USIC_CH_KSCFG_MODEN_Pos)) |
         (USIC_CH_KSCFG_BPNOM_Msk | USIC_CH_KSCFG_BPMODEN_Msk));
}

void uart_poweroff(uart_t uart)
{
    USIC_CH_TypeDef *usic = _usic(uart);

    /* disable BUSY reporting for receive */
    usic->PCR &= ~USIC_CH_PCR_ASCMode_RSTEN_Msk;
    /* enable BUSY reporting for transmit */
    usic->PCR |=  USIC_CH_PCR_ASCMode_TSTEN_Msk;
    /* request stop mode 1 */
    usic->KSCFG |= ((0x3 << USIC_CH_KSCFG_NOMCFG_Pos) |
                    USIC_CH_KSCFG_BPNOM_Msk);
    /* wait for transmissions to finish */
    while (usic->PSR & USIC_CH_PSR_ASCMode_BUSY_Msk);
    /* disable BUSY reporting for transmit */
    usic->PCR &= ~USIC_CH_PCR_ASCMode_TSTEN_Msk;
    /* disable USIC module */
    usic->KSCFG |= ((0 << USIC_CH_KSCFG_MODEN_Pos) |
                    USIC_CH_KSCFG_BPMODEN_Msk);
    /* gate clock */
    GATING_SET(USIC0);
}

#if UART_0_EN
void isr_usic5(void)
{
    USIC_CH_TypeDef *usic = _usic(0);

    LED_1_TOGGLE;

    if (usic->PSR & USIC_CH_PSR_RIF_Msk) {
        char data = (char)usic->RBUF;
        uart_ctx[0].rx_cb(uart_ctx[0].arg, data);
        usic->PSR |= USIC_CH_PSR_RIF_Msk;
    }

    if (sched_context_switch_request) {
        thread_yield();
    }
}
#endif

#endif /* UART_NUMOF */
