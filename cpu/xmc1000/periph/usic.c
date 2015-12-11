/*
 * Copyright (C) 2015 Sebastian Sontberg <sebastian@sontberg.de>
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup         cpu_xmc1000
 * @{
 *
 * @file
 * @brief           Code shared by peripheral drivers using the USIC
 *
 * @author          Sebastian Sontberg <sebastian@sontberg.de>
 *
 * @}
 */

#include "cpu.h"

#include "periph/gating.h"
#include "periph/gpio.h"
#include "periph/usic.h"

static void (* usic_mplex[6])(uint8_t);

static mutex_t locks[2] = {MUTEX_INIT, MUTEX_INIT};

void isr_usic(void) {
    const uint8_t irqn = __get_IPSR() - 25;
    usic_mplex[irqn](irqn > 2 ? 1 : 0);
}

int usic_lock(uint8_t channel)
{
    mutex_lock(&locks[channel]);
    return 0;
}

int usic_unlock(uint8_t channel)
{
    mutex_unlock(&locks[channel]);
    return 0;
}

USIC_CH_TypeDef *usic_init(const usic_channel_t *usic_ch,
                           const usic_brg_t brg,
                           const usic_fdr_t fdr,
                           void (* irq0)(uint8_t),
                           void (* irq1)(uint8_t),
                           void (* irq2)(uint8_t),
                           const uint8_t irqp)

{
    USIC_CH_TypeDef *usic = usic_get(usic_ch->channel);

    const usic_mode_t *controls = usic_ch->mode;

    /* initialize input pin */
    gpio_init(usic_ch->rx_pin & 0xff, (usic_ch->rx_pin >> 8), 0);

    /* clear gating, enable clock */
    GATING_CLEAR(USIC0);

    /* enable the USIC module */
    usic->KSCFG =
        (1 << USIC_CH_KSCFG_MODEN_Pos) |
        (1 << USIC_CH_KSCFG_BPMODEN_Pos);

    /* clear channel control register (reset mode bit) */
    usic->CCR = 0;

    /* configure fractional divider */
    usic->FDR = fdr.reg;

    /* configure baud rate generator */
    usic->BRG = brg.reg;

    /* configure input stages DX0 & DX2 */
    usic->DX0CR = usic_ch->inputs.dx0 << USIC_CH_DX0CR_DSEL_Pos;
    usic->DX1CR = usic_ch->inputs.dx1 << USIC_CH_DX1CR_DSEL_Pos;
    usic->DX2CR = usic_ch->inputs.dx2 << USIC_CH_DX2CR_DSEL_Pos;
    usic->DX3CR = usic_ch->inputs.dx3 << USIC_CH_DX3CR_DSEL_Pos;

    /* shift control register */
    usic->SCTR = controls->sctr;
    /* transmit control/status register */
    usic->TCSR = controls->tcsr;
    /* protocol control register */
    usic->PCR  = controls->pcr;
    /* channel control register */
    usic->CCR  = controls->ccr;

    if (irq0) {
        usic_mplex[usic_ch->channel * 3 + 0] = irq0;
        NVIC_SetPriority(USIC0_0_IRQn + usic_ch->channel * 3, UART_IRQ_PRIO);
        NVIC_EnableIRQ(USIC0_0_IRQn + usic_ch->channel * 3);
    }

    if (irq0) {
        usic_mplex[usic_ch->channel * 3 + 1] = irq1;
        NVIC_SetPriority(USIC0_1_IRQn + usic_ch->channel * 3, UART_IRQ_PRIO);
        NVIC_EnableIRQ(USIC0_1_IRQn + usic_ch->channel * 3);
    }

    if (irq0) {
        usic_mplex[usic_ch->channel * 3 + 2] = irq2;
        NVIC_SetPriority(USIC0_2_IRQn + usic_ch->channel * 3, UART_IRQ_PRIO);
        NVIC_EnableIRQ(USIC0_2_IRQn + usic_ch->channel * 3);
    }

    /* ready to activate output pins */
    gpio_init(usic_ch->tx_pin & 0xff,
              GPIO_DIR_OUT | (usic_ch->tx_pin >> 8), 0);

    return usic;
}
