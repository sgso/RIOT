/*
 * Copyright (C) 2015 Sebastian Sontberg <sebastian@sontberg.de>
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     boards_xmc2go
 * @{
 *
 * @file
 * @brief       Peripheral MCU configuration for the xmc-2go board
 *
 * @author      Sebastian Sontberg <sebastian@sontberg.de>
 */

#ifndef PERIPH_CONF_H_
#define PERIPH_CONF_H_

#include "cpu.h"
#include "periph_cpu.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name RTC configuration
 * @{
 */
#define RTC_NUMOF          (1U)
#define RTC_IRQ_PRIO       CPU_DEFAULT_IRQ_PRIO
#define RTC_DIVIDER        (0x7fff)
/** @} */

/**
 * @name PRNG configuration
 * @{
 */
#define RANDOM_NUMOF       (1U)
/** @} */

/**
 * @name ADC configuration
 * @{
 */
#define ADC_NUMOF          (2U)
#define ADC_0_EN           1
#define ADC_1_EN           0
#define ADC_MAX_CHANNELS   8
/** @} */

/**
 * @name Timer configuration
 * @{
 */
#define TIMER_NUMOF        (1)
#define TIMER_0_EN         (1)
#define TIMER_0_CHANNELS   (2)
#define TIMER_0_MAX_VALUE  (0xffffffff)
#define TIMER_IRQ_PRIO     CPU_DEFAULT_IRQ_PRIO

#define XTIMER_BACKOFF     (40)
#define XTIMER_ISR_BACKOFF (30)
/** @} */

/**
 * @name UART configuration
 * @{
 */
#define UART_NUMOF         (1U)
#define UART_0_EN          (1)
#define UART_IRQ_PRIO      CPU_DEFAULT_IRQ_PRIO

static const uart_conf_t uart_dev[] = {
    {
        // .usic = USIC0_CH1,     // USIC
        // .tx = GPIO(P1, 3),     // TX pin
        // .rx = GPIO(P1, 2),     // RX pin
        // .dsel = 1,             // configure input control register 0
        // .asel = GPIO_ALT_OUT_7 // to use data input DXnG (see page 451)
        .usic = USIC0_CH1, // Usic channel 1
        .tx = GPIO(P1,2), // TX Pin of XMC; connected to RX at USB bridge
        .rx = GPIO(P1,3), // RX Pin of XMC; connected to TX at USB bridge
        .dsel = 0, // USIC0_CH1.DX0A : P1.3 (Table 14-24, Chapter 14.12)
        .asel = GPIO_ALT_OUT_7 // P1.2 : USIC0_CH1.DOUT0 (Table 18-14, Chapter 18.10.3)
    }
};
/** @} */

/**
 * @name GPIO mapping to Event Request Unit (ERU) inputs
 * @{
 */
static const eru_input_t eru_mapping[12] = {
    ERU(0, B0),                 /* P2.0 */
    ERU(1, B0),                 /* P2.1 */
    ERU(1, B1),                 /* P2.2 */
    ERU(1, B1),                 /* P2.3 */
    ERU(0, A1),                 /* P2.4 */
    ERU(1, A1),                 /* P2.5 */
    ERU(2, A1),                 /* P2.6 */
    ERU(3, A1),                 /* P2.7 */
    ERU(3, B1),                 /* P2.8 */
    ERU(3, B0),                 /* P2.9 */
    ERU(2, B0),                 /* P2.10 */
    ERU(2, B1)                  /* P2.11 */
};
/** @} */

#ifdef __cplusplus
}
#endif

#endif /* PERIPH_CONF_H_ */
