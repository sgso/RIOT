/*
 * Copyright (C) 2015 Sebastian Sontberg <sebastian@sontberg.de>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup         cpu_xmc1000
 * @{
 *
 * @file
 * @brief           CPU specific definitions for internal peripheral handling
 *
 * @author          Sebastian Sontberg <sebastian@sontberg.de>
 *
 * @}
 */

#ifndef CPU_PERIPH_H_
#define CPU_PERIPH_H_

#include "cpu.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   Define a custom type for GPIO pins
 * @{
 */
#define HAVE_GPIO_T
typedef uint8_t gpio_t;
/** @} */

/**
 * @brief   Mandatory function for defining a GPIO pin
 * @{
 */
#define GPIO_PIN(Port, Pin)    (Port | Pin)
/** @} */

/**
 * @brief Define a custom type for GPIO pins attached to an alternative
 * output function or routed to an input (from or to a peripheral).
 *  @{
 */
typedef uint16_t gpio_alt_t;
/** @} */

/**
 * @brief   Macro for defining a gpio_alt_t
 * @{
 */
#define GPIO_ALT(Port, Pin, Out)    (((Out) << 8) | (GPIO_PIN(Port, Pin)))
/** @} */

/**
 * @brief   Definition of a fitting UNDEF value
 * @{
 */
#define GPIO_UNDEF         (0xff)
/** @} */

/**
 * @brief   Available PORTS on xmc1100
 * @{
 */
enum {
    P0 = 0,
    P1 = 0x10,
    P2 = 0x20
};
/** @} */

/**
 * @brief   Override directional configuration values
 * @{
 */
#define HAVE_GPIO_DIR_T
typedef enum {
    GPIO_DIR_IN         = 0,    /**< configure pin as input */
    GPIO_DIR_IN_INV     = 4,    /**< configure pin as inverted input */
    GPIO_DIR_OUT        = 16,   /**< configure pin as output */
    GPIO_DIR_OPEN_DRAIN = 24,   /**< configure pin as open-drain output */
} gpio_dir_t;
/** @} */

/**
 * @brief   Alternative output function values
 * @details
 *
 * These are for drivers that need to set an output pin to the output
 * of a peripheral. They are meant to be OR'd with gpio_dir_t. See
 * Table 17-5 in XMC1100 Reference Manual
 * @{
 */
typedef enum {
    GPIO_ALT_OUT_1   = 1,
    GPIO_ALT_OUT_2   = 2,
    GPIO_ALT_OUT_3   = 3,
    GPIO_ALT_OUT_4   = 4,
    GPIO_ALT_OUT_5   = 5,
    GPIO_ALT_OUT_6   = 6,
    GPIO_ALT_OUT_7   = 7,
} gpio_alt_output_t;
/** @} */

/**
 * @brief   Override pull up configuration values
 * @{
 */
#define HAVE_GPIO_PP_T
typedef enum {
    GPIO_NOPULL   = 0,      /**< do not use internal pull resistors */
    GPIO_PULLDOWN = 1,      /**< enable internal pull-down resistor */
    GPIO_PULLUP   = 2,      /**< enable internal pull-up resistor   */
} gpio_pp_t;
/** @} */

/**
 * @brief   Override active flank configuration values
 * @{
 */
#define HAVE_GPIO_FLANK_T
typedef enum {
    GPIO_RISING = 1,        /**< emit interrupt on rising flank */
    GPIO_FALLING = 2,       /**< emit interrupt on falling flank */
    GPIO_BOTH = 3           /**< emit interrupt on both flanks */
} gpio_flank_t;
/** @} */

/**
 * @brief   Definitions for the Event Request Unit (ERU)
 * @{
 */
#define A1 (2)
#define B0 (0)
#define B1 (1)
/** @} */

/**
 * @brief   Macro to map event sources of the ERU
 * @{
 */
#define ERU(No, Input) { No, Input }
/** @} */

/**
 * @brief   Type for mapping event sources of the ERU
 * @{
 */
typedef struct __attribute__((packed)) {
    unsigned exs : 2;           /**< event source selector (see section 6.9.1) */
    unsigned connection : 2;    /**< event source value */
} eru_input_t;
/** @} */

/**
 * @brief GPIO mapping to Event Request Unit (ERU) inputs
 * @{
 */
static const eru_input_t eru_mapping[] = {
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

#endif /* CPU_PERIPH_H_ */
