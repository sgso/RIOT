/*
 * Copyright (C) 2015 Sebastian Sontberg <sebastian@sontberg.de>
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     cpu_xmc1000
 * @{
 *
 * @file
 * @brief       Low-level SPI driver implementation
 *
 * @author      Sebastian Sontberg <sebastian@sontberg.de>
 *
 * @}
 */

#include <stdint.h>

#include "cpu.h"
#include "mutex.h"
#include "periph/gpio.h"
#include "periph/spi.h"
#include "periph/usic.h"
#include "periph_conf.h"
#include "board.h"

#define ENABLE_DEBUG (0)
#include "debug.h"

typedef struct {
    char *tx_buf;
    char *rx_buf;
    uint16_t rx_len;
    uint16_t tx_len;
    uint8_t address;
    uint8_t error;
    kernel_pid_t pid;
} fifo_ops_t;

static fifo_ops_t fifo = { NULL, NULL, 0, 0, 0, 0, KERNEL_PID_UNDEF };

/**
 * @brief Array holding one pre-initialized mutex for each SPI device
 */
static mutex_t locks[] =  {
    [SPI_0] = MUTEX_INIT,
};

static int isr_tx, isr_pp, isr_rx;

void spi_isr_tx(uint8_t channel)
{
    USIC_CH_TypeDef *usic = channel ? USIC0_CH1 : USIC0_CH0;
    isr_tx++;

    while (fifo.tx_len) {
        const uint8_t tci = (fifo.tx_len == 1) ? 0x17 : 0x07;
        
        if (fifo.tx_buf) {
            usic->IN[tci] = *(fifo.tx_buf++);
        } else {
            usic->IN[tci] = 0xFF;
        }

        fifo.tx_len--;

        if (usic->TRBSR & USIC_CH_TRBSR_TFULL_Msk) {
            break;
        }
    }

    
    if (sched_context_switch_request) {
        thread_yield();
    }
}

void spi_isr_protocol(uint8_t channel)
{
    USIC_CH_TypeDef *usic = channel ? USIC0_CH1 : USIC0_CH0;
    isr_pp++;

    usic->PSCR = USIC_CH_PSCR_CST2_Msk;
    
    if (usic->PSR & (USIC_CH_PSR_SSCMode_MSLS_Msk)) {
        
    } else {
        /* frame end */
        gpio_toggle(GPIO_PIN(P0, 0));
        gpio_toggle(GPIO_PIN(P0, 0));
        thread_wakeup(fifo.pid);
        thread_yield();
    }
}

void spi_isr_rx(uint8_t channel)
{
    USIC_CH_TypeDef *usic = channel ? USIC0_CH1 : USIC0_CH0;
    isr_rx++;
    gpio_clear(GPIO_PIN(P0, 5));
    
    while ((USIC0_CH1->TRBSR & USIC_CH_TRBSR_RBFLVL_Msk) &&
           fifo.rx_len--) {
        *(fifo.rx_buf++) = (char)usic->OUTR;
    }

    gpio_set(GPIO_PIN(P0, 5));
    
    if (sched_context_switch_request) {
        thread_yield();
    }
}

int spi_conf_pins(spi_t spi)
{
    gpio_init(spi_instance[spi].mosi_pin & 0xff,
              GPIO_DIR_OUT | (spi_instance[spi].mosi_pin >> 8), GPIO_NOPULL);

    gpio_init(spi_instance[spi].sclk_pin & 0xff,
              GPIO_DIR_OUT | (spi_instance[spi].sclk_pin >> 8), GPIO_NOPULL);

    return 0;
}

int spi_init_master(spi_t dev, spi_conf_t conf, spi_speed_t speed)
{
    const usic_channel_t *usic_ch = (usic_channel_t *)&spi_instance[dev];

    /* returns 0 for USIC0_CH0 & 1 for USIC0_CH1 */
    const uint8_t u_index = usic_index(usic_ch) * 3;

    const usic_fdr_t fdr = {
        /* STEP value: (640/1024) * 32Mhz = 20Mhz base clock */
        .field.step = 640,
        /* DM: fractional divider mode */
        .field.dm	= 2
    };

    usic_brg_t brg = {
        /* (DCTQ+1) x (PCTQ+1) / f[SCLK]: one full phase of clock signal
         * for leading/trailing word delay */
        .field.ctqsel	= USIC_CTQIN_SCLK,
        .field.dctq		= 0,
        .field.pctq		= 0
    };

    /* polarity & phase setting */
    switch (conf) {
        case SPI_CONF_FIRST_RISING:   /* CPOL=0, CPHA=0 */
            brg.field.sclkcfg = 2;
            break;
        case SPI_CONF_SECOND_RISING:  /* CPOL=0, CPHA=1 */
            brg.field.sclkcfg = 0;
            break;
        case SPI_CONF_FIRST_FALLING:  /* CPOL=1, CPHA=0 */
            brg.field.sclkcfg = 3;
            break;
        case SPI_CONF_SECOND_FALLING: /* CPOL=1, CPHA=1 */
            brg.field.sclkcfg = 1;
            break;
        default:
            return -2;
    }

    /* 32Mhz * (640/1024) = 20Mhz / brg_pdiv / 2 = SPI speed */
    switch (speed) {
        case SPI_SPEED_100KHZ:
            brg.field.pdiv = 100;
            break;
        case SPI_SPEED_400KHZ:
            brg.field.pdiv = 25;
            break;
        case SPI_SPEED_1MHZ:
            brg.field.pdiv = 10;
            break;
        case SPI_SPEED_5MHZ:
            brg.field.pdiv = 2;
            break;
        case SPI_SPEED_10MHZ:
            brg.field.pdiv = 1;
            break;
        default:
            return -1;
    }

    /* setup & start the USIC channel */
    usic_init(usic_ch, brg, fdr);

    /* protocol interrupt for MSLS change detection */
    usic_ch->usic->INPR = (u_index + 1) << USIC_CH_INPR_PINP_Pos;

    /* register our interrupt service routines with USIC */
    usic_mplex[u_index]     = &spi_isr_tx;
    usic_mplex[u_index + 1] = &spi_isr_protocol;
    usic_mplex[u_index + 2] = &spi_isr_rx;

    NVIC_SetPriority(USIC0_0_IRQn + u_index, SPI_IRQ_PRIO);
    NVIC_SetPriority(USIC0_1_IRQn + u_index, SPI_IRQ_PRIO);
    NVIC_SetPriority(USIC0_2_IRQn + u_index, SPI_IRQ_PRIO);

    NVIC_EnableIRQ(USIC0_0_IRQn + u_index);
    NVIC_EnableIRQ(USIC0_1_IRQn + u_index);
    NVIC_EnableIRQ(USIC0_2_IRQn + u_index);

    usic_ch->usic->TBCTR =

        /* FIFO size is configurable between 2 (rx_size = 1) and 32
           (rx_size = 5). */

        (spi_instance[dev].fifo.tx_size << USIC_CH_TBCTR_SIZE_Pos) |
        (spi_instance[dev].fifo.tx_dptr << USIC_CH_TBCTR_DPTR_Pos) |

        /* a standard transmit buffer event fires when the fill level
           equals the LIMIT and then decreases (LOF) due to
           transmission: for LIMIT = 1 this results in a event when
           the FIFO just became empty */

        (1 << USIC_CH_TBCTR_LIMIT_Pos) |
        (0 << USIC_CH_TBCTR_LOF_Pos) |

        /* the standard transmit buffer interrupt is enabled (STBIEN).
           It is send to the first IRQ (STBINP). */

        (1 << USIC_CH_TBCTR_STBIEN_Pos) |
        (u_index << USIC_CH_TBCTR_STBINP_Pos);

    usic_ch->usic->RBCTR =

        /* FIFO size is configurable between 2 (rx_size = 1) and 32
           (rx_size = 5). */

        (spi_instance[dev].fifo.rx_size << USIC_CH_RBCTR_SIZE_Pos) |
        (spi_instance[dev].fifo.rx_dptr << USIC_CH_RBCTR_DPTR_Pos) |

        /* filling level is enabled (RNM) and an interrupt is
           generated when the fill level equals the LIMIT (rx_size - 1)
           and a new word arrives (LOF). */

        (0 << USIC_CH_RBCTR_RNM_Pos) |
        (1 << USIC_CH_RBCTR_LOF_Pos) |
        (((1 << spi_instance[dev].fifo.rx_size) - 1) << USIC_CH_RBCTR_LIMIT_Pos) |

        /* receiver control information (RCIM) contains protocol
           errors, parity errors and start of frame notification. */

        (2 << USIC_CH_RBCTR_RCIM_Pos) |

        /* the standard receive buffer interrupt is enabled (SRBIEN).
           It is send to the third IRQn (SRBINP). */

        (1 << USIC_CH_RBCTR_SRBIEN_Pos) |
        ((u_index + 2) << USIC_CH_RBCTR_SRBINP_Pos);

    gpio_init(spi_instance[dev].sclk_pin & 0xff,
              GPIO_DIR_OUT | (spi_instance[dev].sclk_pin >> 8), GPIO_NOPULL);

    gpio_init(GPIO_PIN(P0, 0), GPIO_DIR_OUT, GPIO_NOPULL);
    gpio_init(GPIO_PIN(P0, 5), GPIO_DIR_OUT, GPIO_NOPULL);
    gpio_set(GPIO_PIN(P0, 0));
    gpio_set(GPIO_PIN(P0, 5));
    
    return 0;
}

static void _spi_setup(USIC_CH_TypeDef *usic)
{
    isr_rx = isr_tx = isr_pp = 0;
    
    fifo.error = 0;
    fifo.pid = thread_getpid();

    /* clear all interrupt status flags in PSR register */
    usic->PSCR = 0xFFFFFFFF;

    /* clear all interrupt status flags in TRBSR register */
    usic->TRBSCR = (USIC_CH_TRBSCR_CSRBI_Msk |
                    USIC_CH_TRBSCR_CRBERI_Msk |
                    USIC_CH_TRBSCR_CARBI_Msk |
                    USIC_CH_TRBSCR_CSTBI_Msk |
                    USIC_CH_TRBSCR_CTBERI_Msk |
                    USIC_CH_TRBSCR_CBDV_Msk |
                    USIC_CH_TRBSCR_FLUSHTB_Msk |
                    USIC_CH_TRBSCR_FLUSHRB_Msk);

    /* invalidate former transmission  */
    usic->FMR = 2 << USIC_CH_FMR_MTDV_Pos;

    /* enable the interrupt triggering isr_tx */
    usic->TBCTR |= USIC_CH_TBCTR_STBIEN_Msk;
}

int spi_transfer_byte(spi_t spi, char out, char *in)
{
    return spi_transfer_bytes(spi, &out, in, 1);
}

int spi_transfer_bytes(spi_t dev, char *out, char *in, unsigned int length)
{
    USIC_CH_TypeDef *usic = spi_instance[dev].usic;

    fifo.tx_buf = out;
    fifo.tx_len = length;

    fifo.rx_buf = in;
    fifo.rx_len = in ? length : 0;
    
    _spi_setup(usic);

    /* trigger the spi_isr_tx interrupt to start transmitting*/
    usic->FMR = USIC_CH_FMR_SIO3_Msk;

    gpio_clear(GPIO_PIN(P0, 0));
    thread_sleep();
    gpio_set(GPIO_PIN(P0, 0));

    /* collect the last bytes that have not triggered an interrupt */
    while ((usic->TRBSR & USIC_CH_TRBSR_RBFLVL_Msk) &&
           fifo.rx_len--) {
        *(fifo.rx_buf++) = (char)usic->OUTR;
    }
    
    printf("spi_transfer_bytes(in: %p, out: %p, length: %i) rx: %i, tx: %i, pp: %i\n",
           (void*)out, (void*)in, length, isr_rx, isr_tx, isr_pp);
    
    return length;
}

int spi_transfer_reg(spi_t dev, uint8_t reg, char out, char *in)
{
    return spi_transfer_regs(dev, reg, &out, in, 1);
}

int spi_transfer_regs(spi_t dev, uint8_t reg, char *out, char *in, unsigned int length)
{
    USIC_CH_TypeDef *usic = spi_instance[dev].usic;

    fifo.tx_buf = out;
    fifo.tx_len = length;

    fifo.rx_buf = in;
    fifo.rx_len = length;
    
    _spi_setup(usic);

    /* disable spi_isr_rx */
    usic->RBCTR &= ~USIC_CH_RBCTR_SRBIEN_Msk;
    usic->IN[0x7] = reg;
    /* wait while RxFIFO remains empty */
    while (usic->TRBSR & USIC_CH_TRBSR_REMPTY_Msk) ;
    /* bogus read */
    fifo.address = usic->OUTR;
    usic->RBCTR |= USIC_CH_RBCTR_SRBIEN_Msk;
    
    gpio_clear(GPIO_PIN(P0, 0));
    thread_sleep();
    gpio_set(GPIO_PIN(P0, 0));

    /* collect the last bytes that have not triggered an interrupt */
    while ((usic->TRBSR & USIC_CH_TRBSR_RBFLVL_Msk) &&
           fifo.rx_len--) {
        *(fifo.rx_buf++) = (char)usic->OUTR;
    }
    
    printf("spi_transfer_regs(reg: %u, out: %p, in: %p, length: %i) rx: %i, tx: %i, pp: %i\n",
           reg, (void*)out, (void*)in, length, isr_rx, isr_tx, isr_pp);
    
    return length;
}

int spi_acquire(spi_t spi)
{
    if (spi >= SPI_NUMOF) {
        return -1;
    }
    mutex_lock(&locks[spi]);
    return 0;
}

int spi_release(spi_t spi)
{
    if (spi >= SPI_NUMOF) {
        return -1;
    }
    mutex_unlock(&locks[spi]);
    return 0;
}

void spi_poweron(spi_t dev)
{
    /* not implemented yet */
}

void spi_poweroff(spi_t dev)
{
    /* not implemented yet */
}

void spi_transmission_begin(spi_t dev, char reset_val)
{
    /* not implemented yet */
}

int spi_init_slave(spi_t dev, spi_conf_t conf, char (*cb)(char))
{
    /* not implemented yet */
    return 0;
}
