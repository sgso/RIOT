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
 * @brief       Low-level I2C driver implementation
 *
 * @author      Sebastian Sontberg <sebastian@sontberg.de>
 *
 * @}
 */

#include "mutex.h"
#include "periph_conf.h"
#include "sched.h"
#include "thread.h"

#include "periph/i2c.h"
#include "periph/usic.h"


#define ENABLE_DEBUG    (0)
#include "debug.h"

#define TDF_TRANSMIT       (0)
#define TDF_RECV_ACK       (2 << 8)
#define TDF_RECV_NACK      (3 << 8)
#define TDF_START          (4 << 8)
#define TDF_REPEATED_START (5 << 8)
#define TDF_STOP           (6 << 8)

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
 * @brief Array holding one pre-initialized mutex for each I2C device
 */
static mutex_t locks[] =  {
    [I2C_0] = MUTEX_INIT,
};

void i2c_isr_tx(uint8_t channel)
{
    USIC_CH_TypeDef *usic = channel ? USIC0_CH1 : USIC0_CH0;

    while (fifo.tx_len > 1) {
        if (fifo.tx_buf) {
            usic->IN[0] = TDF_TRANSMIT | *(fifo.tx_buf++);
        }
        else {
            usic->IN[0] = TDF_RECV_ACK;
        }

        fifo.tx_len--;

        if (usic->TRBSR & USIC_CH_TRBSR_TFULL_Msk) {
            goto sched;
        }
    }

    /* TxFIFO not full at this point because of return above */
    while (fifo.tx_len == 1) {
        if (fifo.tx_buf) {
            usic->IN[0] = TDF_TRANSMIT | *(fifo.tx_buf--);
        }
        else {
            usic->IN[0] = TDF_RECV_NACK;
        }

        fifo.tx_len--;

        if (usic->TRBSR & USIC_CH_TRBSR_TFULL_Msk) {
            goto sched;
        }
    }

    /* At this point, there is always at least space for one more
     * entry in the transmit FIFO */

    /* This makes us switch back to transmitting read (N)ACKS in case
     * we want to read after write */
    if (fifo.tx_buf && fifo.rx_len) {
        fifo.tx_buf = NULL;
        fifo.tx_len = fifo.rx_len;

        /* repeated start condition with READ bit set */
        usic->IN[0] = TDF_REPEATED_START | fifo.address | 1;
    }
    else {
        /* disable the interrupt triggering this isr */
        usic->TBCTR &= ~USIC_CH_TBCTR_STBIEN_Msk;
        usic->IN[0] = TDF_STOP;
    }

sched:

    if (sched_context_switch_request) {
        thread_yield();
    }
}

void i2c_isr_protocol(uint8_t channel)
{
    USIC_CH_TypeDef *usic = channel ? USIC0_CH1 : USIC0_CH0;

    if (usic->PSR & (USIC_CH_PSR_IICMode_NACK_Msk |
                     USIC_CH_PSR_IICMode_ERR_Msk |
                     USIC_CH_PSR_IICMode_ARL_Msk)) {
        fifo.error = 1;
    }
    else {
        fifo.error = 0;
    }

    thread_wakeup(fifo.pid);
    thread_yield();
}

void i2c_isr_rx(uint8_t channel)
{
    USIC_CH_TypeDef *usic = channel ? USIC0_CH1 : USIC0_CH0;

    while ((USIC0_CH1->TRBSR & USIC_CH_TRBSR_RBFLVL_Msk) &&
           fifo.rx_len--) {
        *(fifo.rx_buf++) = (char)usic->OUTR;
    }

    if (sched_context_switch_request) {
        thread_yield();
    }
}

int i2c_init_master(i2c_t dev, i2c_speed_t speed)
{
    const usic_channel_t *usic_ch = (usic_channel_t *)&i2c_instance[dev];

    /* returns 0 for USIC0_CH0 & 1 for USIC0_CH1 */
    const uint8_t u_index = usic_index(usic_ch) * 3;

    if (speed != I2C_SPEED_NORMAL) {
        return -2;
    }

    const usic_fdr_t fdr = {
        /* STEP value: (320/1024) * 32Mhz = 10Mhz base clock */
        .field.step = 320,
        /* DM: fractional divider mode */
        .field.dm = 2
    };

    usic_brg_t brg = {
        .field.ctqsel = USIC_CTQIN_PDIV,
        .field.dctq   = 9,
        .field.pctq   = 0,
        .field.pdiv   = 9
    };

    /* setup & start the USIC channel */
    usic_init(usic_ch, brg, fdr);

    usic_ch->usic->INPR = (u_index + 1) << USIC_CH_INPR_PINP_Pos;

    /* register our interrupt service routines with USIC */
    usic_mplex[u_index]     = &i2c_isr_tx;
    usic_mplex[u_index + 1] = &i2c_isr_protocol;
    usic_mplex[u_index + 2] = &i2c_isr_rx;

    NVIC_SetPriority(USIC0_0_IRQn + u_index, I2C_IRQ_PRIO);
    NVIC_SetPriority(USIC0_1_IRQn + u_index, I2C_IRQ_PRIO);
    NVIC_SetPriority(USIC0_2_IRQn + u_index, I2C_IRQ_PRIO);

    NVIC_EnableIRQ(USIC0_0_IRQn + u_index);
    NVIC_EnableIRQ(USIC0_1_IRQn + u_index);
    NVIC_EnableIRQ(USIC0_2_IRQn + u_index);

    usic_ch->usic->TBCTR =

        /* FIFO size is configurable between 2 (rx_size = 1) and 32
           (rx_size = 5). */

        (i2c_instance[dev].fifo.tx_size << USIC_CH_TBCTR_SIZE_Pos) |
        (i2c_instance[dev].fifo.tx_dptr << USIC_CH_TBCTR_DPTR_Pos) |

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

        (i2c_instance[dev].fifo.rx_size << USIC_CH_RBCTR_SIZE_Pos) |
        (i2c_instance[dev].fifo.rx_dptr << USIC_CH_RBCTR_DPTR_Pos) |

        /* filling level is enabled (RNM) and an interrupt is
           generated when the fill level equals the LIMIT (rx_size - 1)
           and a new word arrives (LOF). */

        (0 << USIC_CH_RBCTR_RNM_Pos) |
        (1 << USIC_CH_RBCTR_LOF_Pos) |
        (((1 << i2c_instance[dev].fifo.rx_size) - 1) << USIC_CH_RBCTR_LIMIT_Pos) |

        /* receiver control information (RCIM) contains protocol
           errors, parity errors and start of frame notification. */

        (2 << USIC_CH_RBCTR_RCIM_Pos) |

        /* the standard receive buffer interrupt is enabled (SRBIEN).
           It is send to the third IRQn (SRBINP). */

        (1 << USIC_CH_RBCTR_SRBIEN_Pos) |
        ((u_index + 2) << USIC_CH_RBCTR_SRBINP_Pos);

    return 0;
}

static void _i2c_setup(USIC_CH_TypeDef *usic, uint8_t address)
{
    fifo.error = 0;
    fifo.pid = thread_getpid();
    fifo.address = address << 1;

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

static int _i2c_return(USIC_CH_TypeDef *usic, int length)
{
    fifo.pid = KERNEL_PID_UNDEF;

    /* collect the last bytes that have not triggered an interrupt */
    while ((usic->TRBSR & USIC_CH_TRBSR_RBFLVL_Msk) &&
           fifo.rx_len--) {
        *(fifo.rx_buf++) = (char)usic->OUTR;
    }

    if (fifo.error) {
        return 0;
    }
    else {
        return length - fifo.rx_len - fifo.tx_len;
    }
}

int i2c_read_byte(i2c_t dev, uint8_t address, char *data)
{
    return i2c_read_bytes(dev, address, data, 1);
}

int i2c_read_bytes(i2c_t dev, uint8_t address, char *data, int length)
{
    USIC_CH_TypeDef *usic = i2c_instance[dev].usic;

    fifo.tx_buf = NULL;
    fifo.tx_len = length;

    fifo.rx_buf = data;
    fifo.rx_len = length;

    _i2c_setup(usic, address);

    /* start condition with R/W bit set to READ */
    usic->IN[0] = TDF_START | fifo.address | 1;

    thread_sleep();

    return _i2c_return(usic, length);
}

int i2c_write_bytes(i2c_t dev, uint8_t address, char *data, int length)
{
    USIC_CH_TypeDef *usic = i2c_instance[dev].usic;

    fifo.tx_buf = data;
    fifo.tx_len = length;

    fifo.rx_buf = NULL;
    fifo.rx_len = 0;

    _i2c_setup(usic, address);

    /* start condition with R/W bit set to WRITE */
    usic->IN[0] = TDF_START | fifo.address;

    thread_sleep();

    return _i2c_return(usic, length);
}


int i2c_write_byte(i2c_t dev, uint8_t address, char data)
{
    return i2c_write_bytes(dev, address, &data, 1);
}

int i2c_read_regs(i2c_t dev, uint8_t address, uint8_t reg, char *data, int length)
{
    USIC_CH_TypeDef *usic = i2c_instance[dev].usic;

    fifo.tx_buf = (char *)&reg;
    fifo.tx_len = 1;

    fifo.rx_buf = data;
    fifo.rx_len = length;

    _i2c_setup(usic, address);

    /* start condition with R/W bit set to WRITE */
    usic->IN[0] = TDF_START | fifo.address;

    thread_sleep();

    return _i2c_return(usic, length);
}

int i2c_read_reg(i2c_t dev, uint8_t address, uint8_t reg, char *data)
{
    return i2c_read_regs(dev, address, reg, data, 1);
}

int i2c_write_regs(i2c_t dev, uint8_t address, uint8_t reg, char *data, int length)
{
    USIC_CH_TypeDef *usic = i2c_instance[dev].usic;

    fifo.tx_buf = data;
    fifo.tx_len = length;

    fifo.rx_buf = NULL;
    fifo.rx_len = 0;

    _i2c_setup(usic, address);

    /* start condition with R/W bit set to WRITE */
    usic->IN[0] = TDF_START | fifo.address;

    /* write register */
    usic->IN[0] = TDF_TRANSMIT | reg;

    thread_sleep();

    return _i2c_return(usic, length);
}

int i2c_write_reg(i2c_t dev, uint8_t address, uint8_t reg, char data)
{
    return i2c_write_regs(dev, address, reg, &data, 1);
}

int i2c_acquire(i2c_t dev)
{
    if (dev >= I2C_NUMOF) {
        return -1;
    }

    mutex_lock(&locks[dev]);
    return 0;
}

int i2c_release(i2c_t dev)
{
    if (dev >= I2C_NUMOF) {
        return -1;
    }

    mutex_unlock(&locks[dev]);
    return 0;
}

void i2c_poweron(i2c_t dev)
{
    /* not implemented yet */
}

void i2c_poweroff(i2c_t dev)
{
    /* not implemented yet */
}
