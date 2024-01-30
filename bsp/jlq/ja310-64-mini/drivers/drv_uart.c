/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-3-08      GuEe-GUI     the first version
 */

#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>

#include <board.h>

#define UART_RBR 0 /* In: Receive buffer */
#define UART_THR 0 /* Out: Transmit Holding*/

#define UART_DLL 0    /* Out: Divisor Latch Low */
#define UART_DLH 0x04 /* Out: Divisor Latch High */

#define UART_IER     0x04 /* Out: Interrupt Enable Register */
#define UART_IER_RDI 0x01 /* Enable receiver data interrupt */

#define UART_IIR            0x08 /* In: Interrupt Identity Register */
#define UART_IIR_NO_INT     0x01 /* No interrupts pending */
#define UART_IIR_BUSY       0x07 /* DesignWare APB Busy Detect */
#define UART_IIR_RX_TIMEOUT 0x0C /* OMAP RX Timeout interrupt */

#define UART_FCR            0x08 /* Out: FIFO Control Register */
#define UART_FCR_EN_FIFO    0x01 /* Enable the FIFO */
#define UART_FCR_CLEAR_RCVR 0x02 /* Clear the RCVR FIFO */
#define UART_FCR_CLEAR_XMIT 0x04 /* Clear the XMIT FIFO */
#define UART_FCR_RCVR_TRIG  0x30

#define UART_TCR      0x0C /* Out: Transfer Control Register */
#define UART_TCR_DLAB 0x80 /* Divisor latch access bit */

#define UART_TCR_PARITY 0x8 /* Parity Enable */
#define UART_TCR_STOP   0x4 /* Stop bits: 0=1 bit, 1=2 bits */
#define UART_TCR_WLEN8  0x3 /* Wordlength: 8 bits */

#define UART_TSR    0x14 /* In: Transfer Status Register */
#define UART_TSR_BI 0x10 /* Break interrupt indicator */
#define UART_TSR_DR 0x01 /* Receiver data ready */

#define UART_USR 0x7C /* UART Status Register */

// #define UART_INPUT_CLK      24000000

struct hw_uart_device {
    rt_ubase_t hw_base;
    rt_uint32_t irqno;
#ifdef RT_USING_SMP
    struct rt_spinlock spinlock;
#endif
};

static struct hw_uart_device _uart0_device = {UART0_BASE, UART0_IRQ};
static struct rt_serial_device _serial0;

rt_inline rt_uint32_t jlq_uart_read32(rt_ubase_t addr, rt_ubase_t offset) {
    return *((volatile rt_uint32_t *)(addr + offset));
}

rt_inline void jlq_uart_write32(rt_ubase_t addr, rt_ubase_t offset, rt_uint32_t value) {
    *((volatile rt_uint32_t *)(addr + offset)) = value;

    if (offset == UART_TCR) {
        int tries = 1000;

        /* Make sure TCR write wasn't ignored */
        while (tries--) {
            unsigned int lcr = jlq_uart_read32(addr, UART_TCR);

            jlq_uart_write32(addr, UART_FCR,
                             UART_FCR_EN_FIFO | UART_FCR_CLEAR_RCVR | UART_FCR_CLEAR_XMIT);
            jlq_uart_read32(addr, UART_RBR);

            *((volatile rt_uint32_t *)(addr + offset)) = value;
        }
    }
}

static rt_err_t jlq_uart_uart_configure(struct rt_serial_device *serial,
                                        struct serial_configure *cfg) {
    rt_base_t base, rate;
    struct hw_uart_device *uart;

    RT_ASSERT(serial != RT_NULL);
    uart = (struct hw_uart_device *)serial->parent.user_data;
    base = uart->hw_base;

#ifdef RT_USING_SMP
    rt_spin_lock_init(&uart->spinlock);
#endif

    jlq_uart_write32(base, UART_IER, !UART_IER_RDI);
    jlq_uart_write32(base, UART_FCR,
                     UART_FCR_EN_FIFO | UART_FCR_CLEAR_RCVR | UART_FCR_CLEAR_XMIT |
                         UART_FCR_RCVR_TRIG);

#if 0 // comm uart default baudrate is 115200.
    rate = UART_INPUT_CLK / 16 / serial->config.baud_rate;

    /* Enable access DLL & DLH */
    jlq_uart_write32(base, UART_TCR, jlq_uart_read32(base, UART_TCR) | UART_TCR_DLAB);
    jlq_uart_write32(base, UART_DLL, (rate & 0xff));
    jlq_uart_write32(base, UART_DLH, (rate & 0xff00) >> 8);
    /* Clear DLAB bit */
    // jlq_uart_write32(base, UART_TCR, jlq_uart_read32(base, UART_TCR) & (~UART_TCR_DLAB));
#endif

    jlq_uart_write32(base, UART_TCR, jlq_uart_read32(base, UART_TCR) & (~UART_TCR_DLAB));

    jlq_uart_write32(base, UART_TCR,
                     (jlq_uart_read32(base, UART_TCR) & (~UART_TCR_WLEN8)) | UART_TCR_WLEN8);
    jlq_uart_write32(base, UART_TCR, jlq_uart_read32(base, UART_TCR) & (~UART_TCR_STOP));
    jlq_uart_write32(base, UART_TCR, jlq_uart_read32(base, UART_TCR) & (~UART_TCR_PARITY));

    jlq_uart_write32(base, UART_IER, UART_IER_RDI);
    // rt_gpio_dbg(5);
    return RT_EOK;
}

static rt_err_t jlq_uart_uart_control(struct rt_serial_device *serial, int cmd, void *arg) {
    struct hw_uart_device *uart;

    RT_ASSERT(serial != RT_NULL);
    uart = (struct hw_uart_device *)serial->parent.user_data;
    // rt_gpio_dbg(cmd);
    switch (cmd) {
        case RT_DEVICE_CTRL_CLR_INT:
            /* Disable rx irq */
            rt_hw_interrupt_mask(uart->irqno);
            jlq_uart_write32(uart->hw_base, UART_IER, !UART_IER_RDI);

            break;

        case RT_DEVICE_CTRL_SET_INT:
            /* Enable rx irq */
            rt_hw_interrupt_umask(uart->irqno);
            jlq_uart_write32(uart->hw_base, UART_IER, UART_IER_RDI);

            break;
    }

    return RT_EOK;
}

static int jlq_uart_uart_putc(struct rt_serial_device *serial, char c) {
    rt_base_t base;
    struct hw_uart_device *uart;
    // rt_gpio_dbg(1);
    RT_ASSERT(serial != RT_NULL);
    uart = (struct hw_uart_device *)serial->parent.user_data;
    base = uart->hw_base;

    while ((jlq_uart_read32(base, UART_USR) & 0x1) != 0) { //Bit0 means UART busy bit: 0 for idle; 1 for transfer in progress
    }

    jlq_uart_write32(base, UART_THR, c);
    // rt_gpio_dbg(2);
    return 1;
}

static int jlq_uart_uart_getc(struct rt_serial_device *serial) {
    int ch = -1;
    rt_base_t base;
    struct hw_uart_device *uart;
    // rt_gpio_dbg(3);
    RT_ASSERT(serial != RT_NULL);
    uart = (struct hw_uart_device *)serial->parent.user_data;
    base = uart->hw_base;

    if ((jlq_uart_read32(base, UART_TSR) & 0x1)) {
        ch = jlq_uart_read32(base, UART_RBR) & 0xff;
    }
    // rt_gpio_dbg(4);
    return ch;
}

static const struct rt_uart_ops _uart_ops = {
    jlq_uart_uart_configure,
    jlq_uart_uart_control,
    jlq_uart_uart_putc,
    jlq_uart_uart_getc,
};

static void rt_hw_uart_isr(int irqno, void *param) {
    unsigned int iir, status;
    struct rt_serial_device *serial = (struct rt_serial_device *)param;
    struct hw_uart_device *uart = (struct hw_uart_device *)serial->parent.user_data;

    iir = jlq_uart_read32(uart->hw_base, UART_IIR);

    /* If don't do this in non-DMA mode then the "RX TIMEOUT" interrupt will fire forever. */
    if ((iir & 0x3f) == UART_IIR_RX_TIMEOUT)
    {
#ifdef RT_USING_SMP
        rt_base_t level = rt_spin_lock_irqsave(&uart->spinlock);
#endif
        status = jlq_uart_read32(uart->hw_base, UART_TSR);
        if (!(status & (UART_TSR_DR | UART_TSR_BI))) {
            jlq_uart_read32(uart->hw_base, UART_RBR);
        }

#ifdef RT_USING_SMP
        rt_spin_unlock_irqrestore(&uart->spinlock, level);
#endif
    }

    if (!(iir & UART_IIR_NO_INT))
    // if (((iir&0x0f) & 0x4))
    {
        rt_hw_serial_isr(serial, RT_SERIAL_EVENT_RX_IND);
    }

    if ((iir & UART_IIR_BUSY) == UART_IIR_BUSY) {
        /* Clear the USR */
        jlq_uart_read32(uart->hw_base, UART_USR);
        return;
    }
}


int rt_hw_uart_init(void) {
    struct hw_uart_device *uart;
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;

    config.baud_rate = 115200;

    uart = &_uart0_device;
#ifdef RT_USING_SMART
    uart->hw_base = (rt_uint64_t)rt_ioremap((void*)UART0_BASE, 0x1000);
#endif
    _serial0.ops = &_uart_ops;
    _serial0.config = config;
    rt_hw_serial_register(&_serial0, "uart0", RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX, uart);
    rt_hw_interrupt_install(uart->irqno, rt_hw_uart_isr, &_serial0, "uart0");

    return 0;
}
