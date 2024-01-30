/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author         Notes
 * 2023-02-06     RT-Thread      first version
 */
#ifndef __JA310_H__
#define __JA310_H__

#include <rtthread.h>

#define __REG32(x)  (*((volatile unsigned int *)(x)))
#define __REG16(x)  (*((volatile unsigned short *)(x)))

#define BIT(x)	(1 << (x))

/* GPIO */
#define GPIO_MUX_BASE   0xF9034000
extern size_t gpio_mux_base_addr;
#define GPIO_BASE   0xF9035000

/* COMM UART */
#define UART0_BASE  0xF9037000
#define UART0_IRQ       (31 + 32)

/* GIC IRQ MAX */
#define MAX_HANDLERS                (256)

// /* GIC */
#define INTC_BASE                   (0xff700000)
#define ARM_GIC_NR_IRQS             (512)
#define ARM_GIC_MAX_NR              (512)

#define GIC_IRQ_START   0

#define AP_PWR_BASE		0xFA722000
#define AP_PWR_SIZE		0X1000

/* AP_PWR reset */
extern size_t ap_pwr_base_addr;
#define AP_PWR_SDIO0_CLKS_CTL   (ap_pwr_base_addr + 0x084)
#define AP_PWR_PWEN_CTL         (ap_pwr_base_addr + 0x3d4)
#define AP_PWR_SFRST_CTL		(ap_pwr_base_addr + 0x140)
#define AP_PWR_CHIPRSTN_CTL		(ap_pwr_base_addr + 0x144)
#define AP_PWR_MOD_RSTCTL0		(ap_pwr_base_addr + 0x150)
#define AP_PWR_MOD_RSTCTL1		(ap_pwr_base_addr + 0x154)
#define AP_PWR_WMRST_MODE		(ap_pwr_base_addr + 0x18c)
#define AP_PWR_CPUSYS_RSTCTL		(ap_pwr_base_addr + 0x190)
#define CHIP_RSTN_MK			(15)
#define SFRST_CHIP_RSTN_CNT_TRIGGER_MK	(13)
#define SFRST_PWR2SYS_RSTN_MK		(12)
#define CHIP_RSTN_CNT_LIMIT		(0xfff)

/* DDR PWR*/
#define DDR_PWR_BASE		0xF9030000
#define DDR_PWR_SIZE		0x1000

/* sysdump start reg*/
extern size_t ddr_pwr_base_addr;
#define AP_PWR_START_REG0		(ap_pwr_base_addr + 0x180)
#define DDR_PWR_RES_REG0		(ddr_pwr_base_addr + 0x500)

#define DDRPWR_RES_REG0_SYSDUMP_FLAG		(0xf)
#define DDRPWR_RES_REG0_REBOOT_FLAG		(0xe)

/* EMMC */
#define EMMC_BASE_ADDR 0xFA507000
/* SDIO */
#define SDIO0_BASE_ADDR 0xFA508000
extern size_t sdio0_base_addr;
#define MUXPIN_CTRL_SD0CLK  0x16C
#define MUXPIN_CTRL_SD0CMD  0x170
#define MUXPIN_CTRL_SD0D3   0x174
#define MUXPIN_CTRL_SD0D2   0x178
#define MUXPIN_CTRL_SD0D1   0x17C
#define MUXPIN_CTRL_SD0D0   0x180

#define SDIO1_BASE_ADDR 0xFA509000
#define SDIO2_BASE_ADDR 0xFA50A000

/* I2C */
#define I2C0_BASE_ADDR		0xFA727000
extern size_t i2c0_base_addr;
#define I2C1_BASE_ADDR	0xFA728000
extern size_t i2c1_base_addr;
#define I2C2_BASE_ADDR	0xFA729000
extern size_t i2c2_base_addr;
#define I2C3_BASE_ADDR	0xF9018000
extern size_t i2c3_base_addr;
#define I2C4_BASE_ADDR	0xF9028000
extern size_t i2c4_base_addr;
#define I2C5_BASE_ADDR	0xF9029000
extern size_t i2c5_base_addr;

#define I2C0_SCL_MUXPIN		0xEC
#define I2C0_SDA_MUXPIN		0xF0
#define I2C1_SCL_MUXPIN		0xF4
#define I2C1_SDA_MUXPIN		0xF8
#define I2C2_SCL_MUXPIN		0xFC
#define I2C2_SDA_MUXPIN		0x100
#define I2C3_SCL_MUXPIN		0x114
#define I2C3_SDA_MUXPIN		0x118
#define I2C4_SCL_MUXPIN		0x11C
#define I2C4_SDA_MUXPIN		0x120
#define I2C5_SCL_MUXPIN		0x124
#define I2C5_SDA_MUXPIN		0x128

/* the basic constants and interfaces needed by gic */
#define GIC_600_DISTRIBUTOR_PPTR      0xFA900000
#define GIC_600_REDISTRIBUTOR_PPTR    0xFA940000
#define GIC_600_CONTROLLER_PPTR       RT_NULL

#define DELAY_MICROS(micros)                            \
    do{                                                 \
        for (int j = 120; j > 0; j--);              \
    } while (0)

rt_inline rt_uint32_t platform_get_gic_dist_base(void)
{
    return GIC_600_DISTRIBUTOR_PPTR;
}

rt_inline rt_uint32_t platform_get_gic_redist_base(void)
{
    return GIC_600_REDISTRIBUTOR_PPTR;
}

rt_inline rt_uint32_t platform_get_gic_cpu_base(void)
{
    return GIC_600_CONTROLLER_PPTR;
}

#endif
