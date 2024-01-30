/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author         Notes
 * 2020-10-27     bigmagic       first version
 */

#include <drivers/i2c.h>

#include "drv_i2c.h"
#include "board.h"

#define DBG_TAG               "i2c"
#define DBG_LVL               DBG_INFO
#include <rtdbg.h>

/* designware i2c ip driver for rt-thread by JLQ*/
struct dw_scl_sda_cfg {
	rt_uint32_t ss_hcnt;
	rt_uint32_t fs_hcnt;
	rt_uint32_t ss_lcnt;
	rt_uint32_t fs_lcnt;
	rt_uint32_t sda_hold;
};

/* saved register address variable */
static rt_uint8_t g_read_reg_addr;
static rt_uint32_t JLQ_IC_CLK;

static void dw_i2c_enable(struct i2c_regs *i2c_base, rt_bool_t enable)
{
	rt_uint32_t ena = enable ? IC_ENABLE_0B : 0;
	int timeout = 100;

	do {
		writel(ena, &i2c_base->ic_enable);
		if ((readl(&i2c_base->ic_enable_status) & IC_ENABLE_0B) == ena)
			return;

		/*
		 * Wait 10 times the signaling period of the highest I2C
		 * transfer supported by the driver (for 400KHz this is
		 * 25us) as described in the DesignWare I2C databook.
		 */
		DELAY_MICROS(25);
	} while (timeout--);

	LOG_E("timeout in %sabling I2C adapter\n", enable ? "en" : "dis");
}

/*
 * i2c_set_bus_speed - Set the i2c speed
 * @speed:	required i2c speed
 *
 * Set the i2c speed.
 */
static unsigned int __dw_i2c_set_bus_speed(struct i2c_regs *i2c_base,
					   struct dw_scl_sda_cfg *scl_sda_cfg,
					   unsigned int speed)
{
	unsigned int cntl;
	unsigned int hcnt, sda_hold, spk, lcnt;
	int i2c_spd;
	unsigned int min_hcnt, min_lcnt;
	unsigned int sum;

	if (speed >= I2C_MAX_SPEED)
		i2c_spd = IC_SPEED_MODE_MAX;
	else if (speed >= I2C_FAST_SPEED)
		i2c_spd = IC_SPEED_MODE_FAST;
	else
		i2c_spd = IC_SPEED_MODE_STANDARD;

	/* Because the IC CLK of i2c3 is different from i2c0/i2c1/i2c2/i2c4/i2c5 on JA310.
	 * The IC_CLK is a macor, it is a constant, so its value can't be changed.
	 * so we declare JLQ_IC_CLK to replace IC_CLK.
	 */
	if (i2c_base && i2c_base == (struct i2c_regs *)i2c3_base_addr)
		JLQ_IC_CLK = IC_CLK;
	else
		JLQ_IC_CLK = IC_CLK / 2;

	/* to set speed cltr must be disabled */
	dw_i2c_enable(i2c_base, RT_FALSE);

	cntl = (readl(&i2c_base->ic_con) & (~IC_CON_SPD_MSK));

	/* sda hold at least 300ns */
	sda_hold = SDA_HOLD_TIME_NANO / (NANO_TO_MICRO / JLQ_IC_CLK) + 1;
	writel(sda_hold, &i2c_base->ic_sda_hold);
	/*the calculation based on i2c standard*/

	switch (i2c_spd) {
#ifndef CONFIG_X86 /* No High-speed for BayTrail yet */
	case IC_SPEED_MODE_MAX:
		cntl |= IC_CON_SPD_SS;
		if (scl_sda_cfg) {
			hcnt = scl_sda_cfg->fs_hcnt;
			lcnt = scl_sda_cfg->fs_lcnt;
		} else {
			/* cal hcnt/lcnt min value depend on designware-i2c-spec */
			min_hcnt = (JLQ_IC_CLK * MIN_HS_SCL_HIGHTIME) / NANO_TO_MICRO;
			min_lcnt = (JLQ_IC_CLK * MIN_HS_SCL_LOWTIME) / NANO_TO_MICRO;
			/* Cal sum count for the speed 3400K */
			sum = JLQ_IC_CLK * NANO_TO_MICRO * NANO_TO_MICRO / I2C_MAX_SPEED;
			hcnt = sum / 3 + 1;
			spk = 10 / (NANO_TO_MICRO / JLQ_IC_CLK) + 1;
			/* Select a value for hcnt/lcnt, make them corresponds to SPEC & Speed */
			hcnt = max(hcnt - 6 - spk, min_hcnt);
			lcnt = max(sum - (hcnt + spk + 6) - 1, min_lcnt);
		}
		writel(hcnt, &i2c_base->ic_hs_scl_hcnt);
		writel(lcnt, &i2c_base->ic_hs_scl_lcnt);
		break;
#endif

	case IC_SPEED_MODE_STANDARD:
		cntl |= IC_CON_SPD_SS;
		if (scl_sda_cfg) {
			hcnt = scl_sda_cfg->ss_hcnt;
			lcnt = scl_sda_cfg->ss_lcnt;
		} else {
			/* cal hcnt/lcnt min value depend on designware-i2c-spec */
			min_hcnt = (JLQ_IC_CLK * MIN_SS_SCL_HIGHTIME) / NANO_TO_MICRO;
			min_lcnt = (JLQ_IC_CLK * MIN_SS_SCL_LOWTIME) / NANO_TO_MICRO;
			/* Cal sum count for the speed 100K */
			sum = JLQ_IC_CLK * NANO_TO_MICRO * NANO_TO_MICRO / I2C_STANDARD_SPEED;
			hcnt = (sum * 40) / 87 + 1;
			spk = 50 / (NANO_TO_MICRO / JLQ_IC_CLK) + 1;
			/* Select a value for hcnt/lcnt, make them corresponds to SPEC & Speed */
			hcnt = max(hcnt - 6 - spk, min_hcnt);
			lcnt = max(sum - (hcnt + spk + 6) - 1, min_lcnt);
		}
		writel(hcnt, &i2c_base->ic_ss_scl_hcnt);
		writel(lcnt, &i2c_base->ic_ss_scl_lcnt);
		break;

	case IC_SPEED_MODE_FAST:
	default:
		cntl |= IC_CON_SPD_FS;
		if (scl_sda_cfg) {
			hcnt = scl_sda_cfg->fs_hcnt;
			lcnt = scl_sda_cfg->fs_lcnt;
		} else {
			/* cal hcnt/lcnt min value depend on designware-i2c-spec */
			min_hcnt = (JLQ_IC_CLK * MIN_FS_SCL_HIGHTIME) / NANO_TO_MICRO;
			min_lcnt = (JLQ_IC_CLK * MIN_FS_SCL_LOWTIME) / NANO_TO_MICRO;
			/* Cal sum count for the speed 400K */
			sum = JLQ_IC_CLK * NANO_TO_MICRO * NANO_TO_MICRO / I2C_FAST_SPEED;
			hcnt = (sum * 6) / 19 + 1;
			spk = 50 / (NANO_TO_MICRO / JLQ_IC_CLK) + 1;
			/* Select a value for hcnt/lcnt, make them corresponds to SPEC & Speed */
			hcnt = max(hcnt - 6 - spk, min_hcnt);
			lcnt = max(sum - (hcnt + spk + 6) - 1, min_lcnt);
		}
		writel(hcnt, &i2c_base->ic_fs_scl_hcnt);
		writel(lcnt, &i2c_base->ic_fs_scl_lcnt);
		break;
	}

	writel(cntl, &i2c_base->ic_con);

	LOG_I("clk: %u Hz, mode: %d, hcnt: %u, lcnt: %u, sda_hold: %u\n",
			JLQ_IC_CLK, i2c_spd, hcnt, lcnt, sda_hold);

	/* Configure SDA Hold Time if required */
	if (scl_sda_cfg)
		writel(scl_sda_cfg->sda_hold, &i2c_base->ic_sda_hold);

	/* Enable back i2c now speed set */
	dw_i2c_enable(i2c_base, RT_TRUE);

	return 0;
}

/*
 * i2c_setaddress - Sets the target slave address
 * @i2c_addr:	target i2c address
 *
 * Sets the target slave address.
 */
static void i2c_setaddress(struct i2c_regs *i2c_base, unsigned int i2c_addr)
{
	/* Disable i2c */
	dw_i2c_enable(i2c_base, RT_FALSE);

	writel(i2c_addr, &i2c_base->ic_tar);

	/* Enable i2c */
	dw_i2c_enable(i2c_base, RT_TRUE);
}

/*
 * i2c_flush_rxfifo - Flushes the i2c RX FIFO
 *
 * Flushes the i2c RX FIFO
 */
static void i2c_flush_rxfifo(struct i2c_regs *i2c_base)
{
	while (readl(&i2c_base->ic_status) & IC_STATUS_RFNE)
		readl(&i2c_base->ic_cmd_data);
}

/*
 * i2c_wait_for_bb - Waits for bus busy
 *
 * Waits for bus busy
 */
static int i2c_wait_for_bb(struct i2c_regs *i2c_base)
{
    int cnt = 10000;
	while ((readl(&i2c_base->ic_status) & IC_STATUS_MA) ||
	       !(readl(&i2c_base->ic_status) & IC_STATUS_TFE)) {
		cnt--;
		DELAY_MICROS(1);
		/* Evaluate timeout */
		if (cnt <= 0)
			return 1;
	}

	return 0;
}

static int i2c_xfer_init(struct i2c_regs *i2c_base, rt_uint8_t chip, rt_uint32_t addr,
			 int alen)
{
	if (i2c_wait_for_bb(i2c_base))
		return 1;

	i2c_setaddress(i2c_base, chip);
	while (alen) {
		alen--;
		/* high byte address going out first */
		writel((addr >> (alen * 8)) & 0xff,
		       &i2c_base->ic_cmd_data);
	}
	return 0;
}

static int i2c_xfer_finish(struct i2c_regs *i2c_base)
{
	int cnt = 1000;
	while (1) {
		if ((readl(&i2c_base->ic_raw_intr_stat) & IC_STOP_DET)) {
			readl(&i2c_base->ic_clr_stop_det);
			break;
		}
		cnt--;
		DELAY_MICROS(1);
		if (cnt <= 0) {
			break;
		}
	}

	if (i2c_wait_for_bb(i2c_base)) {
		LOG_E("Timed out waiting for bus\n");
		return 1;
	}

	i2c_flush_rxfifo(i2c_base);

	return 0;
}

/*
 * i2c_read - Read from i2c memory
 * @chip:	target i2c address
 * @addr:	address to read from
 * @alen:
 * @buffer:	buffer for read data
 * @len:	no of bytes to be read
 *
 * Read from i2c memory.
 */
static int __dw_i2c_read(struct i2c_regs *i2c_base, rt_uint8_t dev, rt_uint32_t addr,
			 int alen, rt_uint8_t *buffer, int len)
{
	unsigned int active = 0;
	int cnt = 10000;
#ifdef CONFIG_SYS_I2C_EEPROM_ADDR_OVERFLOW
	/*
	 * EEPROM chips that implement "address overflow" are ones
	 * like Catalyst 24WC04/08/16 which has 9/10/11 bits of
	 * address and the extra bits end up in the "chip address"
	 * bit slots. This makes a 24WC08 (1Kbyte) chip look like
	 * four 256 byte chips.
	 *
	 * Note that we consider the length of the address field to
	 * still be one byte because the extra address bits are
	 * hidden in the chip address.
	 */
	dev |= ((addr >> (alen * 8)) & CONFIG_SYS_I2C_EEPROM_ADDR_OVERFLOW);
	addr &= ~(CONFIG_SYS_I2C_EEPROM_ADDR_OVERFLOW << (alen * 8));

	LOG_D("%s: fix addr_overflow: dev %02x addr %02x\n", __func__, dev,
	      addr);
#endif

	LOG_D("%s: dev 0x%02x addr 0x%02x alen %d len %d\n", __func__, dev,
	      addr, alen, len);

	if (i2c_xfer_init(i2c_base, dev, addr, alen))
		return 1;

	/* send the saved register address again before read
	 * alen = 0 only when called by designware_i2c_xfer()
	 */
	if (alen == 0)
		writel(addr & 0xff, &i2c_base->ic_cmd_data);

	while (len) {
		if (!active) {
			/*
			 * Avoid writing to ic_cmd_data multiple times
			 * in case this loop spins too quickly and the
			 * ic_status RFNE bit isn't set after the first
			 * write. Subsequent writes to ic_cmd_data can
			 * trigger spurious i2c transfer.
			 */
			if (len == 1)
				writel(IC_CMD | IC_STOP | IC_START_DET, &i2c_base->ic_cmd_data);
			else
				writel(IC_CMD, &i2c_base->ic_cmd_data);
			active = 1;
		}

		if (readl(&i2c_base->ic_status) & IC_STATUS_RFNE) {
			*buffer++ = (rt_uint8_t)readl(&i2c_base->ic_cmd_data);
			len--;
			cnt--;
			DELAY_MICROS(1);
			active = 0;
		}
		if (cnt <= 0) {
			LOG_E("%s: timeout! dev 0x%02x addr 0x%02x alen %d len %d \n", __func__, dev, addr, alen, len);			
			return 1;
		}
	}
	return i2c_xfer_finish(i2c_base);
}

/*
 * i2c_write - Write to i2c memory
 * @chip:	target i2c address
 * @addr:	address to read from
 * @alen:
 * @buffer:	buffer for read data
 * @len:	no of bytes to be read
 *
 * Write to i2c memory.
 */
static int __dw_i2c_write(struct i2c_regs *i2c_base, rt_uint8_t dev, rt_uint32_t addr,
			  int alen, rt_uint8_t *buffer, int len)
{
	int nb = len;
	int cnt = 10000;
#ifdef CONFIG_SYS_I2C_EEPROM_ADDR_OVERFLOW
	/*
	 * EEPROM chips that implement "address overflow" are ones
	 * like Catalyst 24WC04/08/16 which has 9/10/11 bits of
	 * address and the extra bits end up in the "chip address"
	 * bit slots. This makes a 24WC08 (1Kbyte) chip look like
	 * four 256 byte chips.
	 *
	 * Note that we consider the length of the address field to
	 * still be one byte because the extra address bits are
	 * hidden in the chip address.
	 */
	dev |= ((addr >> (alen * 8)) & CONFIG_SYS_I2C_EEPROM_ADDR_OVERFLOW);
	addr &= ~(CONFIG_SYS_I2C_EEPROM_ADDR_OVERFLOW << (alen * 8));

	LOG_D("%s: fix addr_overflow: dev %02x addr %02x\n", __func__, dev,
	      addr);
#endif

	if (i2c_xfer_init(i2c_base, dev, addr, alen))
		return 1;

	/* save the register address for the next read operation
	 * During read operation, when it is called by designware_i2c_xfer(),
	 * nb = 1 indicate only have the register address, don't have
	 * any value written to the register
	 */
	if (nb == 1)
		g_read_reg_addr = *buffer;

	while (len) {
		if (readl(&i2c_base->ic_status) & IC_STATUS_TFNF) {
			if (--len == 0) {
				writel(*buffer | IC_STOP,
				       &i2c_base->ic_cmd_data);
			} else {
				writel(*buffer, &i2c_base->ic_cmd_data);
			}
			buffer++;
			cnt--;
			DELAY_MICROS(1);

		}
		if (cnt <= 0) {
			LOG_E("Timed out. i2c write Failed\n");
			return 1;
		}
	}

	return i2c_xfer_finish(i2c_base);
}

// /*
//  * __dw_i2c_init - Init function
//  * @speed:	required i2c speed
//  * @slaveaddr:	slave address for the device
//  *
//  * Initialization function.
//  */
// static void __dw_i2c_init(struct i2c_regs *i2c_base, int speed, int slaveaddr)
// {
// 	/* Disable i2c */
// 	dw_i2c_enable(i2c_base, RT_FALSE);

// 	writel(IC_CON_SD | IC_CON_RE | IC_CON_SPD_FS | IC_CON_MM,
// 	       &i2c_base->ic_con);
// 	writel(IC_RX_TL, &i2c_base->ic_rx_tl);
// 	writel(IC_TX_TL, &i2c_base->ic_tx_tl);
// 	writel(IC_STOP_DET, &i2c_base->ic_intr_mask);

// 	__dw_i2c_set_bus_speed(i2c_base, NULL, speed);
// 	writel(slaveaddr, &i2c_base->ic_sar);

// 	/* Enable i2c */
// 	dw_i2c_enable(i2c_base, RT_TRUE);
// }

/*
 * The legacy I2C functions. These need to get removed once
 * all users of this driver are converted to DM.
 */
static struct i2c_regs *i2c_get_base(struct i2c_adapter *adap)
{
	LOG_D("i2c_get_base adap id %d\n", adap->id);
	switch (adap->id) {
	case 5:
		return (struct i2c_regs *)i2c5_base_addr;		
	case 4:
		return (struct i2c_regs *)i2c4_base_addr;		
	case 3:
		return (struct i2c_regs *)i2c3_base_addr;
	case 2:
		return (struct i2c_regs *)i2c2_base_addr;
	case 1:
		return (struct i2c_regs *)i2c1_base_addr;
	case 0:
		return (struct i2c_regs *)i2c0_base_addr;
	default:
		LOG_I("Wrong I2C-adapter id %d\n", adap->id);
	}

	return NULL;
}

static unsigned int dw_i2c_set_bus_speed(struct i2c_adapter *adap, rt_uint32_t speed)
{
	adap->speed = speed;
	return __dw_i2c_set_bus_speed(i2c_get_base(adap), NULL, speed);
}

static int dw_i2c_read(struct i2c_adapter *adap, rt_uint8_t dev, int alen, rt_uint8_t *buffer, int len)
{
	return __dw_i2c_read(i2c_get_base(adap), dev, buffer[0], alen, &buffer[1], len-1);
}

static int dw_i2c_write(struct i2c_adapter *adap, rt_uint8_t dev, int alen, rt_uint8_t *buffer, int len)
{
	return __dw_i2c_write(i2c_get_base(adap), dev, buffer[0], alen, buffer, len);
}

static rt_ssize_t jlq_i2c_mst_xfer(struct rt_i2c_bus_device *bus,
                                    struct rt_i2c_msg msgs[],
                                    rt_uint32_t num)
{
    rt_size_t i;
    rt_uint8_t reason;
    RT_ASSERT(bus != RT_NULL);

    struct i2c_adapter *adap = (struct i2c_adapter*)(bus->priv);

    for (i = 0; i < num; i++)
    {
        if (msgs[i].flags & RT_I2C_RD)
			reason = dw_i2c_read(adap, msgs[i].addr, (msgs[i].flags&RT_I2C_ADDR_10BIT)>>2, msgs[i].buf, msgs[i].len);
        else
			reason = dw_i2c_write(adap, msgs[i].addr, (msgs[i].flags&RT_I2C_ADDR_10BIT)>>2, msgs[i].buf, msgs[i].len);
    }
	LOG_D("%s reason=%d!\n",__func__, reason);	
    return (reason == 0)? i : 0;
}

static rt_err_t jlq_i2c_bus_control(struct rt_i2c_bus_device *bus,
                                      rt_uint32_t cmd,
                                      rt_uint32_t arg)
{
	struct i2c_adapter *adap = (struct i2c_adapter*)(bus->priv);
	switch (cmd)
	{
		case RT_I2C_DEV_CTRL_CLK:
			dw_i2c_set_bus_speed(adap, arg);
			break;
		default:
			return -RT_EIO;
	}
	return RT_EOK;
}

static rt_err_t jlq_i2c_configure(struct i2c_adapter *cfg)
{
    RT_ASSERT(cfg != RT_NULL);

	/*Set I2C muxpin ctrl*/
	writel(0x18, gpio_mux_base_addr + cfg->scl_muxpin);//scl
	writel(0x18, gpio_mux_base_addr + cfg->sda_muxpin);//sda

    return RT_EOK;
}

static const struct rt_i2c_bus_device_ops jlq_i2c_ops =
{
    .master_xfer = jlq_i2c_mst_xfer,
    .slave_xfer = NULL,
    .i2c_bus_control = jlq_i2c_bus_control,
};

#if defined (BSP_USING_I2C0)
#define I2C0_BUS_NAME    "i2c0"
static struct i2c_adapter hw_device0 =
{
	.id = 0,
	.scl_muxpin = I2C0_SCL_MUXPIN,
	.sda_muxpin = I2C0_SDA_MUXPIN,
};

struct rt_i2c_bus_device device0 =
{
    .ops = &jlq_i2c_ops,
    .priv =  (void *)&hw_device0,
};
#endif

#if defined (BSP_USING_I2C1)
#define I2C1_BUS_NAME    "i2c1"
static struct i2c_adapter hw_device1 =
{
	.id = 1,
	.scl_muxpin = I2C1_SCL_MUXPIN,
	.sda_muxpin = I2C1_SDA_MUXPIN,
};

struct rt_i2c_bus_device device1 =
{
    .ops = &jlq_i2c_ops,
    .priv =  (void *)&hw_device1,
};
#endif

#if defined (BSP_USING_I2C2)
#define I2C2_BUS_NAME    "i2c2"
static struct i2c_adapter hw_device2 =
{
	.id = 2,
	.scl_muxpin = I2C2_SCL_MUXPIN,
	.sda_muxpin = I2C2_SDA_MUXPIN,
};

struct rt_i2c_bus_device device2 =
{
    .ops = &jlq_i2c_ops,
    .priv =  (void *)&hw_device2,
};
#endif


#if defined (BSP_USING_I2C3)
#define I2C3_BUS_NAME    "i2c3"
static struct i2c_adapter hw_device3 =
{
    .id = 3,
	.scl_muxpin = I2C3_SCL_MUXPIN,
	.sda_muxpin = I2C3_SDA_MUXPIN,
};

struct rt_i2c_bus_device device3 =
{
    .ops = &jlq_i2c_ops,
    .priv =  (void *)&hw_device3,
};
#endif

#if defined (BSP_USING_I2C4)
#define I2C4_BUS_NAME    "i2c4"
static struct i2c_adapter hw_device4 =
{
    .id = 4,
	.scl_muxpin = I2C4_SCL_MUXPIN,
	.sda_muxpin = I2C4_SDA_MUXPIN,
};

struct rt_i2c_bus_device device4 =
{
    .ops = &jlq_i2c_ops,
    .priv =  (void *)&hw_device4,
};
#endif

#if defined (BSP_USING_I2C5)
#define I2C5_BUS_NAME    "i2c5"
static struct i2c_adapter hw_device5 =
{
    .id = 5,
	.scl_muxpin = I2C5_SCL_MUXPIN;
	.sda_muxpin = I2C5_SDA_MUXPIN;
};

struct rt_i2c_bus_device device5 =
{
    .ops = &jlq_i2c_ops,
    .priv =  (void *)&hw_device5,
};
#endif

int rt_hw_i2c_init(void)
{
#if defined(BSP_USING_I2C0)
    jlq_i2c_configure(&hw_device0);
    rt_i2c_bus_device_register(&device0, I2C0_BUS_NAME);
#endif

#if defined(BSP_USING_I2C1)
    jlq_i2c_configure(&hw_device1);
    rt_i2c_bus_device_register(&device1, I2C1_BUS_NAME);
#endif

#if defined(BSP_USING_I2C2)
    jlq_i2c_configure(&hw_device2);
    rt_i2c_bus_device_register(&device2, I2C2_BUS_NAME);
#endif

#if defined(BSP_USING_I2C3)
    jlq_i2c_configure(&hw_device3);
    rt_i2c_bus_device_register(&device3, I2C3_BUS_NAME);
#endif

#if defined(BSP_USING_I2C4)
    jlq_i2c_configure(&hw_device4);
    rt_i2c_bus_device_register(&device4, I2C4_BUS_NAME);
#endif

#if defined(BSP_USING_I2C5)
    jlq_i2c_configure(&hw_device5);
    rt_i2c_bus_device_register(&device5, I2C5_BUS_NAME);
#endif

    return 0;
}

INIT_BOARD_EXPORT(rt_hw_i2c_init);