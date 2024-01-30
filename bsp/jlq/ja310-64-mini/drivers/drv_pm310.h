/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author         Notes
 * 2020-11-28     bigmagic       first version
 */

#ifndef __DRV_PM310_H__
#define __DRV_PM310_H__

#include <rthw.h>

#define PM310_I2C_BUS_NAME  "i2c3"
#define PM310_SLAVE_ADDR    0x33 // PM310 Slave address
#define PM310_I2C_CLOCK 400000 // 400KHz

#define PM310_VERSION_REG   0x00
#define PM310_CHIP_VERSION  0x10

#define PM310_LDO_NUM	    20

#define PM310_LDO_UV_LSTEP		25000 /* uV lower value step */
#define PM310_LDO_UV_HSTEP		50000 /* uV higher value step */

/* mask/shift bits */
#define PM310_EN_LDO_M				BIT(7)
#define PM310_EN_LDO_FST			BIT(6)
#define PM310_EN_LDO_SLP			BIT(5)

rt_int32_t rt_hw_pm310_init();
rt_err_t pm310_write_reg(rt_uint8_t reg, rt_uint8_t data);
rt_err_t pm310_read_reg(rt_uint8_t reg, rt_uint8_t *value);
rt_int32_t pm310_ldo_set(rt_uint8_t id, rt_uint32_t uV, rt_bool_t enable);

#endif
