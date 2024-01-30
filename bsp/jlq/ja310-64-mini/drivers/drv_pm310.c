/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author         Notes
 * 2020-11-28     bigmagic       first version
 */
#include <rtthread.h>
#include <rtdevice.h>
#include <stdio.h>
#include "drv_pm310.h"
#include "board.h"


static struct rt_i2c_bus_device *pm310_i2c_bus = RT_NULL;
static rt_bool_t initialized = RT_FALSE;
static const rt_int8_t pm310_ldo_ctrl_reg[] = {
	0xff,
	0x3f, /*LDOCR1*/
	0x38, /*LDOCR2*/
	0x3e, /*LDOCR3*/
	0x35, /*LDOCR4*/
	0x47, /*LDOCR5*/
	0x40, /*LDOCR6*/
	0x48, /*LDOCR7*/
	0x34, /*LDOCR8*/
	0x41, /*LDOCR9*/
	0x2c, /*LDOCR10*/
	0x2e, /*LDOCR11*/
	0x46, /*LDOCR12*/
	0x42, /*LDOCR13*/
	0x32, /*LDOCR14*/
	0x26, /*LDOCR15*/
	0x33, /*LDOCR16*/
	0x43, /*LDOCR17*/
	0x31, /*LDOCR18*/
	0x21, /*LDOCR19*/
	0x30  /*LDOCR20*/
};


 /*
 * pm310_write_reg - Write to pm310 chip
 * @reg:	address to write to chip
 * @data:	value write to address
 *
 * Write to pm310 chip register by i2c.
 */
rt_err_t pm310_write_reg(rt_uint8_t reg, rt_uint8_t data)
{
    rt_uint8_t buf[2] = {0,0};
    struct rt_i2c_msg msgs;
    rt_uint32_t buf_size = 1;
    if (pm310_i2c_bus == RT_NULL)
    {
        if(rt_hw_pm310_init() != RT_EOK)
        {
            return -RT_ERROR;
        }
    }
    buf[0] = reg; //cmd
    if (data != RT_NULL)
    {
        buf[1] = data;
        buf_size = 2;
    }

    msgs.addr = PM310_SLAVE_ADDR;
    msgs.flags = RT_I2C_WR;
    msgs.buf = buf;
    msgs.len = buf_size;

    if (rt_i2c_transfer(pm310_i2c_bus, &msgs, 1) == 1)
    {
        return RT_EOK;
    }
    else
    {
        return -RT_ERROR;
    }
}

 /*
 * pm310_read_reg - read from pm310 chip
 * @reg:	address to read from
 * @value:	value read from address
 *
 * Read from pm310 chip register by i2c.
 */
rt_err_t pm310_read_reg(rt_uint8_t reg, rt_uint8_t *value)
{
    struct rt_i2c_msg msgs;
    rt_uint8_t buf[2] = {0,0};
    msgs.addr = PM310_SLAVE_ADDR;
    msgs.flags = RT_I2C_RD;
    buf[0] = reg;
    *value = buf[1];
    msgs.buf = buf;
    msgs.len = 2;

    if (pm310_i2c_bus == RT_NULL)
    {
        if(rt_hw_pm310_init() != RT_EOK)
        {
            return -RT_ERROR;
        }
    }

    if (rt_i2c_transfer(pm310_i2c_bus, &msgs, 1) == 1)
    {
        *value = buf[1];
        return RT_EOK;
    }
    else
    {
        return -RT_ERROR;
    }
}

 /*
 * pm310_ldo_set - pm310 ldo set control
 * @id:	    LDO Id
 * @uV:	    voltage to set to LDO
 * @enable:	enable or disable.
 *
 * PM310 Digital LDO control
 */
rt_int32_t pm310_ldo_set(rt_uint8_t id, rt_uint32_t uV, rt_bool_t enable)
{
	rt_uint8_t val = 0;
	rt_uint8_t hex;
    rt_uint32_t min_uV;
    rt_uint32_t ldo_step = PM310_LDO_UV_HSTEP;

	if (id < 1 || id > PM310_LDO_NUM) {
		rt_kprintf("Wrong ldo number: %d", id);
		return -RT_ERROR;
	}

	switch (id) {
        case 2:
        case 12:
        case 15:
        case 17:
            min_uV = 850000;
            ldo_step = PM310_LDO_UV_HSTEP;
            break;
        case 4:
            min_uV = 900000;
            ldo_step = PM310_LDO_UV_HSTEP;
            break;        
        case 18:
        case 19:        
            min_uV = 700000;
            ldo_step = PM310_LDO_UV_LSTEP;
            break;             

        default:
            min_uV = 1750000;
            ldo_step = PM310_LDO_UV_HSTEP;
	}
    hex = (uV - min_uV) / ldo_step;
    val |= hex;

    if(enable)
        val |= PM310_EN_LDO_M|PM310_EN_LDO_FST;
    if(pm310_write_reg(pm310_ldo_ctrl_reg[id], val) != RT_EOK)
    {
        rt_kprintf("%s ldo[%d] set fail!\n", __func__, id);
        return -RT_ERROR;
    }

	return RT_EOK;
}

 /*
 * pm310_ldo_get - get pm310 ldo status
 * @id:	    LDO Id
 * @uV:	    voltage value from LDO
 * @status:	enable or disable.
 *
 * PM310 Digital LDO Status
 */
rt_int32_t pm310_ldo_get(rt_uint8_t id, rt_uint32_t *uV, rt_bool_t *status)
{
	rt_uint8_t val = 0;
	rt_uint8_t hex;
    rt_uint32_t min_uV;
    rt_uint32_t ldo_step = PM310_LDO_UV_HSTEP;

	if (id < 1 || id > PM310_LDO_NUM) {
		rt_kprintf("Wrong ldo number: %d", id);
		return -RT_ERROR;
	}

    if(pm310_read_reg(pm310_ldo_ctrl_reg[id], &val) != RT_EOK)
    {
        rt_kprintf("%s ldo[%d] get fail!\n", __func__, id);
        return -RT_ERROR;
    }

    hex = val&0x3f;
    if((val & PM310_EN_LDO_M) == PM310_EN_LDO_M)
    {
        *status = RT_TRUE;
    }
    else
    {
        *status = RT_FALSE;
    }

	switch (id) {
        case 2:
        case 12:
        case 15:
        case 17:
            min_uV = 850000;
            ldo_step = PM310_LDO_UV_HSTEP;
            break;
        case 4:
            min_uV = 900000;
            ldo_step = PM310_LDO_UV_HSTEP;
            break;        
        case 18:
        case 19:        
            min_uV = 700000;
            ldo_step = PM310_LDO_UV_LSTEP;
            break;             

        default:
            min_uV = 1750000;
            ldo_step = PM310_LDO_UV_HSTEP;
	}
    *uV = hex * ldo_step + min_uV;

	return RT_EOK;    
}
void pm310_ldo_dump()
{
    rt_uint32_t uV;
    rt_bool_t status;
    rt_kprintf("LDO\tVoltage\tStatus\n");
    for(int i=1; i<=20; i++)
    {
        pm310_ldo_get(i, &uV, &status);
        rt_kprintf("%d\t%d\t%d\n", i, uV, status);
    }
}
MSH_CMD_EXPORT(pm310_ldo_dump, Dump pm310 ldo status);

rt_int32_t rt_hw_pm310_init()
{
    rt_uint8_t version = 0;

    if(initialized == RT_TRUE)
    {
        rt_kprintf("pm310 already initialized!\n");
        return RT_EOK;
    }

    /* 查找I2C总线设备，获取I2C总线设备句柄 */
    pm310_i2c_bus = (struct rt_i2c_bus_device *)rt_device_find(PM310_I2C_BUS_NAME);

    if (pm310_i2c_bus == RT_NULL)
    {
        rt_kprintf("%s can't find %s device!\n", __func__, PM310_I2C_BUS_NAME);
        return -RT_ERROR;
    }
    else
    {
        if(rt_i2c_control(pm310_i2c_bus, RT_I2C_DEV_CTRL_CLK, PM310_I2C_CLOCK) != RT_EOK)
        {
           rt_kprintf("%s Set I2C clock error!\n", __func__);
           return -RT_ERROR; 
        }    

        pm310_read_reg(PM310_VERSION_REG, &version);
        if(version == PM310_CHIP_VERSION)
        {
            initialized = RT_TRUE;
            rt_kprintf("%s ok, version=0x%x!\n",__func__, version);
            return RT_EOK;
        }
        else
        {
           rt_kprintf("%s fail, wrong version=0x%x!\n",__func__, version);
           return -RT_ERROR;
        }
    }
}
// INIT_DEVICE_EXPORT(rt_hw_pm310_init);

int pm310_reg(int argc, char **argv)
{
    rt_err_t result = RT_EOK;
    rt_uint8_t val = 0;
    if (argc == 2)
    {
        unsigned int reg_addr;
        sscanf(argv[1], "0x%x", &reg_addr);
        if (pm310_i2c_bus == RT_NULL)
        {
            rt_kprintf("pm310_i2c_bus is null\n");
            return -RT_ERROR;
        }
        pm310_read_reg((rt_uint8_t)reg_addr, &val);
        rt_kprintf("0x%x\n",val);
    }
    else if(argc == 3)
    {
        unsigned int reg_addr;
        unsigned int reg_val;
        sscanf(argv[1], "0x%x", &reg_addr);
        sscanf(argv[2], "0x%x", &reg_val);

        if(pm310_write_reg((rt_uint8_t)reg_addr, (rt_uint8_t)reg_val) != RT_EOK)
            rt_kprintf("write reg fail!\n");
        rt_kprintf("ok!\n");
    }
    else
    {
        rt_kprintf("Usage: \n");
        rt_kprintf("pm310_reg <reg addr>[0x..]           - Read the value from register\n");
        rt_kprintf("pm310_reg <reg addr>[0x..] <value>   - Write the value to register\n");
        result = -RT_ERROR;
    }
    return result;
}
MSH_CMD_EXPORT(pm310_reg, Read or Write pm310 chip register);
