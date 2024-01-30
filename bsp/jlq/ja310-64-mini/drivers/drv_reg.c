/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-3-08      GuEe-GUI     the first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <stdio.h>
#include <board.h>

int jlq_reg(int argc, char **argv)
{
    rt_err_t result = RT_EOK;

    if (argc == 2)
    {
        size_t reg_addr;
        sscanf(argv[1], "0x%lx", &reg_addr);
        #ifdef RT_USING_SMART
        reg_addr = (size_t)rt_ioremap((void*)reg_addr, 100);
        #endif
        rt_kprintf("0x%x\n",readl(reg_addr));
    }
    else if(argc == 3)
    {
        size_t reg_addr;
        int reg_val;
        sscanf(argv[1], "0x%lx", &reg_addr);
        sscanf(argv[2], "0x%x", &reg_val);

        #ifdef RT_USING_SMART
        reg_addr = (size_t)rt_ioremap((void*)reg_addr, 4);
        #endif
        writel(reg_val, reg_addr);
        rt_kprintf("ok!\n");
    }
    else
    {
        rt_kprintf("Usage: \n");
        rt_kprintf("jlq_reg <reg addr>[0x...]           - Read the value from register\n");
        rt_kprintf("jlq_reg <reg addr>[0x...] <value>   - Write the value to register\n");
        result = -RT_ERROR;
    }
    return result;
}
MSH_CMD_EXPORT(jlq_reg, Read or Write 32-bit register);