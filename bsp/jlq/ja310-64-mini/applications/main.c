/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author         Notes
 * 2020-04-16     bigmagic       first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#ifdef RT_USING_SMP

struct rt_thread jlq_demo;

static rt_uint8_t jlq_demo_stack[1024];

static void jlq_demo_thread(void *arg)
{
    rt_base_t level;
    while (1)
    {
        /* code */
        // level = rt_cpus_lock();
        // rt_kprintf("core%d \r\n", rt_hw_cpu_id());
        // rt_cpus_unlock(level);
        rt_thread_mdelay(5000);
    }
}

void jlq_demo_init(void)
{
    rt_ubase_t i;
    rt_ubase_t cpu_id = 1; //random cpu id

    rt_thread_init(&jlq_demo, "jlq_demo", jlq_demo_thread,
                       RT_NULL,
                       &jlq_demo_stack,
                       sizeof(jlq_demo_stack),
                       21,
                       32);

    rt_thread_control(&jlq_demo, RT_THREAD_CTRL_BIND_CPU, (void *)cpu_id);
    rt_thread_startup(&jlq_demo);

}
#endif

int main(int argc, char** argv)
{
    rt_thread_mdelay(1*1000);
    rt_kprintf("\nJLQ-JA310 RT-Thread Demo!!\n\r");

#ifdef RT_USING_SMP
    jlq_demo_init();
#endif

    return 0;
}
