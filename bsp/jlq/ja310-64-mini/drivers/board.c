/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author         Notes
 * 2020-04-16     bigmagic       first version
 */

#include <rthw.h>
#include <rtthread.h>
#include <mm_aspace.h>

#include "board.h"
#include "drv_uart.h"
#include "cp15.h"
#include "mmu.h"

#include <mm_page.h>

#ifdef RT_USING_SMART
#include <lwp_arch.h>
#endif

extern size_t MMUTable[];

size_t gpio_base_addr = GPIO_BASE;
size_t gpio_mux_base_addr = GPIO_MUX_BASE;
size_t ap_pwr_base_addr = AP_PWR_BASE;
size_t ddr_pwr_base_addr = DDR_PWR_BASE;
size_t sdio0_base_addr   = SDIO0_BASE_ADDR;
size_t i2c0_base_addr = I2C0_BASE_ADDR;
size_t i2c1_base_addr = I2C1_BASE_ADDR;
size_t i2c2_base_addr = I2C2_BASE_ADDR;
size_t i2c3_base_addr = I2C3_BASE_ADDR;
size_t i2c4_base_addr = I2C4_BASE_ADDR;
size_t i2c5_base_addr = I2C5_BASE_ADDR;
void rt_gpio_dbg(int value) {
#if 0 // Debug for raspi 4B board --- GPIO16
    for(int i=0;i<value;i++)
    {
        writel(0x52000,0xFE200000+0x4);
        writel(0x6771696f,0xFE200000+0x1c);
        for(int j=50000000;j>0;j--);
        writel(0x12000,0xFE200000+0x4);
        writel(0x6770696f,0xFE200000+0x1c);      
        for(int j=50000000;j>0;j--);  
    }
    writel(0x52000,0xFE200000+0x4);
    writel(0x6771696f,0xFE200000+0x1c);
#else // Debug for ja310 evb
    writel(0x1C, gpio_mux_base_addr+0x70);       // set GPIO34 mux
    writel(0x00040024, gpio_base_addr+0x48); // set GPIO34 output
    for (int i = 0; i < value; i++) {

        writel(0x00040024, gpio_base_addr+0x08);

        DELAY_MICROS(1);

        writel(0x00040020, gpio_base_addr+0x08);

        DELAY_MICROS(1);
    }

    DELAY_MICROS(10);; // delay 500ms
#endif
}

#ifdef RT_USING_SMART
struct mem_desc platform_mem_desc[] = {
    {KERNEL_VADDR_START, KERNEL_VADDR_START + 0x0fffffff, (rt_size_t)ARCH_MAP_FAILED, NORMAL_MEM}};
#else
struct mem_desc platform_mem_desc[] = {
    {0x00200000, (128ul << 20) - 1, 0x00200000, NORMAL_MEM},
    {0xF9000000, 0x000100000000 - 1, 0xF9000000, DEVICE_MEM},
};
#endif

const rt_uint32_t platform_mem_desc_size = sizeof(platform_mem_desc) / sizeof(platform_mem_desc[0]);

void idle_wfi(void) { asm volatile("wfi"); }

/**
 * This function will initialize board
 */

extern size_t MMUTable[];
int rt_hw_gtimer_init(void);

rt_region_t init_page_region = {
    PAGE_START,
    PAGE_END,
};

__attribute__((__noreturn__)) void reboot(void) {
    unsigned int val = 0;

    writel(DDRPWR_RES_REG0_REBOOT_FLAG, DDR_PWR_RES_REG0);

    val = readl(AP_PWR_CHIPRSTN_CTL);
    val &= ~(1 << SFRST_CHIP_RSTN_CNT_TRIGGER_MK);//pmic reset
    val &= ~(1 << CHIP_RSTN_MK);
    val |= (1 << SFRST_PWR2SYS_RSTN_MK);
    val |= CHIP_RSTN_CNT_LIMIT;
    writel(val, AP_PWR_CHIPRSTN_CTL);
    rt_kprintf("DDR_PWR_RES_REG0 = 0x%x\n",readl(DDR_PWR_RES_REG0));
    rt_kprintf("AP_PWR_CHIPRSTN_CTL = 0x%x\n",readl(AP_PWR_CHIPRSTN_CTL));
    __DSB();
    writel(0xFF, AP_PWR_SFRST_CTL);
    __DSB();
    __WFI();
    /* wait for the system to power down */
    for (;;) {
        ;
    }
}
MSH_CMD_EXPORT(reboot, ja310 reboot);

__attribute__((__noreturn__)) void poweroff(void) {
    writel(0, AP_PWR_PWEN_CTL);
    __WFI();
    /* wait for the system to power down */
    for (;;) {
        ;
    }
}
MSH_CMD_EXPORT(poweroff, ja310 poweroff);

/**
 *  Initialize the Hardware related stuffs. Called from rtthread_startup()
 *  after interrupt disabled.
 */
void rt_hw_board_init(void) {

    /* io device remap */
#ifdef RT_USING_SMART
    rt_hw_mmu_map_init(&rt_kernel_space, (void *)0xfffffffff0000000, 0x10000000, MMUTable,
                       PV_OFFSET);
#else
    rt_hw_mmu_map_init(&rt_kernel_space, (void *)0x000400000000, 0x10000000, MMUTable, 0);
#endif

    rt_page_init(init_page_region);
    rt_hw_mmu_setup(&rt_kernel_space, platform_mem_desc, platform_mem_desc_size);

    // rt_gpio_dbg(1); // For debug

    /* map peripheral address to virtual address */
#ifdef RT_USING_HEAP
    /* initialize system heap */
    rt_system_heap_init((void *)HEAP_BEGIN, (void *)HEAP_END);
#endif

#ifdef RT_USING_SMART
    //gpio
    gpio_base_addr = (size_t)rt_ioremap((void*)GPIO_BASE, 0x1000);
    gpio_mux_base_addr = (size_t)rt_ioremap((void*)GPIO_MUX_BASE, 0x1000);

    ap_pwr_base_addr = (size_t)rt_ioremap((void*)AP_PWR_BASE, AP_PWR_SIZE);
    ddr_pwr_base_addr = (size_t)rt_ioremap((void*)DDR_PWR_BASE, DDR_PWR_SIZE);

    sdio0_base_addr = (size_t)rt_ioremap((void*)SDIO0_BASE_ADDR, 0x1000);
    //i2c
    i2c0_base_addr = (size_t)rt_ioremap((void*)I2C0_BASE_ADDR, 0x1000);
    i2c1_base_addr = (size_t)rt_ioremap((void*)I2C1_BASE_ADDR, 0x1000);
    i2c2_base_addr = (size_t)rt_ioremap((void*)I2C2_BASE_ADDR, 0x1000);
    i2c3_base_addr = (size_t)rt_ioremap((void*)I2C3_BASE_ADDR, 0x1000);
    i2c4_base_addr = (size_t)rt_ioremap((void*)I2C4_BASE_ADDR, 0x1000);
    i2c5_base_addr = (size_t)rt_ioremap((void*)I2C5_BASE_ADDR, 0x1000);
#endif

    /* initialize hardware interrupt */
    rt_hw_interrupt_init();

    // rt_gpio_dbg(2);

    /* initialize uart */
    rt_hw_uart_init();
    // rt_gpio_dbg(3);

    /* initialize timer for os tick */
    rt_hw_gtimer_init();

#ifdef RT_USING_CONSOLE
    /* set console device */
    rt_console_set_device(RT_CONSOLE_DEVICE_NAME);
#endif /* RT_USING_CONSOLE */

    rt_kprintf("heap: 0x%08x - 0x%08x\n", HEAP_BEGIN, HEAP_END);

#ifdef RT_USING_COMPONENTS_INIT
    rt_components_board_init();
#endif
    rt_thread_idle_sethook(idle_wfi);
    // rt_gpio_dbg(5);
}

#ifdef RT_USING_SMP
#include <gicv3.h>
#include <interrupt.h>
#include <smccc.h>

void rt_hw_mmu_ktbl_set(unsigned long tbl);
void _secondary_cpu_entry(void);

void rt_hw_secondary_cpu_up(void)
{
    rt_uint32_t i;
    rt_uint32_t cpu_mask = 0;

    struct arm_smccc_res_t res;

    for (i = 1; i < RT_CPUS_NR;i++)
    {
        cpu_mask = i << 8; //For cotex-a55
        char *entry = (char *)_secondary_cpu_entry;
        entry += PV_OFFSET;
        arm_smccc_smc(PSCI_CPU_ON_64, cpu_mask, (uintptr_t)entry, 0, 0, 0, 0, 0, &res, RT_NULL);
        if(res.a0 != PSCI_RET_SUCCESS)
        {
            rt_kprintf("cpu %d res.a0=%d\n", cpu_mask, res.a0);
        }
        asm volatile ("dsb sy");
        asm volatile ("sev");
    }
}

void rt_hw_secondary_cpu_bsp_start(void)
{
    rt_hw_spin_lock(&_cpus_lock);

    rt_hw_mmu_ktbl_set((unsigned long)MMUTable);

    rt_hw_vector_init();

    arm_gic_cpu_init(0, 0);

    arm_gic_redist_init(0, 0);

    rt_hw_gtimer_init();

    rt_kprintf("cpu %d boot success\n", rt_hw_cpu_id());
    rt_system_scheduler_start();
}

void rt_hw_secondary_cpu_idle_exec(void)
{
    asm volatile ("wfe":::"memory", "cc");
}
#endif
