/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author         Notes
 * 2020-04-16     bigmagic       first version
 */

#ifndef BOARD_H__
#define BOARD_H__

#include <stdint.h>
#include "ja310.h"
#include "mmu.h"
#include "ioremap.h"
#include <rtdevice.h>

extern int __bss_end;
#define HEAP_BEGIN      ((void*)&__bss_end)

#ifdef RT_USING_SMART
#define HEAP_END        ((size_t)KERNEL_VADDR_START + 32 * 1024 * 1024)
#define PAGE_START      HEAP_END
#define PAGE_END        ((size_t)KERNEL_VADDR_START + 128 * 1024 * 1024)
#else
#define KERNEL_VADDR_START 0x0
#define PV_OFFSET   0x0
#define HEAP_END        (KERNEL_VADDR_START + 64 * 1024 * 1024)
#define PAGE_START      HEAP_END
#define PAGE_END        ((size_t)PAGE_START + 64 * 1024 * 1024)
#endif

#ifdef RT_USING_SMP
#define PSCI_CPU_OFF        0x84000002
#define PSCI_CPU_ON_32      0x84000003
#define PSCI_CPU_ON_64      0xc4000003

/* PSCI return values (inclusive of all PSCI versions) */
#define PSCI_RET_SUCCESS			0
#define PSCI_RET_NOT_SUPPORTED			-1
#define PSCI_RET_INVALID_PARAMS			-2
#define PSCI_RET_DENIED				-3
#define PSCI_RET_ALREADY_ON			-4
#define PSCI_RET_ON_PENDING			-5
#define PSCI_RET_INTERNAL_FAILURE		-6
#define PSCI_RET_NOT_PRESENT			-7
#define PSCI_RET_DISABLED			-8
#define PSCI_RET_INVALID_ADDRESS		-9
#endif

void rt_hw_board_init(void);

void rt_gpio_dbg(int value);
#define readl(addr)           (*(volatile unsigned int *)(addr))
#define writel(value,addr)    (*(volatile unsigned int *)(addr) = (value))


#endif
