/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author         Notes
 * 2020-10-27     bigmagic       first version
 */


#ifndef __DRV_RTC_H__
#define __DRV_RTC_H__

#include <rtthread.h>
#include <rtdevice.h>
#include <sys/time.h>
#include "drv_pm310.h"

// struct rt_rtc_device
// {
//     struct rt_device device;
// };

/* RTC Register */
#define PM310_REG_RTC_SEC					0x60
#define PM310_REG_RTC_MIN					0x61
#define PM310_REG_RTC_HOUR					0x62
#define PM310_REG_RTC_DAY					0x63
#define PM310_REG_RTC_WEEK					0x64
#define PM310_REG_RTC_MONTH				    0x65
#define PM310_REG_RTC_YEAR					0x66
#define PM310_REG_RTC_TIME_UPDATE			0x67
#define PM310_REG_RTC_32K_ADJL				0x68
#define PM310_REG_RTC_32K_ADJH				0x69
#define PM310_REG_RTC_SEC_CYCL				0x6a
#define PM310_REG_RTC_SEC_CYCH				0x6b
#define PM310_REG_RTC_CALIB_UPDATE			0x6c
#define PM310_REG_RTC_AL1_SEC				0x6d
#define PM310_REG_RTC_AL1_MIN				0x6e
#define PM310_REG_RTC_AL2_SEC				0x6f
#define PM310_REG_RTC_AL2_MIN				0x70
#define PM310_REG_RTC_AL2_HOUR				0x71
#define PM310_REG_RTC_AL2_DAY				0x72
#define PM310_REG_RTC_AL2_MONTH			    0x73
#define PM310_REG_RTC_AL2_YEAR				0x74
#define PM310_REG_RTC_AL_UPDATE			    0x75
#define PM310_REG_RTC_INT_EN				0x76
#define PM310_REG_RTC_INT_STATUS			0x77
#define PM310_REG_RTC_INT_RAW				0x78
#define PM310_REG_RTC_CTRL					0x79
#define PM310_REG_RTC_BK					0x7a
#define PM310_REG_RTC_INT_MASK				0x7b
#define PM310_REG_RTC_AL1_HOUR				0x7c
#define PM310_REG_RTC_AL1_DAY				0x7d
#define PM310_REG_RTC_AL1_MONTH			    0x7e
#define PM310_REG_RTC_AL1_YEAR				0x7f
#define PM310_REG_RTC_DATA1				    0x80
#define PM310_REG_RTC_DATA2				    0x81
#define PM310_REG_RTC_DATA3				    0x82
#define PM310_REG_RTC_DATA4				    0x83
#define PM310_REG_SMPL_CR					0x84

/* RTC_SEC: RTC Second Counter Register(0x60) */
#define PM310_REG_BITMASK_RTC_SEC					0x3f
/* RTC_MIN: RTC Minute Counter Register(0x61) */
#define PM310_REG_BITMASK_RTC_MIN					0x3f
/* RTC_HOUR: RTC Hour Counter Register(0x62) */
#define PM310_REG_BITMASK_RTC_HOUR					0x1f
/* RTC_DAY: RTC Day of a Month Counter Register(0x63) */
#define PM310_REG_BITMASK_RTC_DAY					0x1f
/* RTC_WEEK: RTC Current Day of a Week Counter Register(0x64) */
#define PM310_REG_BITMASK_RTC_WEEK					0x07
/* RTC_MONTH: RTC Month Counter Register(0x65) */
#define PM310_REG_BITMASK_RTC_MONTH					0x0f
/* RTC_YEAR: RTC Year Counter Register(0x66) */
#define PM310_REG_BITMASK_RTC_YEAR					0x7f
/* RTC_TIME_UPDATE: RTC Time Update Enable Register(0x67) */
#define PM310_REG_BITMASK_RTC_TIME_UPDATE			0xff

/* PM310 week Days. */
#define PM310_RTC_WEEK_DAYS			(7)

/* PM310 RTC base year. */
#define PM310_RTC_YEAR_BASE	(1980)
#define PM310_RTC_YEAR_MIN (1980)
#define PM310_RTC_YEAR_MAX (2038)

#define PM310_RTC_YEAR_CHECK(year)	\
		(((year) < (PM310_RTC_YEAR_MIN - 1900)) \
		|| ((year) > (PM310_RTC_YEAR_MAX - 1900)))

#define REG2BCD(val, mask)		(val & mask)
#define BCD2REG(val, mask)		(val & mask)
#define ALL_TIME_REGS			(7)

int rt_hw_rtc_init(void);

#endif
