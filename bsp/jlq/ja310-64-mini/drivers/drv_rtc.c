/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author         Notes
 * 2020-10-27     bigmagic       first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <sys/time.h>
#include "drv_rtc.h"

#ifdef BSP_USING_RTC

#define DBG_TAG               "rtc"
#define DBG_LVL               DBG_INFO
#include <rtdbg.h>

#if defined(RT_USING_SOFT_RTC)
#error "Please CANCEL the RT_USING_SOFT_RTC option. Make sure there is only one RTC device on the system."
#endif

static struct rt_rtc_device rtc;

static rt_err_t pm310_rtc_get_secs(time_t *timestamp)
{
    struct tm tm_new = {0};
	rt_uint8_t rtc_data[ALL_TIME_REGS] = {0};
	int i = 0;
    LOG_D("%s", __func__);
	/* Read hour, minute and second. */
	for (i = 0; i < 7; i++)
		pm310_read_reg(PM310_REG_RTC_SEC + i, &rtc_data[i]);
	tm_new.tm_sec =  REG2BCD(rtc_data[0], PM310_REG_BITMASK_RTC_SEC);
	tm_new.tm_min =  REG2BCD(rtc_data[1], PM310_REG_BITMASK_RTC_MIN);
	tm_new.tm_hour = REG2BCD(rtc_data[2], PM310_REG_BITMASK_RTC_HOUR);
	tm_new.tm_mday = REG2BCD(rtc_data[3], PM310_REG_BITMASK_RTC_DAY) + 1;
	tm_new.tm_mon =  REG2BCD(rtc_data[5], PM310_REG_BITMASK_RTC_MONTH);
	tm_new.tm_year = REG2BCD(rtc_data[6], PM310_REG_BITMASK_RTC_YEAR) + (PM310_RTC_YEAR_BASE - 1900);

	LOG_D("%s:%d-%d-%d %d:%d:%d\n", __func__,
		 tm_new.tm_year, tm_new.tm_mon, tm_new.tm_mday,
		 tm_new.tm_hour, tm_new.tm_min, tm_new.tm_sec);
    *timestamp = timegm(&tm_new);
    return RT_EOK;
}

static rt_err_t pm310_rtc_set_secs(time_t *timestamp)
{
    struct tm tm_new;

	rt_uint8_t save_control, rtc_data[ALL_TIME_REGS] = {0};
	int i = 0;
    LOG_D("%s", __func__);
    gmtime_r(timestamp, &tm_new);
	if (PM310_RTC_YEAR_CHECK(tm_new.tm_year))
		return -RT_ERROR;

	/* Write hour, minute and second. */
	rtc_data[0] = BCD2REG(tm_new.tm_sec, PM310_REG_BITMASK_RTC_SEC);
	rtc_data[1] = BCD2REG(tm_new.tm_min, PM310_REG_BITMASK_RTC_MIN);
	rtc_data[2] = BCD2REG(tm_new.tm_hour, PM310_REG_BITMASK_RTC_HOUR);
	rtc_data[3] = BCD2REG(tm_new.tm_mday, PM310_REG_BITMASK_RTC_DAY) - 1;
	rtc_data[5] = BCD2REG(tm_new.tm_mon, PM310_REG_BITMASK_RTC_MONTH);
	rtc_data[6] = BCD2REG(tm_new.tm_year, PM310_REG_BITMASK_RTC_YEAR) - (PM310_RTC_YEAR_BASE - 1900);

	/* Stop RTC while updating the RTC registers */
	pm310_read_reg(PM310_REG_RTC_CTRL, &save_control);
	save_control = save_control & (~(0x01));
	pm310_write_reg(PM310_REG_RTC_CTRL, save_control);
	/* Write second.minute hour day*/
	for (i = 0; i < 4; i++)
		pm310_write_reg(PM310_REG_RTC_SEC + i, rtc_data[i]);
	/* Write month and year. */
	pm310_write_reg(PM310_REG_RTC_MONTH, rtc_data[5]);
	pm310_write_reg(PM310_REG_RTC_YEAR, rtc_data[6]);

	/* Write data to RTC */
	pm310_write_reg(PM310_REG_RTC_TIME_UPDATE, 0x1a);

	/* Enable RTC */
	pm310_write_reg(PM310_REG_RTC_CTRL, 1);

    return RT_EOK;
}

static rt_err_t pm310_rtc_get_timeval(struct timeval *tv)
{
    time_t timestamp;
    LOG_D("%s", __func__);
    if(pm310_rtc_get_secs(&timestamp) == RT_EOK)
    {
        tv->tv_sec = timestamp;
        return RT_EOK;    
    }
    tv->tv_sec = 0;
    return RT_ERROR;
}

static rt_err_t pm310_rtc_init(void)
{
    LOG_D("%s", __func__);
    return RT_EOK;
}

const static struct rt_rtc_ops rtc_ops =
    {
        .init = pm310_rtc_init,
        .get_secs = pm310_rtc_get_secs,
        .set_secs = pm310_rtc_set_secs,
        .get_alarm = RT_NULL,
        .set_alarm = RT_NULL,
        .get_timeval = pm310_rtc_get_timeval,
        .set_timeval = RT_NULL};

int rt_hw_rtc_init(void)
{
    rt_err_t result;
    rt_memset(&rtc, 0, sizeof(rtc));
    rtc.ops = &rtc_ops;

    result = rt_hw_rtc_register(&rtc, "rtc", RT_DEVICE_FLAG_RDWR, RT_NULL);
    if (result != RT_EOK)
    {
        LOG_E("rtc register err code: %d", result);
        return result;
    }
    LOG_I("rtc init success");
    return RT_EOK;
}
INIT_DEVICE_EXPORT(rt_hw_rtc_init);
#endif /* BSP_USING_RTC */

