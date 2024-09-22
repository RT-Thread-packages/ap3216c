/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-05-31     yangjie      the first version
 */

#ifndef LSC_AP3216C_SENSOR_V2_H__
#define LSC_AP3216C_SENSOR_V2_H__

#include "ap3216c.h"

#include <rtthread.h>
#include <rtdevice.h>

#define AP3216C_I2C_ADDR 0x1e

#if defined(RT_VERSION_CHECK)
    #if (RTTHREAD_VERSION >= RT_VERSION_CHECK(5, 0, 2))
        #define RT_SIZE_TYPE   rt_ssize_t
    #else
        #define RT_SIZE_TYPE   rt_size_t
    #endif
#endif

int rt_hw_ap3216c_init(const char *name, struct rt_sensor_config *cfg);

#endif
