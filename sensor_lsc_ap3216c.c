/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-05-31     yangjie      the first version
 */

#include "sensor_lsc_ap3216c.h"

#define DBG_TAG "sensor.lsc.ap3216c"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

static RT_SIZE_TYPE _ap3216c_polling_get_data(rt_sensor_t sensor, struct rt_sensor_data *data)
{
    struct ap3216c_device *als_ps_dev = sensor->parent.user_data;

    if (sensor->info.type == RT_SENSOR_TYPE_LIGHT)
    {
        data->data.light = ap3216c_read_ambient_light(als_ps_dev);
        data->timestamp = rt_sensor_get_ts();
        return 1;
    }
    else if (sensor->info.type == RT_SENSOR_TYPE_PROXIMITY)
    {
        data->data.proximity = (rt_sensor_float_t)ap3216c_read_ps_data(als_ps_dev);
        data->timestamp = rt_sensor_get_ts();
        return 1;
    }
    else
    {
        return -RT_EINVAL;
    }
}

static RT_SIZE_TYPE ap3216c_fetch_data(rt_sensor_t sensor, rt_sensor_data_t buf, rt_size_t len)
{
    RT_ASSERT(buf);

    if (RT_SENSOR_MODE_GET_FETCH(sensor->info.mode) == RT_SENSOR_MODE_FETCH_POLLING)
    {
        return _ap3216c_polling_get_data(sensor, buf);
    }
    else
    {
        return -RT_EINVAL;
    }
}

static rt_err_t _ap3216c_set_accuracy(rt_sensor_t sensor, rt_uint8_t accuracy)
{
    rt_err_t result = -RT_EINVAL;
    rt_uint8_t ap3216_range;

    struct ap3216c_device *als_ps_dev = sensor->parent.user_data;

    if (sensor->info.type == RT_SENSOR_TYPE_LIGHT)
    {
        switch(accuracy)
        {
            case RT_SENSOR_MODE_ACCURACY_HIGHEST:
                ap3216_range = AP3216C_ALS_RANGE_323;
                sensor->info.accuracy.resolution = 0.0049; //Resolution = 0.0049 lux/count
                sensor->info.accuracy.error = 0;
                sensor->info.scale.range_min = 0;
                sensor->info.scale.range_max = 323;
                break;
            case RT_SENSOR_MODE_ACCURACY_HIGH:
                ap3216_range = AP3216C_ALS_RANGE_1291;
                sensor->info.accuracy.resolution = 0.0197; //Resolution = 0.0197 lux/count.
                sensor->info.accuracy.error = 0;
                sensor->info.scale.range_min = 0;
                sensor->info.scale.range_max = 1291;
                break;
            case RT_SENSOR_MODE_ACCURACY_MEDIUM:
                ap3216_range = AP3216C_ALS_RANGE_5162;
                sensor->info.accuracy.resolution = 0.0788; //Resolution = 0.0788 lux/count.
                sensor->info.accuracy.error = 0;
                sensor->info.scale.range_min = 0;
                sensor->info.scale.range_max = 5162;
                break;
            case RT_SENSOR_MODE_ACCURACY_LOWEST:
                ap3216_range = AP3216C_ALS_RANGE_20661;
                sensor->info.accuracy.resolution = 0.35; //Resolution = 0.35 lux/count.
                sensor->info.accuracy.error = 0;
                sensor->info.scale.range_min = 0;
                sensor->info.scale.range_max = 20661;
                break;
            default:
                return -RT_EINVAL;
        }
        result = ap3216c_set_param(als_ps_dev, AP3216C_ALS_RANGE, ap3216_range);
    }
    else if (sensor->info.type == RT_SENSOR_TYPE_PROXIMITY)
    {
        switch(accuracy)
        {
            default:
                return -RT_EINVAL;
        }
        result = ap3216c_set_param(als_ps_dev, AP3216C_PS_GAIN, ap3216_range);
    }

    return result;
}

static rt_err_t _ap3216c_set_power(rt_sensor_t sensor, rt_uint8_t power)
{
    rt_err_t result = -RT_EINVAL;
    struct ap3216c_device *als_ps_dev = sensor->parent.user_data;
    rt_uint8_t power_mode;

    result = ap3216c_get_param(als_ps_dev, AP3216C_SYSTEM_MODE, &power_mode);
    if(result != RT_EOK)
    {
        LOG_E("Fail to get the ap3216c parameters!");
        return result;
    }

    if (sensor->info.type == RT_SENSOR_TYPE_LIGHT)
    {
        switch(power)
        {
            case RT_SENSOR_MODE_POWER_HIGHEST:
                power_mode |= 0x01; /* enable ALS function */
                sensor->info.acquire_min = 100;
                break;
            case RT_SENSOR_MODE_POWER_DOWN:
                power_mode &= 0xFE; /* disable ALS function */
                break;
            default:
                return -RT_EINVAL;
        }
        result = ap3216c_set_param(als_ps_dev, AP3216C_SYSTEM_MODE, power_mode);
    }
    else if (sensor->info.type == RT_SENSOR_TYPE_PROXIMITY)
    {
        switch(power)
        {
            case RT_SENSOR_MODE_POWER_HIGHEST:
                power_mode |= 0x02; /* enable PS+IR function */
                sensor->info.acquire_min = 13; /* 12.5ms */
                break;
            case RT_SENSOR_MODE_POWER_DOWN:
                power_mode &= 0xFD; /* disable PS+IR function */
                break;
            default:
                return -RT_EINVAL;
        }
        result = ap3216c_set_param(als_ps_dev, AP3216C_SYSTEM_MODE, power_mode);
    }
    return result;
}

static rt_err_t _ap3216c_reset(rt_sensor_t sensor)
{
    struct ap3216c_device *als_ps_dev = sensor->parent.user_data;
    return ap3216c_reset_sensor(als_ps_dev);
}

static rt_err_t ap3216c_control(rt_sensor_t sensor, int cmd, void *args)
{
    rt_err_t result = -RT_EINVAL;

    switch (cmd)
    {
        case RT_SENSOR_CTRL_SET_ACCURACY_MODE:
            result = _ap3216c_set_accuracy(sensor, (rt_uint32_t)args);
            break;
        case RT_SENSOR_CTRL_SET_POWER_MODE:
            result = _ap3216c_set_power(sensor, (rt_uint32_t)args);
            break;
        case RT_SENSOR_CTRL_SOFT_RESET:
            result = _ap3216c_reset(sensor);
            break;
        default:
            return -RT_EINVAL;
    }

    return result;
}

static struct rt_sensor_ops sensor_ops =
{
    ap3216c_fetch_data,
    ap3216c_control
};

static const char * sensor_name = "ap3216c";

int rt_hw_ap3216c_init(const char *name, struct rt_sensor_config *cfg)
{
    rt_int8_t result;
    rt_sensor_t sensor_als = RT_NULL, sensor_ps = RT_NULL;
    struct ap3216c_device *als_ps_dev;
    
    als_ps_dev = ap3216c_init(cfg->intf.dev_name);
    if (als_ps_dev == RT_NULL)
    {
        return -RT_ERROR;
    }

    /* temperature sensor register */
    sensor_als = rt_calloc(1, sizeof(struct rt_sensor_device));
    if (sensor_als == RT_NULL)
        return -1;

    sensor_als->info.type       = RT_SENSOR_TYPE_LIGHT;
    sensor_als->info.vendor     = RT_SENSOR_VENDOR_LSC;
    sensor_als->info.name       = sensor_name;
    sensor_als->info.unit       = RT_SENSOR_UNIT_LUX;
    sensor_als->info.intf_type  = RT_SENSOR_INTF_I2C;

    rt_memcpy(&sensor_als->config, cfg, sizeof(struct rt_sensor_config));
    sensor_als->ops = &sensor_ops;

    result = rt_hw_sensor_register(sensor_als, name, RT_DEVICE_FLAG_RDWR, als_ps_dev);    
    if (result != RT_EOK)
    {
        LOG_E("device register err code: %d", result);
        goto __exit;
    }

    /* humidity sensor register */
    sensor_ps = rt_calloc(1, sizeof(struct rt_sensor_device));
    if (sensor_ps == RT_NULL)
        return -1;

    sensor_ps->info.type       = RT_SENSOR_TYPE_PROXIMITY;
    sensor_ps->info.vendor     = RT_SENSOR_VENDOR_LSC;
    sensor_ps->info.name       = sensor_name;
    sensor_ps->info.unit       = RT_SENSOR_UNIT_CM;
    sensor_ps->info.intf_type  = RT_SENSOR_INTF_I2C;

    rt_memcpy(&sensor_ps->config, cfg, sizeof(struct rt_sensor_config));
    sensor_ps->ops = &sensor_ops;

    result = rt_hw_sensor_register(sensor_ps, name, RT_DEVICE_FLAG_RDWR, als_ps_dev);
    if (result != RT_EOK)
    {
        LOG_E("device register err code: %d", result);
        goto __exit;
    }

    return RT_EOK;

__exit:
    if (sensor_als)
        rt_free(sensor_als);
    if (sensor_ps)
        rt_free(sensor_ps);
    if (als_ps_dev)
        ap3216c_deinit(als_ps_dev);
    return -RT_ERROR;
}
