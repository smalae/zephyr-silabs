/*
 * Copyright (c) 2024 Silicon Laboratories Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <zephyr/irq.h>
#include <zephyr/sys/util.h>
#include <zephyr/device.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/logging/log.h>
#include <zephyr/types.h>
#include "sl_si91x_watchdog_timer.h"
#include "sl_status.h"

#define DT_DRV_COMPAT          silabs_siwx917_watchdog
#define WDT_SIWG917_MAX_RESET_SELECT_VALUE	31
#define WDT_SIWG917_MAX_WINDOW_SELECT_VALUE	15

LOG_MODULE_REGISTER(si91x_watchdog, CONFIG_WDT_LOG_LEVEL);

struct wdt_siwg917_config {           
	watchdog_timer_clock_config_t wdt_clock_cfg;
	void (*irq_config)(void);
};

struct wdt_siwg917_data {
	wdt_callback_t callback;
	watchdog_timer_config_t wdt_config;
	bool timeout_installed;
};

static uint32_t get_timeout_from_regvalue(uint8_t reg_value)
{
	uint32_t ticks = BIT(reg_value);
	float timeout = ((float)ticks / 32768);
	timeout *= 1000;

	return ((uint32_t)timeout);//msec
}

static int get_regvalue_from_timeout(uint32_t timeout)
{
	int index;
	
	for(index = 0; index < WDT_SIWG917_MAX_RESET_SELECT_VALUE; index++) {
		if(get_timeout_from_regvalue(index) >= timeout) {
			break;
		}
	}
	return index;
}

static int wdt_siwg917_install_timeout(const struct device *dev,
				     const struct wdt_timeout_cfg *cfg)
{
	struct wdt_siwg917_data *data = dev->data;
	
	if (data->timeout_installed) {
		LOG_ERR("No more timeouts can be installed");
		return -EINVAL;
	}
	if (cfg->window.max > get_timeout_from_regvalue(WDT_SIWG917_MAX_RESET_SELECT_VALUE) || cfg->window.max == 0) {
		LOG_ERR("Upper limit reset timeout out of range");
		return -EINVAL;
	}
	if (cfg->window.min > 0) {
		if (cfg->window.min > get_timeout_from_regvalue(WDT_SIWG917_MAX_WINDOW_SELECT_VALUE)) {
		LOG_ERR("Upper limit window timeout out of range");
		return -EINVAL;
		}
		data->wdt_config.window_time = get_regvalue_from_timeout(cfg->window.min);	
	} else {
		data->wdt_config.window_time = 0;
	}
	switch (cfg->flags) {
	case WDT_FLAG_RESET_SOC:
	case WDT_FLAG_RESET_CPU_CORE:
		if (cfg->callback != NULL) {
			LOG_ERR("Reset mode with callback not supported\n");
			return -ENOTSUP;
		}
		data->wdt_config.system_reset_time = get_regvalue_from_timeout(cfg->window.max);
		data->wdt_config.interrupt_time = 0;
		RSI_WWDT_IntrMask();
		break;
	case WDT_FLAG_RESET_NONE:
		data->wdt_config.system_reset_time = WDT_SIWG917_MAX_RESET_SELECT_VALUE;
		data->wdt_config.interrupt_time = get_regvalue_from_timeout(cfg->window.max);
		if (cfg->callback != NULL) {
			data->callback = cfg->callback;
		}
		break;
	default:
		LOG_ERR("Unsupported watchdog config flag");
		return -EINVAL;
	}
	data->timeout_installed = true;
	return 0;
}

static int wdt_siwg917_setup(const struct device *dev, uint8_t options)
{
	struct wdt_siwg917_data *data = dev->data;
	
	if (!data->timeout_installed) {
		LOG_ERR("No valid timeouts installed");
		return -EINVAL;
	}
	sl_si91x_watchdog_set_configuration(&data->wdt_config);
	if(data->wdt_config.window_time) {
		sl_si91x_watchdog_set_window_time(data->wdt_config.window_time);
	}
	sl_si91x_watchdog_start_timer();
	return 0;
}

static int wdt_siwg917_disable(const struct device *dev)
{
	struct wdt_siwg917_data *data = dev->data;
	
	sl_si91x_watchdog_stop_timer();
	data->timeout_installed = false;
	return 0;	
}

static int wdt_siwg917_feed(const struct device *dev, int channel_id)
{
	if (channel_id != 0) {
		LOG_ERR("Invalid channel id");
		return -EINVAL;
	}
	sl_si91x_watchdog_restart_timer();
	return 0;
}

static void wdt_siwg917_isr(const struct device *dev)
{
	struct wdt_siwg917_data *data = dev->data;
	
	RSI_WWDT_IntrClear();
	if(data->wdt_config.interrupt_time) {
		sl_si91x_watchdog_stop_timer();
		data->wdt_config.interrupt_time = 0;
	}
	if (data->callback != NULL) {
		data->callback(dev, 0);
	}
}

static int wdt_siwg917_init(const struct device *dev)
{
	const struct wdt_siwg917_config *config = dev->config;
	
	sl_si91x_watchdog_init_timer();
	sl_si91x_watchdog_configure_clock((watchdog_timer_clock_config_t *)&config->wdt_clock_cfg);
	config->irq_config();
	
	return 0;
}

static DEVICE_API(wdt, wdt_siwg917_driver_api) = {
	.setup = wdt_siwg917_setup,
	.disable = wdt_siwg917_disable,
	.install_timeout = wdt_siwg917_install_timeout,
	.feed = wdt_siwg917_feed,
};

static void siwg917_wdt_irq_configure(void)
{
	IRQ_DIRECT_CONNECT(DT_INST_IRQN(0),
			   DT_INST_IRQ(0, priority),
			   wdt_siwg917_isr, 0);
	irq_enable(DT_INST_IRQN(0));
}

static const struct wdt_siwg917_config wdt_siwg917_dev_config = {
	.wdt_clock_cfg = {DT_INST_PROP(0, silabs_fsm_clock), DT_INST_PROP(0, silabs_bg_pmu_clock)},
	.irq_config = siwg917_wdt_irq_configure,
};

static struct wdt_siwg917_data wdt_siwg917_data;

DEVICE_DT_INST_DEFINE(0, wdt_siwg917_init, NULL, &wdt_siwg917_data,
		      &wdt_siwg917_dev_config, PRE_KERNEL_1, CONFIG_WDT_SIWX917_COMMON_INIT_PRIORITY,
		      &wdt_siwg917_driver_api);
