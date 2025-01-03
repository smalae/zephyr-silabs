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

#define DT_DRV_COMPAT                       silabs_siwx917_watchdog
#define WDT_SIWG917_MAX_RESET_SELECT_VALUE  31
#define WDT_SIWG917_MAX_WINDOW_SELECT_VALUE 15
#define CLOCK_FREQUENCY                     32768
#define TIMEOUT_INSTALL_STATUS              0x01
#define SETUP_STATUS                        0x02

LOG_MODULE_REGISTER(si91x_watchdog, CONFIG_WDT_LOG_LEVEL);

struct wdt_siwg917_config {
	/* Configuration for the watchdog timer clock */
	watchdog_timer_clock_config_t wdt_clock_cfg;
	/* Function pointer for the IRQ (Interrupt Request) configuration */
	void (*irq_config)(void);
};

struct wdt_siwg917_data {
	/* Callback function to be called on watchdog timer events */
	wdt_callback_t callback;
	/* Configuration settings for the watchdog timer */
	watchdog_timer_config_t wdt_config;
	/* Flags indicating if the timeout/setup is done */
	uint8_t wdt_setup_timeout_status;
};

/* Function to get the timeout value in milliseconds from the register value */
static uint32_t get_timeout_from_regvalue(uint8_t reg_value)
{
	uint32_t ticks = BIT(reg_value);
	float timeout = ((float)ticks / CLOCK_FREQUENCY);
	timeout *= 1000;

	/* Return the timeout value as an unsigned 32-bit integer in milliseconds */
	return ((uint32_t)timeout);
}

/* Function to get the register value from the timeout value in milliseconds */
static int get_regvalue_from_timeout(uint32_t timeout)
{
	int index;

	/* Iterate through possible register values */
	for (index = 0; index < WDT_SIWG917_MAX_RESET_SELECT_VALUE; index++) {
		/* Check if the calculated timeout for the current register value */
		/* is greater than or equal to the input timeout value */
		if (get_timeout_from_regvalue(index) >= timeout) {
			break;
		}
	}
	return index;
}

/* Function to install WDT timeouts */
static int wdt_siwg917_install_timeout(const struct device *dev, const struct wdt_timeout_cfg *cfg)
{
	struct wdt_siwg917_data *data = dev->data;

	if (data->wdt_setup_timeout_status & SETUP_STATUS) {
		return -EBUSY;
	}
	if (data->wdt_setup_timeout_status & TIMEOUT_INSTALL_STATUS) {
		LOG_ERR("No more timeouts can be installed");
		return -ENOMEM;
	}
	if (cfg->window.max > get_timeout_from_regvalue(WDT_SIWG917_MAX_RESET_SELECT_VALUE) ||
	    cfg->window.max == 0) {
		LOG_ERR("Upper limit reset timeout out of range");
		return -EINVAL;
	}
	if (cfg->window.min > 0) {
		if (cfg->window.min >
			    get_timeout_from_regvalue(WDT_SIWG917_MAX_WINDOW_SELECT_VALUE) ||
		    cfg->window.min < 2) {
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
		/* During a system or CPU core reset, interrupts are not needed. Thus, we set */
		/* the interrupt time to 0 to ensure no interrupts occur while resetting. */
		data->wdt_config.interrupt_time = 0;
		/* Mask the WWDT interrupt */
		RSI_WWDT_IntrMask();
		break;
	case WDT_FLAG_RESET_NONE:
		/* Set the reset time to maximum value */
		data->wdt_config.system_reset_time = WDT_SIWG917_MAX_RESET_SELECT_VALUE;
		data->wdt_config.interrupt_time = get_regvalue_from_timeout(cfg->window.max);
		if (cfg->callback != NULL) {
			data->callback = cfg->callback;
		}
		break;
	default:
		LOG_ERR("Unsupported watchdog config flag");
		return -ENOTSUP;
	}
	data->wdt_setup_timeout_status |= TIMEOUT_INSTALL_STATUS;
	return 0;
}

/* Function to setup and start WDT */
static int wdt_siwg917_setup(const struct device *dev, uint8_t options)
{
	struct wdt_siwg917_data *data = dev->data;

	if (data->wdt_setup_timeout_status & SETUP_STATUS) {
		return -EBUSY;
	}
	if (!(data->wdt_setup_timeout_status & TIMEOUT_INSTALL_STATUS)) {
		return -ENOTSUP;
	}
	if (options & (WDT_OPT_PAUSE_IN_SLEEP)) {
		return -ENOTSUP;
	}
	if (sl_si91x_watchdog_set_configuration(&data->wdt_config) != SL_STATUS_OK) {
		return -EINVAL;
	}
	if (data->wdt_config.window_time) {
		if (sl_si91x_watchdog_set_window_time(data->wdt_config.window_time) !=
		    SL_STATUS_OK) {
			return -EINVAL;
		}
	}
	sl_si91x_watchdog_start_timer();
	data->wdt_setup_timeout_status |= SETUP_STATUS;
	return 0;
}

/* Function to disable WDT */
static int wdt_siwg917_disable(const struct device *dev)
{
	struct wdt_siwg917_data *data = dev->data;

	if (!(data->wdt_setup_timeout_status & TIMEOUT_INSTALL_STATUS)) {
		return -EFAULT;
	}
	sl_si91x_watchdog_stop_timer();
	/* Clear the timeout install and setup flags */
	data->wdt_setup_timeout_status &= ~TIMEOUT_INSTALL_STATUS;
	data->wdt_setup_timeout_status &= ~SETUP_STATUS;
	return 0;
}

/* Function to feed (reset) the WDT */
static int wdt_siwg917_feed(const struct device *dev, int channel_id)
{
	struct wdt_siwg917_data *data = dev->data;

	if (!(data->wdt_setup_timeout_status & SETUP_STATUS) ||
	    !(data->wdt_setup_timeout_status & TIMEOUT_INSTALL_STATUS)) {
		return -EINVAL;
	}
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
	if (data->wdt_config.interrupt_time) {
		sl_si91x_watchdog_stop_timer();
		data->wdt_config.interrupt_time = 0;
	}
	if (data->callback != NULL) {
		data->callback(dev, 0);
	}
}

/* Function to initialize the WDT */
static int wdt_siwg917_init(const struct device *dev)
{
	const struct wdt_siwg917_config *config = dev->config;

	sl_si91x_watchdog_init_timer();
	if (sl_si91x_watchdog_configure_clock(
		    (watchdog_timer_clock_config_t *)&config->wdt_clock_cfg) != SL_STATUS_OK) {
		return -EINVAL;
	}
	config->irq_config();

	return 0;
}

static DEVICE_API(wdt, wdt_siwg917_driver_api) = {
	.setup = wdt_siwg917_setup,
	.disable = wdt_siwg917_disable,
	.install_timeout = wdt_siwg917_install_timeout,
	.feed = wdt_siwg917_feed,
};

#define SIWX917_WDT_INIT(inst)                                                                     \
	static struct wdt_siwg917_data siwg917_wdt##inst##_data;                                   \
	static void siwx917_wdt##inst##_irq_configure(void)                                        \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQ(inst, irq), DT_INST_IRQ(inst, priority), wdt_siwg917_isr,  \
			    DEVICE_DT_INST_GET(inst), 0);                                          \
		irq_enable(DT_INST_IRQ(inst, irq));                                                \
	}                                                                                          \
	static const struct wdt_siwg917_config siwg917_wdt##inst##_config = {                      \
		.wdt_clock_cfg = {DT_INST_PROP(0, silabs_fsm_clock),                               \
				  DT_INST_PROP(0, silabs_bg_pmu_clock)},                           \
		.irq_config = siwx917_wdt##inst##_irq_configure,                                   \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, &wdt_siwg917_init, NULL, &siwg917_wdt##inst##_data,            \
			      &siwg917_wdt##inst##_config, PRE_KERNEL_1,                           \
			      CONFIG_WDT_SIWX917_COMMON_INIT_PRIORITY, &wdt_siwg917_driver_api);

DT_INST_FOREACH_STATUS_OKAY(SIWX917_WDT_INIT)
