/*
 * Copyright (c) 2024 Silicon Laboratories Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT silabs_siwx917_counter

#include <errno.h>
#include <soc.h>
#include <stddef.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "rsi_rom_clks.h"
#include "sl_si91x_ulp_timer.h"
#include "rsi_timers.h"

LOG_MODULE_REGISTER(counter_siwg917, CONFIG_COUNTER_LOG_LEVEL);

#define ULP_TIMER_MAX_COUNTER_VALUE 0xFFFFFFFF
#define COUNTER_CHANNEL_COUNT       3 /* Define the number of channels available for the counter */
#define TIMER0_INT                  0x01 /* Define a macro for the interrupt bitmask for Timer 0 */

struct counter_siwg917_config {
	struct counter_config_info
		counter_info;     /* Basic configuration information for the counter */
	TIMERS_Type *reg;         /* Pointer to the ULP_TIMER register base address */
	uint8_t channel_count;    /* Number of timer modules available for generating alarms */
	void (*irq_config)(void); /* Function pointer for configuring interrupts */
};

struct counter_siwg917_data {
	counter_alarm_callback_t callback; /* Function pointer for the alarm callback */
	void *user_data; /* Pointer to user data that will be passed to the callback */
};

/* Map a channel ID to the corresponding ULP timer module index */
static uint8_t chan_id2_timermodule_idx(uint8_t chan_id)
{
	uint8_t timer_module_idx;

	switch (chan_id) {
	case 0:
		timer_module_idx = 1;
		break;
	case 1:
		timer_module_idx = 2;
		break;
	default:
		timer_module_idx = 3;
		break;
	}
	return timer_module_idx;
}

static int counter_siwg917_start(const struct device *dev)
{
	const struct counter_siwg917_config *cfg = dev->config;

	/* Set the maximum count value for the timer0 */
	if (sl_si91x_ulp_timer_set_count(TIMER_0, cfg->counter_info.max_top_value)) {
		return -EINVAL;
	}
	/* Start timer 0 */
	if (sl_si91x_ulp_timer_start(TIMER_0)) {
		return -EINVAL;
	}
	return 0;
}

static int counter_siwg917_stop(const struct device *dev)
{
	ARG_UNUSED(dev);

	/* Stop timer 0 */
	if (sl_si91x_ulp_timer_stop(TIMER_0)) {
		return -EINVAL;
	}
	return 0;
}

static int counter_siwg917_get_value(const struct device *dev, uint32_t *ticks)
{
	ARG_UNUSED(dev);

	/* Retrieve the current count of timer0 */
	if (sl_si91x_ulp_timer_get_count(TIMER_0, ticks)) {
		return -EINVAL;
	}
	return 0;
}

static int counter_siwg917_set_top_value(const struct device *dev,
					 const struct counter_top_cfg *cfg)
{
	const struct counter_siwg917_config *dev_cfg = dev->config;

	/* Check if the provided top value matches the maximum supported top value */
	if (cfg->ticks != dev_cfg->counter_info.max_top_value) {
		return -ENOTSUP;
	}
	return 0;
}

static uint32_t counter_siwg917_get_pending_int(const struct device *dev)
{
	ARG_UNUSED(dev);

	return 0;
}

static uint32_t counter_siwg917_get_top_value(const struct device *dev)
{
	const struct counter_siwg917_config *cfg = dev->config;

	/* Return the maximum top value defined in the device configuration */
	return cfg->counter_info.max_top_value;
}

static int counter_siwg917_set_alarm(const struct device *dev, uint8_t chan_id,
				     const struct counter_alarm_cfg *alarm_cfg)
{
	/* Get the current value of the counter */
	uint32_t count = sl_si91x_ulp_timer_get_count(TIMER_0, &count);
	const struct counter_siwg917_config *cfg = dev->config;
	uint8_t channel = chan_id2_timermodule_idx(chan_id);
	struct counter_siwg917_data *dev_data = dev->data;
	uint32_t ticks_to_program;

	/* Check if the channel is already in use */
	if (dev_data[chan_id].callback != NULL) {
		return -EBUSY;
	}

	if (chan_id >= cfg->channel_count) {
		return -EINVAL;
	}

	/* If the alarm is configured for an absolute time */
	if ((alarm_cfg->flags & COUNTER_ALARM_CFG_ABSOLUTE) != 0) {
		if (alarm_cfg->ticks > count) {
			/* The specified timeout occurs prior to reaching the overflow condition */
			ticks_to_program = alarm_cfg->ticks - count;
		} else {
			/* The specified timeout occurs after reaching the overflow condition */
			ticks_to_program =
				(cfg->counter_info.max_top_value - count) + alarm_cfg->ticks;
		}
	} else {
		/* If the alarm is relative, simply set the ticks to the value specified */
		ticks_to_program = alarm_cfg->ticks;
	}

	/* Clear any pending interrupt for the specified channel */
	if (RSI_TIMERS_InterruptClear(cfg->reg, channel)) {
		return -EINVAL;
	}
	/* Store the callback and user data for the channel */
	dev_data[chan_id].callback = alarm_cfg->callback;
	dev_data[chan_id].user_data = alarm_cfg->user_data;

	/* Set the count for the timer to the calculated ticks value */
	if (sl_si91x_ulp_timer_set_count(channel, ticks_to_program)) {
		return -EINVAL;
	}
	if (RSI_TIMERS_InterruptEnable(cfg->reg, channel)) {
		return -EINVAL;
	}

	if (sl_si91x_ulp_timer_start(channel)) {
		return -EINVAL;
	}

	return 0;
}

static int counter_siwg917_cancel_alarm(const struct device *dev, uint8_t chan_id)
{
	const struct counter_siwg917_config *cfg = dev->config;
	uint8_t channel = chan_id2_timermodule_idx(chan_id);
	struct counter_siwg917_data *dev_data = dev->data;

	if (chan_id >= cfg->channel_count) {
		return -EINVAL;
	}

	/* Clear any pending interrupt for the specified channel */
	if (RSI_TIMERS_InterruptClear(cfg->reg, channel)) {
		return -EINVAL;
	}
	if (sl_si91x_ulp_timer_stop(channel)) {
		return -EINVAL;
	}

	/* Reset the callback and user data associated with the channel */
	dev_data[chan_id].callback = NULL;
	dev_data[chan_id].user_data = NULL;

	return 0;
}

static int counter_siwg917_init(const struct device *dev)
{
	const struct counter_siwg917_config *cfg = dev->config;
	/* Define and initialize the configuration for the counter (TIMER_0) */
	ulp_timer_config_t timer_handle_timer = {
		.timer_num = TIMER_0,
		.timer_mode = ULP_TIMER_MODE_PERIODIC,
		.timer_type = ULP_TIMER_TYP_DEFAULT,
		.timer_match_value = cfg->counter_info.max_top_value,
		.timer_direction = UP_COUNTER,
	};

	if (sl_si91x_ulp_timer_set_configuration(&timer_handle_timer)) {
		return -EINVAL;
	}

	/* Configure the remaining timer modules, which act as channels for generating timeouts */
	/* relative to the main counter (TIMER_0), allowing them to trigger alarms or events */
	timer_handle_timer.timer_match_value = 0;
	timer_handle_timer.timer_mode = ULP_TIMER_MODE_ONESHOT;

	for (int index = 0; index < COUNTER_CHANNEL_COUNT; index++) {
		timer_handle_timer.timer_num = index + 1;
		if (sl_si91x_ulp_timer_set_configuration(&timer_handle_timer)) {
			return -EINVAL;
		}
	}
	cfg->irq_config();
	return 0;
}

static void counter_siwg917_isr(const struct device *dev)
{
	const struct counter_siwg917_config *cfg = dev->config;
	uint8_t int_status = cfg->reg->MCUULP_TMR_INTR_STAT;
	struct counter_siwg917_data *dev_data = dev->data;
	counter_alarm_callback_t alarm_callback;
	uint32_t count = 0;

	if (sl_si91x_ulp_timer_get_count(TIMER_0, &count)) {
		return;
	}

	if (int_status & TIMER0_INT) {
		if (RSI_TIMERS_InterruptClear(cfg->reg, TIMER_0)) {
			return;
		}
	} else {
		for (int index = 0; index < COUNTER_CHANNEL_COUNT; index++) {
			/* Check if this alarm timer triggered an interrupt */
			if (int_status & BIT(index + 1)) {
				/* If the callback is set for this alarm, invoke it */
				if (dev_data[index].callback) {
					alarm_callback = dev_data[index].callback;
					dev_data[index].callback = NULL;
					/* Call the user-defined callback with the current count and
					 * user data */
					alarm_callback(dev, index, count,
						       dev_data[index].user_data);
				}
				RSI_TIMERS_InterruptClear(cfg->reg, (index + 1));
			}
		}
	}
}

static const struct counter_driver_api counter_siwg917_driver_api = {
	.start = counter_siwg917_start,
	.stop = counter_siwg917_stop,
	.get_value = counter_siwg917_get_value,
	.set_alarm = counter_siwg917_set_alarm,
	.cancel_alarm = counter_siwg917_cancel_alarm,
	.set_top_value = counter_siwg917_set_top_value,
	.get_pending_int = counter_siwg917_get_pending_int,
	.get_top_value = counter_siwg917_get_top_value,
};

#define SIWG917_COUNTER_IRQ_CONNECT(n, inst)                                                       \
	IRQ_CONNECT(DT_INST_IRQ_BY_IDX(inst, n, irq), DT_INST_IRQ_BY_IDX(inst, n, priority),       \
		    counter_siwg917_isr, DEVICE_DT_INST_GET(inst), 0);                             \
	irq_enable(DT_INST_IRQ_BY_IDX(inst, n, irq));

#define CONFIGURE_ALL_IRQS(inst, n) LISTIFY(n, SIWG917_COUNTER_IRQ_CONNECT, (), inst)

static void siwg917_counter_irq_configure(void)
{
	CONFIGURE_ALL_IRQS(0, DT_NUM_IRQS(DT_DRV_INST(0)));
}

static const struct counter_siwg917_config counter_siwg917_dev_config = {
	.counter_info =
		{
			.max_top_value = ULP_TIMER_MAX_COUNTER_VALUE,
			.freq = DT_INST_PROP(0, frequency),
			.flags = COUNTER_CONFIG_INFO_COUNT_UP,
			.channels = COUNTER_CHANNEL_COUNT,
		},
	.reg = (TIMERS_Type *)DT_INST_REG_ADDR(0),
	.channel_count = COUNTER_CHANNEL_COUNT,
	.irq_config = siwg917_counter_irq_configure,
};

static struct counter_siwg917_data counter_siwg917_alarm_data[COUNTER_CHANNEL_COUNT];

DEVICE_DT_INST_DEFINE(0, counter_siwg917_init, NULL, &counter_siwg917_alarm_data,
		      &counter_siwg917_dev_config, PRE_KERNEL_2, CONFIG_COUNTER_INIT_PRIORITY,
		      &counter_siwg917_driver_api);
