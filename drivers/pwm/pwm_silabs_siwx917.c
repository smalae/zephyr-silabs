/*
 * Copyright (c) 2025 Silicon Laboratories Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <zephyr/irq.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/util_macro.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/logging/log.h>
#include <zephyr/types.h>
#include "sl_si91x_pwm.h"

#define DT_DRV_COMPAT silabs_siwx917_pwm

#define PWM_CHANNELS      4

LOG_MODULE_REGISTER(si91x_pwm, CONFIG_PWM_LOG_LEVEL);

struct pwm_siwx917_config {
	const struct device *clock_dev;
	clock_control_subsys_t clock_subsys;
	const struct pinctrl_dev_config *pcfg;
	uint32_t frequency;
};

struct pwm_siwx917_data {
	sl_pwm_config_t pwm_channel_config[PWM_CHANNELS];
};

uint32_t convert_period_pulse_cycles(uint32_t cycles) {
	
	return 0;
}

static int pwm_siwx917_set_cycles(const struct device *dev, uint32_t channel,
				  uint32_t period_cycles, uint32_t pulse_cycles, pwm_flags_t flags)
{
	struct pwm_siwx917_data *data = dev->data;
	uint32_t prev_period;
	uint32_t current_period = convert_period_pulse_cycles(period_cycles);
	uint32_t current_duty_cycle = convert_period_pulse_cycles(pulse_cycles);
	
	if(channel > PWM_CHANNELS || channel == 0) {
		return -EINVAL;
	}
	
	if(sl_si91x_pwm_get_time_period(channel, (uint16_t *)&prev_period)) {
		return -ENOTSUP;
	}
	
	if(prev_period == 0 && current_period != 0) {
		if(sl_si91x_pwm_set_output_polarity(data->pwm_channel_config[channel - 1].is_polarity_low,
						    data->pwm_channel_config[channel - 1].is_polarity_high)) {
			return -EINVAL;				    
		}
		if(sl_si91x_pwm_set_time_period(channel, current_period, 0)) {
			return -EINVAL;
		}
		if(sl_si91x_pwm_set_output_mode(data->pwm_channel_config[channel - 1].is_mode, channel)) {
			return -EINVAL;
		}
		if(sl_si91x_pwm_set_duty_cycle(current_duty_cycle, channel)) {
			return -EINVAL;
		}
		if(sl_si91x_pwm_set_base_timer_mode(data->pwm_channel_config[channel - 1].base_timer_mode, channel)) {
			return -EINVAL;
		}
		if(sl_si91x_pwm_control_base_timer(data->pwm_channel_config[channel - 1].channel_timer_selection)) {
			return -EINVAL;
		}
		if(sl_si91x_pwm_start(channel)) {
			return -EINVAL;
		}
		return 0;
	}
	
	if(current_period != prev_period) {
		if(sl_si91x_pwm_set_time_period(channel, current_period, 0)) {
			return -EINVAL;
		}
	}
	
	if(current_duty_cycle != data->pwm_channel_config[channel].duty_cycle) {
		if(sl_si91x_pwm_set_duty_cycle(current_duty_cycle, channel)) {
			return -EINVAL;
		}
		data->pwm_channel_config[channel - 1].duty_cycle = current_duty_cycle;
	}

	return 0;
}

static int pwm_siwx917_get_cycles_per_sec(const struct device *dev, uint32_t channel,
					  uint64_t *cycles)
{
	const struct pwm_siwx917_config *config = dev->config;
	
	if(channel > PWM_CHANNELS) {
		return -EINVAL;
	}
	
	*cycles = (uint64_t)config->frequency;

	return 0;
}

/* Function to initialize PWM peripheral */
static int pwm_siwx917_init(const struct device *dev)
{
	const struct pwm_siwx917_config *config = dev->config;
	struct pwm_siwx917_data *data = dev->data;
	int ret;

	ret = clock_control_on(config->clock_dev, config->clock_subsys);
	if (ret) {
		return ret;
	}

	ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret) {
		return ret;
	}
	
	for(int i = 0;i <= PWM_CHANNELS; i++) {
		/* Initialize the channel structures with default values */
		data->pwm_channel_config[i].channel = i+1;
		data->pwm_channel_config[i].is_polarity_high = 1;
		data->pwm_channel_config[i].is_mode = SL_MODE_INDEPENDENT;
		data->pwm_channel_config[i].base_timer_mode = SL_FREE_RUN_MODE;
		data->pwm_channel_config[i].channel_timer_selection = SL_BASE_TIMER_EACH_CHANNEL;
	}

	return 0;
}

static const struct pwm_driver_api pwm_siwx917_driver_api = {
	.set_cycles = pwm_siwx917_set_cycles,
	.get_cycles_per_sec = pwm_siwx917_get_cycles_per_sec,
};

#define SIWX917_PWM_INIT(inst)                                                                     \
	PINCTRL_DT_INST_DEFINE(inst);								   \
	static struct pwm_siwx917_data pwm_siwx917_data_##inst;                                    \
	static const struct pwm_siwx917_config pwm_config_##inst = {                               \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(inst)),                             \
		.clock_subsys = (clock_control_subsys_t)DT_INST_PHA(inst, clocks, clkid),          \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                                      \
		.frequency = 32000000                                                              \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, &pwm_siwx917_init, NULL, &pwm_siwx917_data_##inst,             \
			      &pwm_config_##inst, PRE_KERNEL_1, CONFIG_PWM_INIT_PRIORITY,         \
			      &pwm_siwx917_driver_api);

DT_INST_FOREACH_STATUS_OKAY(SIWX917_PWM_INIT)
