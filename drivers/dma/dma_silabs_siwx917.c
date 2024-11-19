/*
 * Copyright (c) 2024 Silicon Laboratories Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rsi_rom_udma_wrapper.h"
#include "rsi_udma.h"
#include "sl_status.h"

#include <errno.h>
#include <zephyr/irq.h>
#include <zephyr/sys/util.h>

#include <zephyr/device.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/logging/log.h>

#define DT_DRV_COMPAT                       silabs_siwx917_dma
#define DMA_MAX_TRANSFER_COUNT              1024
#define DMA_CH_PRIORITY_HIGH                1
#define DMA_CH_PRIORITY_LOW                 0
#define UDMA0_INSTANCE                      0
#define ULP_UDMA_INSTANCE                   1
#define VALID_BURST_LENGTH                  0
#define UDMA_ADDR_INC_NONE                  0X03
#define PERIPHERAL_REQUEST_DISABLE          0
#define PERIPHERAL_REQUEST_ENABLE           1
#define NEXT_BURST_DISABLE                  0
#define SOURCE_PROTECT_CONTROL_DISABLE      0
#define DESTINATION_PROTECT_CONTROL_DISABLE 0
#define BURST_REQUEST_DISABLE               0

LOG_MODULE_REGISTER(si91x_dma, CONFIG_DMA_LOG_LEVEL);

struct dma_siwx917_config {
	UDMA0_Type *reg;             /* UDMA register base address */
	uint8_t channels;            /* UDMA channel count */
	uint8_t irq_number;          /* IRQ number */
	void (*irq_configure)(void); /* IRQ configure function */
};

struct dma_siwx917_data {
	UDMA_Channel_Info *udma_channel_info;
	dma_callback_t dma_callback; /* User callback */
	void *cb_data;               /* User callback data */
	uint32_t dma_rom_buff[30];   /* Buffers which stores UDMA handle*/
				     /* related information */
};

/* Function to validate and convert channel transfer direction for sl layer usage */
static inline int siwx917_dma_transfer_direction(uint32_t dir)
{
	switch (dir) {
	case MEMORY_TO_MEMORY:
		return PERIPHERAL_REQUEST_DISABLE;
	case MEMORY_TO_PERIPHERAL:
		return PERIPHERAL_REQUEST_ENABLE;
	case PERIPHERAL_TO_MEMORY:
		return PERIPHERAL_REQUEST_ENABLE;
	default:
		return -ENOTSUP;
	}
}

/* Function to validate and convert channel data width for sl layer usage */
static inline int siwx917_dma_data_width(uint32_t data_width)
{
	switch (data_width) {
	case 1:
		return SRC_SIZE_8;
	case 2:
		return SRC_SIZE_16;
	case 4:
		return SRC_SIZE_32;
	default:
		return -EINVAL;
	}
}

/* Function to validate and convert channel burst length for sl layer usage */
static inline int siwx917_dma_burst_length(uint32_t blen)
{
	switch (blen) {
	case 1:
		return VALID_BURST_LENGTH; /* 8-bit burst */
	default:
		return -EINVAL;
	}
}

/* Function to validate and convert channel addr increment value for sl layer usage */
static inline int siwx917_dma_addr_adjustment(uint32_t adjustment)
{
	switch (adjustment) {
	case 0:
		return 0; /* Addr Increment */
	case 2:
		return UDMA_ADDR_INC_NONE; /* No Address increment */
	default:
		return -EINVAL;
	}
}

static int dma_channel_config(const struct device *dev, RSI_UDMA_HANDLE_T udma_handle,
			      uint32_t rsi_channel, struct dma_config *config,
			      UDMA_Channel_Info *channel_info)
{
	const struct dma_siwx917_config *cfg = dev->config;
	UDMA_RESOURCES UDMA_Resources = {
		.reg = cfg->reg,
		.udma_irq_num = cfg->irq_number,
		/* SRAM address where UDMA descriptor is stored */
		.desc = (RSI_UDMA_DESC_T *)cfg->reg->CTRL_BASE_PTR,
	};
	RSI_UDMA_CHA_CONFIG_DATA_T channel_control = {
		.transferType = UDMA_MODE_BASIC,
		.nextBurst = NEXT_BURST_DISABLE,
		.srcProtCtrl = SOURCE_PROTECT_CONTROL_DISABLE,
		.dstProtCtrl = DESTINATION_PROTECT_CONTROL_DISABLE,
	};
	RSI_UDMA_CHA_CFG_T channel_config = {
		.burstReq = BURST_REQUEST_DISABLE,
	};
	int status;

	channel_config.channelPrioHigh = config->channel_priority;
	channel_config.periphReq = siwx917_dma_transfer_direction(config->channel_direction);
	if (channel_config.periphReq < 0) {
		return -EINVAL;
	}
	channel_config.dmaCh = rsi_channel;
	if (channel_config.periphReq) {
		/* Arbitration power for peripheral<->memory transfers */
		channel_control.rPower = ARBSIZE_1;
	} else {
		/* Arbitration power for mem-mem transfers */
		channel_control.rPower = ARBSIZE_1024;
	}
	/* Obtain the number of transfers */
	config->head_block->block_size /= config->source_data_size;
	if (config->head_block->block_size >= DMA_MAX_TRANSFER_COUNT) {
		/* Maximum number of transfers is 1024 */
		channel_control.totalNumOfDMATrans = (DMA_MAX_TRANSFER_COUNT - 1);
	} else {
		channel_control.totalNumOfDMATrans = config->head_block->block_size;
	}
	/* Validate source and data sizes */
	if ((siwx917_dma_data_width(config->source_data_size) < 0) ||
	    (siwx917_dma_data_width(config->dest_data_size) < 0)) {
		return -EINVAL;
	}
	/* Validate burst length */
	if ((siwx917_dma_burst_length(config->source_burst_length >> 3) < 0) ||
	    (siwx917_dma_burst_length(config->dest_burst_length >> 3) < 0)) {
		return -EINVAL;
	}
	channel_control.srcSize = siwx917_dma_data_width(config->source_data_size);
	channel_control.dstSize = siwx917_dma_data_width(config->dest_data_size);
	/* Validate the addr increment value */
	if ((siwx917_dma_addr_adjustment(config->head_block->source_addr_adj) < 0) ||
	    (siwx917_dma_addr_adjustment(config->head_block->dest_addr_adj) < 0)) {
		return -EINVAL;
	}
	/* Update source and destination addr increment values */
	if (siwx917_dma_addr_adjustment(config->head_block->source_addr_adj) == 0) {
		channel_control.srcInc = channel_control.srcSize;
	} else {
		channel_control.srcInc = UDMA_SRC_INC_NONE;
	}
	if (siwx917_dma_addr_adjustment(config->head_block->dest_addr_adj) == 0) {
		channel_control.dstInc = channel_control.dstSize;
	} else {
		channel_control.dstInc = UDMA_DST_INC_NONE;
	}
	/* Configure dma channel for transfer */
	status = (int)UDMAx_ChannelConfigure(&UDMA_Resources, (uint8_t)rsi_channel,
					     (uint32_t)(config->head_block->source_address),
					     (uint32_t)(config->head_block->dest_address),
					     config->head_block->block_size, channel_control,
					     &channel_config, NULL, channel_info, udma_handle);
	return status;
}

/* Function to configure UDMA channel for transfer */
static int dma_siwx917_configure(const struct device *dev, uint32_t channel,
				 struct dma_config *config)
{
	const struct dma_siwx917_config *cfg = dev->config;
	struct dma_siwx917_data *data = dev->data;
	uint32_t rsi_channel = channel - 1;
	int status;
	RSI_UDMA_HANDLE_T udma_handle = (RSI_UDMA_HANDLE_T)data->dma_rom_buff;

	/* Expecting a fixed channel number between 1-32 for udma0 and 1-12 for udma1 */
	if ((channel > cfg->channels) || (channel == 0)) {
		return -EINVAL;
	}

	/* Disable the channel before configuring */
	if (RSI_UDMA_ChannelDisable(udma_handle, rsi_channel) != 0) {
		return -EIO;
	}

	/* Validate the priority */
	if ((config->channel_priority != DMA_CH_PRIORITY_LOW) &&
	    (config->channel_priority != DMA_CH_PRIORITY_HIGH)) {
		return -EINVAL;
	}

	/* Configure dma channel for transfer */
	status = dma_channel_config(dev, udma_handle, rsi_channel, config, data->udma_channel_info);
	if (status != 0) {
		return -ECANCELED;
	}
	return 0;
}

/* Function to reload UDMA channel for new transfer */
static int dma_siwx917_reload(const struct device *dev, uint32_t channel, uint32_t src,
			      uint32_t dst, size_t size)
{
	const struct dma_siwx917_config *cfg = dev->config;
	struct dma_siwx917_data *data = dev->data;
	uint32_t rsi_channel = channel - 1;
	RSI_UDMA_DESC_T *UDMA_Table;
	uint32_t desc_src_addr;
	uint32_t desc_dst_addr;
	RSI_UDMA_HANDLE_T udma_handle = (RSI_UDMA_HANDLE_T)data->dma_rom_buff;

	/* Expecting a fixed channel number between 1-32 for udma0 and 1-12 for udma1 */
	if ((channel > cfg->channels) || (channel == 0)) {
		return -EINVAL;
	}
	/* Fetch the SRAM address where UDMA descriptor is stored */
	UDMA_Table = (RSI_UDMA_DESC_T *)(cfg->reg->CTRL_BASE_PTR);

	/* Disable the channel before reloading transfer */
	if (RSI_UDMA_ChannelDisable(udma_handle, rsi_channel) != 0) {
		return -EIO;
	}

	/* Update new channel info to dev->data structure */
	data->udma_channel_info[rsi_channel].SrcAddr = src;
	data->udma_channel_info[rsi_channel].DestAddr = dst;
	data->udma_channel_info[rsi_channel].Size = size;

	/* Update new transfer size to dev->data structure */
	if (size >= DMA_MAX_TRANSFER_COUNT) {
		data->udma_channel_info[rsi_channel].Cnt = (DMA_MAX_TRANSFER_COUNT - 1);
	} else {
		data->udma_channel_info[rsi_channel].Cnt = size;
	}

	uint32_t length;
	/* Program the DMA descriptors with new transfer data information. */
	if (UDMA_Table[rsi_channel].vsUDMAChaConfigData1.srcInc != UDMA_SRC_INC_NONE) {
		length = (data->udma_channel_info[rsi_channel].Cnt)
			 << UDMA_Table[rsi_channel].vsUDMAChaConfigData1.srcInc;
		desc_src_addr = (uint32_t)((uint32_t)src + length - 1);
		UDMA_Table[rsi_channel].pSrcEndAddr = (void *)((uint32_t)desc_src_addr);
	}
	if (UDMA_Table[rsi_channel].vsUDMAChaConfigData1.dstInc != UDMA_SRC_INC_NONE) {
		length = (data->udma_channel_info[rsi_channel].Cnt)
			 << UDMA_Table[rsi_channel].vsUDMAChaConfigData1.dstInc;
		desc_dst_addr = (uint32_t)((uint32_t)dst + length - 1);
		UDMA_Table[rsi_channel].pDstEndAddr = (void *)((uint32_t)desc_dst_addr);
	}
	UDMA_Table[rsi_channel].vsUDMAChaConfigData1.totalNumOfDMATrans =
		data->udma_channel_info[rsi_channel].Cnt;
	UDMA_Table[rsi_channel].vsUDMAChaConfigData1.transferType = UDMA_MODE_BASIC;

	return 0;
}

/* Function to start a DMA transfer */
static int dma_siwx917_start(const struct device *dev, uint32_t channel)
{
	const struct dma_siwx917_config *cfg = dev->config;
	struct dma_siwx917_data *data = dev->data;
	uint32_t rsi_channel = channel - 1;
	RSI_UDMA_DESC_T *UDMA_Table;
	RSI_UDMA_HANDLE_T udma_handle = (RSI_UDMA_HANDLE_T)data->dma_rom_buff;

	/* Expecting a fixed channel number between 1-32 for udma0 and 1-12 for udma1 */
	if ((channel > cfg->channels) || (channel == 0)) {
		return -EINVAL;
	}
	/* Enable UDMA channel */
	if (RSI_UDMA_ChannelEnable(udma_handle, rsi_channel) != 0) {
		return -ECANCELED;
	}
	/* Fetch the SRAM address where UDMA descriptor is stored */
	UDMA_Table = (RSI_UDMA_DESC_T *)(cfg->reg->CTRL_BASE_PTR);

	/* Check if the transfer type is memory-memory */
	if ((UDMA_Table[rsi_channel].vsUDMAChaConfigData1.srcInc != UDMA_SRC_INC_NONE) &&
	    (UDMA_Table[rsi_channel].vsUDMAChaConfigData1.dstInc != UDMA_DST_INC_NONE)) {
		/* Apply software trigger to start transfer */
		cfg->reg->CHNL_SW_REQUEST |= SET_BIT(rsi_channel);
	}
	return 0;
}

/* Function to stop a DMA transfer */
static int dma_siwx917_stop(const struct device *dev, uint32_t channel)
{
	const struct dma_siwx917_config *cfg = dev->config;
	struct dma_siwx917_data *data = dev->data;
	RSI_UDMA_HANDLE_T udma_handle = (RSI_UDMA_HANDLE_T)data->dma_rom_buff;

	/* Expecting a fixed channel number between 1-32 for udma0 and 1-12 for udma1 */
	if ((channel > cfg->channels) || (channel == 0)) {
		return -EINVAL;
	}
	/* Disable UDMA channel */
	if (RSI_UDMA_ChannelDisable(udma_handle, (channel - 1)) != 0) {
		return -EIO;
	}
	return 0;
}

/* Function to fetch DMA channel status */
static int dma_siwx917_get_status(const struct device *dev, uint32_t channel,
				  struct dma_status *stat)
{
	const struct dma_siwx917_config *cfg = dev->config;
	uint32_t rsi_channel = channel - 1;
	RSI_UDMA_DESC_T *UDMA_Table;

	/* Expecting a fixed channel number between 1-32 for udma0 and 1-12 for udma1 */
	if ((channel > cfg->channels) || (channel == 0)) {
		return -EINVAL;
	}
	/* Read the channel status register */
	if ((cfg->reg->CHANNEL_STATUS_REG >> rsi_channel) & 0x01) {
		stat->busy = 1;
	} else {
		stat->busy = 0;
	}
	/* Fetch the SRAM address where UDMA descriptor is stored */
	UDMA_Table = (RSI_UDMA_DESC_T *)(cfg->reg->CTRL_BASE_PTR);

	/* Obtain the transfer direction from channel descriptors */
	if (UDMA_Table[rsi_channel].vsUDMAChaConfigData1.srcInc == UDMA_SRC_INC_NONE) {
		stat->dir = PERIPHERAL_TO_MEMORY;
	} else if (UDMA_Table[rsi_channel].vsUDMAChaConfigData1.dstInc == UDMA_DST_INC_NONE) {
		stat->dir = MEMORY_TO_PERIPHERAL;
	} else {
		stat->dir = MEMORY_TO_MEMORY;
	}
	return 0;
}

/* Function to initialize DMA peripheral */
static int dma_siwx917_init(const struct device *dev)
{
	const struct dma_siwx917_config *cfg = dev->config;
	struct dma_siwx917_data *data = dev->data;
	RSI_UDMA_HANDLE_T udma_handle = NULL;
	/* Initialize UDMA Resources */
	UDMA_RESOURCES UDMA_Siwx917_Resources = {
		.reg = cfg->reg, /* UDMA register base address */
		.udma_irq_num = cfg->irq_number,
	};

	if (cfg->reg == UDMA0) {
		UDMA_Siwx917_Resources.desc =
			(RSI_UDMA_DESC_T *)DT_PROP(DT_NODELABEL(udma0), sram_desc_addr);
	} else if (cfg->reg == UDMA1) {
		UDMA_Siwx917_Resources.desc =
			(RSI_UDMA_DESC_T *)DT_PROP(DT_NODELABEL(udma1), sram_desc_addr);
	} else {
		return -EINVAL;
	}

	udma_handle = UDMAx_Initialize(&UDMA_Siwx917_Resources, UDMA_Siwx917_Resources.desc,
				       udma_handle, data->dma_rom_buff);
	if (udma_handle != (RSI_UDMA_HANDLE_T)data->dma_rom_buff) {
		return -EINVAL;
	}

	/* Connect the DMA interrupt */
	cfg->irq_configure();

	/* Enable UDMA instance */
	if (UDMAx_DMAEnable(&UDMA_Siwx917_Resources, udma_handle) != 0) {
		return -EBUSY;
	}
	return 0;
}

static void dma_siwx917_isr(const struct device *dev)
{
	const struct dma_siwx917_config *cfg = dev->config;
	struct dma_siwx917_data *data = dev->data;
	uint32_t irq_number = cfg->irq_number;
	uint8_t transfer_complete = 0;
	uint8_t soft_trig_flag = 0;
	uint32_t int_status;
	uint8_t channel;
	/* Initialize UDMA Resources */
	UDMA_RESOURCES UDMA_Siwx917_Resources = {
		cfg->reg, irq_number,
		(RSI_UDMA_DESC_T *)cfg->reg->CTRL_BASE_PTR /* SRAM base address */
	};

	/* Disable IRQ */
	irq_disable(irq_number);
	int_status = cfg->reg->UDMA_DONE_STATUS_REG; /* Read the interrupt status */
	/* Identify the interrupt channel */
	for (channel = 0; channel < cfg->channels; channel++) {
		if (!(int_status & (1U << channel))) {
			continue;
		}
		if (data->udma_channel_info[channel].Cnt == data->udma_channel_info[channel].Size) {
			transfer_complete = 1;
			break;
		}
		/* Check if the transfer type is memory-memory */
		if ((UDMA_Siwx917_Resources.desc[channel].vsUDMAChaConfigData1.srcInc !=
		     UDMA_SRC_INC_NONE) &&
		    (UDMA_Siwx917_Resources.desc[channel].vsUDMAChaConfigData1.dstInc !=
		     UDMA_DST_INC_NONE)) {
			/* Need to apply a software trigger later */
			soft_trig_flag = 1;
		}
		break;
	}
	if (transfer_complete) {
		if (data->dma_callback != NULL) {
			/* Transfer complete, call user callback */
			data->dma_callback(dev, data->cb_data, channel + 1, 0);
		}
		cfg->reg->UDMA_DONE_STATUS_REG = (1U << channel);
	} else {
		/* Call UDMA ROM IRQ handler. */
		ROMAPI_UDMA_WRAPPER_API->uDMAx_IRQHandler(&UDMA_Siwx917_Resources,
							  UDMA_Siwx917_Resources.desc,
							  data->udma_channel_info);
		if (soft_trig_flag) {
			/* Set the software trigger bit for starting next transfer */
			cfg->reg->CHNL_SW_REQUEST |= (1U << channel);
		}
	}
	/* Enable IRQ */
	irq_enable(irq_number);
}

/* Store the Si91x DMA APIs */
static const struct dma_driver_api siwx917_dma_driver_api = {
	.config = dma_siwx917_configure,
	.reload = dma_siwx917_reload,
	.start = dma_siwx917_start,
	.stop = dma_siwx917_stop,
	.get_status = dma_siwx917_get_status,
};

#define SIWX917_DMA_INIT(inst)                                                                     \
	static UDMA_Channel_Info dma##inst##_channel_info[DT_INST_PROP(inst, dma_channels)];       \
	static struct dma_siwx917_data dma##inst##_data = {                                        \
		.udma_channel_info = dma##inst##_channel_info,                                     \
	};                                                                                         \
	static void siwx917_dma##inst##_irq_configure(void)                                        \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQ(inst, irq), DT_INST_IRQ(inst, priority), dma_siwx917_isr,  \
			    DEVICE_DT_INST_GET(inst), 0);                                          \
		irq_enable(DT_INST_IRQ(inst, irq));                                                \
	}                                                                                          \
	static const struct dma_siwx917_config dma##inst##_cfg = {                                 \
		.reg = (UDMA0_Type *)DT_INST_REG_ADDR(inst),                                       \
		.channels = DT_INST_PROP(inst, dma_channels),                                      \
		.irq_number = DT_INST_PROP_BY_IDX(inst, interrupts, 0),                            \
		.irq_configure = siwx917_dma##inst##_irq_configure,                                \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, &dma_siwx917_init, NULL, &dma##inst##_data, &dma##inst##_cfg,  \
			      PRE_KERNEL_1, CONFIG_DMA_INIT_PRIORITY, &siwx917_dma_driver_api);

DT_INST_FOREACH_STATUS_OKAY(SIWX917_DMA_INIT)
