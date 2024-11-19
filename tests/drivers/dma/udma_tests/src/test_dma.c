/*
 * Copyright (c) 2024 Silicon Laboratories Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/ztest.h>

#define BUFF_LENGTH1 100
#define BUFF_LENGTH2 1500

static uint8_t trx_buffer8_1[BUFF_LENGTH1];
static uint8_t rrx_buffer8_1[BUFF_LENGTH1];

static uint8_t trx_buffer8_2[BUFF_LENGTH2];
static uint8_t rrx_buffer8_2[BUFF_LENGTH2];

static uint16_t trx_buffer16_1[BUFF_LENGTH1];
static uint16_t rrx_buffer16_1[BUFF_LENGTH1];

static uint16_t trx_buffer16_2[BUFF_LENGTH2];
static uint16_t rrx_buffer16_2[BUFF_LENGTH2];

static uint32_t trx_buffer32_1[BUFF_LENGTH1];
static uint32_t rrx_buffer32_1[BUFF_LENGTH1];

static uint32_t trx_buffer32_2[BUFF_LENGTH2];
static uint32_t rrx_buffer32_2[BUFF_LENGTH2];

static void test_done(const struct device *dma_dev, void *arg, uint32_t id, int status)
{
	if (status >= 0) {
		TC_PRINT("DMA transfer done\n");
	} else {
		TC_PRINT("DMA transfer met an error\n");
	}
}

static int test_task8_1(const struct device *dma, uint32_t chan_id)
{
	struct dma_config dma_cfg = {0};
	struct dma_block_config dma_block_cfg = {0};
	uint32_t blen = 8;

	if (!device_is_ready(dma)) {
		TC_PRINT("dma controller device is not ready\n");
		return TC_FAIL;
	}

	for (int i = 0; i < BUFF_LENGTH1; i++) {
		trx_buffer8_1[i] = (uint8_t)(i + 1);
	}
	dma_cfg.channel_direction = MEMORY_TO_MEMORY;
	dma_cfg.source_data_size = 1U;
	dma_cfg.dest_data_size = 1U;
	dma_cfg.source_burst_length = blen;
	dma_cfg.dest_burst_length = blen;
	dma_cfg.dma_callback = test_done;
	dma_cfg.complete_callback_en = 0U;
	dma_cfg.error_callback_dis = 0U;
	dma_cfg.block_count = 1U;
	dma_cfg.head_block = &dma_block_cfg;

	TC_PRINT("Preparing DMA Controller: Name=%s, Chan_ID=%u, BURST_LEN=%u\n", dma->name,
		 chan_id, blen >> 3);

	TC_PRINT("Starting the transfer\n");
	(void)memset(rrx_buffer8_1, 0, sizeof(rrx_buffer8_1));
	dma_block_cfg.block_size = sizeof(trx_buffer8_1);
	dma_block_cfg.source_address = (uint32_t)trx_buffer8_1;
	dma_block_cfg.dest_address = (uint32_t)rrx_buffer8_1;
	if (dma_config(dma, chan_id, &dma_cfg)) {
		TC_PRINT("ERROR: transfer\n");
		return TC_FAIL;
	}
	if (dma_start(dma, chan_id)) {
		TC_PRINT("ERROR: transfer\n");
		return TC_FAIL;
	}
	k_sleep(K_MSEC(2000));

	int counter;

	for (counter = 0; counter < BUFF_LENGTH1; counter++) {
		if (rrx_buffer8_1[counter] != trx_buffer8_1[counter]) {
			TC_PRINT("Test case fail\n");
			break;
		}
	}
	if (counter != BUFF_LENGTH1) {
		return TC_FAIL;
	}
	return TC_PASS;
}

static int test_task_reload8_2(const struct device *dma, uint32_t chan_id)
{

	for (int i = 0; i < BUFF_LENGTH2; i++) {
		trx_buffer8_2[i] = (uint8_t)(i + 1);
	}
	(void)memset(rrx_buffer8_2, 0, sizeof(rrx_buffer8_2));
	TC_PRINT("Reloading DMA Controller with %d buffer size\n", BUFF_LENGTH2);
	TC_PRINT("Starting the transfer\n");
	if (dma_reload(dma, chan_id, (uint32_t)trx_buffer8_2, (uint32_t)rrx_buffer8_2,
		       sizeof(trx_buffer8_2))) {
		TC_PRINT("ERROR: transfer\n");
		return TC_FAIL;
	}
	if (dma_start(dma, chan_id)) {
		TC_PRINT("ERROR: transfer\n");
		return TC_FAIL;
	}
	k_sleep(K_MSEC(2000));
	int counter;

	for (counter = 0; counter < BUFF_LENGTH2; counter++) {
		if (rrx_buffer8_2[counter] != trx_buffer8_2[counter]) {
			TC_PRINT("Test case fail\n");
			break;
		}
	}
	if (counter != BUFF_LENGTH2) {
		return TC_FAIL;
	}
	TC_PRINT("Test case pass\n");
	return TC_PASS;
}

static int test_task16_1(const struct device *dma, uint32_t chan_id)
{
	struct dma_config dma_cfg = {0};
	struct dma_block_config dma_block_cfg = {0};
	uint32_t blen = 8;

	if (!device_is_ready(dma)) {
		TC_PRINT("dma controller device is not ready\n");
		return TC_FAIL;
	}

	for (int i = 0; i < BUFF_LENGTH1; i++) {
		trx_buffer16_1[i] = (uint16_t)(i + 1);
	}
	dma_cfg.channel_direction = MEMORY_TO_MEMORY;
	dma_cfg.source_data_size = 2U;
	dma_cfg.dest_data_size = 2U;
	dma_cfg.source_burst_length = blen;
	dma_cfg.dest_burst_length = blen;
	dma_cfg.dma_callback = test_done;
	dma_cfg.complete_callback_en = 0U;
	dma_cfg.error_callback_dis = 0U;
	dma_cfg.block_count = 1U;
	dma_cfg.head_block = &dma_block_cfg;

	TC_PRINT("Preparing DMA Controller: Name=%s, Chan_ID=%u, BURST_LEN=%u\n", dma->name,
		 chan_id, blen >> 3);

	TC_PRINT("Starting the transfer\n");
	(void)memset(rrx_buffer16_1, 0, sizeof(rrx_buffer16_1));
	dma_block_cfg.block_size = sizeof(trx_buffer16_1);
	dma_block_cfg.source_address = (uint32_t)trx_buffer16_1;
	dma_block_cfg.dest_address = (uint32_t)rrx_buffer16_1;
	if (dma_config(dma, chan_id, &dma_cfg)) {
		TC_PRINT("ERROR: transfer\n");
		return TC_FAIL;
	}
	if (dma_start(dma, chan_id)) {
		TC_PRINT("ERROR: transfer\n");
		return TC_FAIL;
	}
	k_sleep(K_MSEC(2000));

	int counter;

	for (counter = 0; counter < BUFF_LENGTH1; counter++) {
		if (rrx_buffer16_1[counter] != trx_buffer16_1[counter]) {
			TC_PRINT("Test case fail\n");
			break;
		}
	}
	if (counter != BUFF_LENGTH1) {
		return TC_FAIL;
	}
	TC_PRINT("Test case pass\n");
	return TC_PASS;
}

static int test_task_reload16_2(const struct device *dma, uint32_t chan_id)
{

	for (int i = 0; i < BUFF_LENGTH2; i++) {
		trx_buffer16_2[i] = (uint16_t)(i + 1);
	}
	(void)memset(rrx_buffer16_2, 0, sizeof(rrx_buffer16_2));
	TC_PRINT("Reloading DMA Controller with %d buffer size\n", BUFF_LENGTH2);
	TC_PRINT("Starting the transfer\n");
	if (dma_reload(dma, chan_id, (uint32_t)trx_buffer16_2, (uint32_t)rrx_buffer16_2,
		       sizeof(trx_buffer16_2))) {
		TC_PRINT("ERROR: transfer\n");
		return TC_FAIL;
	}
	if (dma_start(dma, chan_id)) {
		TC_PRINT("ERROR: transfer\n");
		return TC_FAIL;
	}
	k_sleep(K_MSEC(2000));
	int counter;

	for (counter = 0; counter < BUFF_LENGTH2; counter++) {
		if (rrx_buffer16_2[counter] != trx_buffer16_2[counter]) {
			TC_PRINT("Test case fail\n");
			break;
		}
	}
	if (counter != BUFF_LENGTH2) {
		return TC_FAIL;
	}
	TC_PRINT("Test case pass\n");
	return TC_PASS;
}

static int test_task32_1(const struct device *dma, uint32_t chan_id)
{
	struct dma_config dma_cfg = {0};
	struct dma_block_config dma_block_cfg = {0};
	uint32_t blen = 8;

	if (!device_is_ready(dma)) {
		TC_PRINT("dma controller device is not ready\n");
		return TC_FAIL;
	}

	for (int i = 0; i < BUFF_LENGTH1; i++) {
		trx_buffer32_1[i] = (uint32_t)(i + 1);
	}
	dma_cfg.channel_direction = MEMORY_TO_MEMORY;
	dma_cfg.source_data_size = 4U;
	dma_cfg.dest_data_size = 4U;
	dma_cfg.source_burst_length = blen;
	dma_cfg.dest_burst_length = blen;
	dma_cfg.dma_callback = test_done;
	dma_cfg.complete_callback_en = 0U;
	dma_cfg.error_callback_dis = 0U;
	dma_cfg.block_count = 1U;
	dma_cfg.head_block = &dma_block_cfg;

	TC_PRINT("Preparing DMA Controller: Name=%s, Chan_ID=%u, BURST_LEN=%u\n", dma->name,
		 chan_id, blen >> 3);

	TC_PRINT("Starting the transfer\n");
	(void)memset(rrx_buffer32_1, 0, sizeof(rrx_buffer32_1));
	dma_block_cfg.block_size = sizeof(trx_buffer32_1);
	dma_block_cfg.source_address = (uint32_t)trx_buffer32_1;
	dma_block_cfg.dest_address = (uint32_t)rrx_buffer32_1;
	if (dma_config(dma, chan_id, &dma_cfg)) {
		TC_PRINT("ERROR: transfer\n");
		return TC_FAIL;
	}
	if (dma_start(dma, chan_id)) {
		TC_PRINT("ERROR: transfer\n");
		return TC_FAIL;
	}
	k_sleep(K_MSEC(2000));

	int counter;

	for (counter = 0; counter < BUFF_LENGTH1; counter++) {
		if (rrx_buffer32_1[counter] != trx_buffer32_1[counter]) {
			TC_PRINT("Test case fail\n");
			break;
		}
	}
	if (counter != BUFF_LENGTH1) {
		return TC_FAIL;
	}
	TC_PRINT("Test case pass\n");
	return TC_PASS;
}

static int test_task_reload32_2(const struct device *dma, uint32_t chan_id)
{

	for (int i = 0; i < BUFF_LENGTH2; i++) {
		trx_buffer32_2[i] = (uint32_t)(i + 1);
	}
	(void)memset(rrx_buffer32_2, 0, sizeof(rrx_buffer32_2));
	TC_PRINT("Reloading DMA Controller with %d buffer size\n", BUFF_LENGTH2);
	TC_PRINT("Starting the transfer\n");
	if (dma_reload(dma, chan_id, (uint32_t)trx_buffer32_2, (uint32_t)rrx_buffer32_2,
		       sizeof(trx_buffer32_2))) {
		TC_PRINT("ERROR: transfer\n");
		return TC_FAIL;
	}
	if (dma_start(dma, chan_id)) {
		TC_PRINT("ERROR: transfer\n");
		return TC_FAIL;
	}
	k_sleep(K_MSEC(2000));
	int counter;

	for (counter = 0; counter < BUFF_LENGTH2; counter++) {
		if (rrx_buffer32_2[counter] != trx_buffer32_2[counter]) {
			TC_PRINT("Test case fail\n");
			break;
		}
	}
	if (counter != BUFF_LENGTH2) {
		return TC_FAIL;
	}
	TC_PRINT("Test case pass\n");
	return TC_PASS;
}

#define DMA_NAME(i, _) tst_dma##i
#define DMA_LIST       LISTIFY(CONFIG_DMA_LOOP_TRANSFER_NUMBER_OF_DMAS, DMA_NAME, (,))

#define TEST_TASK(dma_name)                                                                        \
	ZTEST(dma_m2m, test_##dma_name##_m2m_chan0_8_buf1)                                         \
	{                                                                                          \
		const struct device *dma = DEVICE_DT_GET(DT_NODELABEL(dma_name));                  \
		zassert_true((test_task8_1(dma, CONFIG_DMA_TRANSFER_CHANNEL_NR_0) == TC_PASS));    \
	}                                                                                          \
                                                                                                   \
	ZTEST(dma_m2m, test_##dma_name##_m2m_chan1_8_buf1)                                         \
	{                                                                                          \
		const struct device *dma = DEVICE_DT_GET(DT_NODELABEL(dma_name));                  \
		zassert_true((test_task8_1(dma, CONFIG_DMA_TRANSFER_CHANNEL_NR_1) == TC_PASS));    \
	}                                                                                          \
	ZTEST(dma_m2m, test_##dma_name##_m2m_chan0_reload8_buf2)                                   \
	{                                                                                          \
		const struct device *dma = DEVICE_DT_GET(DT_NODELABEL(dma_name));                  \
		zassert_true(                                                                      \
			(test_task_reload8_2(dma, CONFIG_DMA_TRANSFER_CHANNEL_NR_0) == TC_PASS));  \
	}                                                                                          \
                                                                                                   \
	ZTEST(dma_m2m, test_##dma_name##_m2m_chan1_reload8_buf2)                                   \
	{                                                                                          \
		const struct device *dma = DEVICE_DT_GET(DT_NODELABEL(dma_name));                  \
		zassert_true(                                                                      \
			(test_task_reload8_2(dma, CONFIG_DMA_TRANSFER_CHANNEL_NR_1) == TC_PASS));  \
	}                                                                                          \
	ZTEST(dma_m2m, test_##dma_name##_m2m_chan0_16_buf1)                                        \
	{                                                                                          \
		const struct device *dma = DEVICE_DT_GET(DT_NODELABEL(dma_name));                  \
		zassert_true((test_task16_1(dma, CONFIG_DMA_TRANSFER_CHANNEL_NR_0) == TC_PASS));   \
	}                                                                                          \
                                                                                                   \
	ZTEST(dma_m2m, test_##dma_name##_m2m_chan1_16_buf1)                                        \
	{                                                                                          \
		const struct device *dma = DEVICE_DT_GET(DT_NODELABEL(dma_name));                  \
		zassert_true((test_task16_1(dma, CONFIG_DMA_TRANSFER_CHANNEL_NR_1) == TC_PASS));   \
	}                                                                                          \
	ZTEST(dma_m2m, test_##dma_name##_m2m_chan0_reload16_buf2)                                  \
	{                                                                                          \
		const struct device *dma = DEVICE_DT_GET(DT_NODELABEL(dma_name));                  \
		zassert_true(                                                                      \
			(test_task_reload16_2(dma, CONFIG_DMA_TRANSFER_CHANNEL_NR_0) == TC_PASS)); \
	}                                                                                          \
                                                                                                   \
	ZTEST(dma_m2m, test_##dma_name##_m2m_chan1_reload16_buf2)                                  \
	{                                                                                          \
		const struct device *dma = DEVICE_DT_GET(DT_NODELABEL(dma_name));                  \
		zassert_true(                                                                      \
			(test_task_reload16_2(dma, CONFIG_DMA_TRANSFER_CHANNEL_NR_1) == TC_PASS)); \
	}                                                                                          \
	ZTEST(dma_m2m, test_##dma_name##_m2m_chan0_32_buf1)                                        \
	{                                                                                          \
		const struct device *dma = DEVICE_DT_GET(DT_NODELABEL(dma_name));                  \
		zassert_true((test_task32_1(dma, CONFIG_DMA_TRANSFER_CHANNEL_NR_0) == TC_PASS));   \
	}                                                                                          \
                                                                                                   \
	ZTEST(dma_m2m, test_##dma_name##_m2m_chan1_32_buf1)                                        \
	{                                                                                          \
		const struct device *dma = DEVICE_DT_GET(DT_NODELABEL(dma_name));                  \
		zassert_true((test_task32_1(dma, CONFIG_DMA_TRANSFER_CHANNEL_NR_1) == TC_PASS));   \
	}                                                                                          \
	ZTEST(dma_m2m, test_##dma_name##_m2m_chan0_reload32_buf2)                                  \
	{                                                                                          \
		const struct device *dma = DEVICE_DT_GET(DT_NODELABEL(dma_name));                  \
		zassert_true(                                                                      \
			(test_task_reload32_2(dma, CONFIG_DMA_TRANSFER_CHANNEL_NR_0) == TC_PASS)); \
	}                                                                                          \
                                                                                                   \
	ZTEST(dma_m2m, test_##dma_name##_m2m_chan1_reload32_buf2)                                  \
	{                                                                                          \
		const struct device *dma = DEVICE_DT_GET(DT_NODELABEL(dma_name));                  \
		zassert_true(                                                                      \
			(test_task_reload32_2(dma, CONFIG_DMA_TRANSFER_CHANNEL_NR_1) == TC_PASS)); \
	}

FOR_EACH(TEST_TASK, (), DMA_LIST)
	;
