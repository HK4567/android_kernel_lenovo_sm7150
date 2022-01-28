/*
 * dbmdx-usecase-config-def  --  Definition of USE CASE configurations
 *
 * Copyright (C) 2014 DSP Group
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _DBMDX_USECASE_CONFIG_DEF_H
#define _DBMDX_USECASE_CONFIG_DEF_H

#include "dbmdx-interface.h"

#define NUM_OF_TDM_CONF_REGS		6
#define MAX_NUMBER_OF_TDM_CONFIGS	12
#define NUM_OF_AUDIO_ROUTING_CONF	9
#define MAX_USECASE_FILENAME_LEN	256
#define RESERVED_OFFSET			32

#define DBMDX_USECASE_MGR_CMD_MASK			0x0000000f
#define DBMDX_USECASE_MGR_CMD_BITS			0
#define DBMDX_USECASE_MGR_MODE_MASK			0x000000f0
#define DBMDX_USECASE_MGR_MODE_BITS			4
#define DBMDX_USECASE_MGR_UID_MASK			0x000fff00
#define DBMDX_USECASE_MGR_UID_BITS			8
#define DBMDX_USECASE_MGR_MODEL_MODE_MASK		0x00f00000
#define DBMDX_USECASE_MGR_MODEL_MODE_BITS		20
#define DBMDX_USECASE_MGR_MODEL_SELECT_MASK		0x03000000
#define DBMDX_USECASE_MGR_MODEL_SELECT_BITS		24
#define DBMDX_USECASE_MGR_MODEL_OPTIONS_MASK		0x1C000000
#define DBMDX_USECASE_MGR_MODEL_OPTIONS_BITS		26
#define DBMDX_USECASE_MGR_MODEL_CUSTOM_PARAMS_MASK	0xE0000000
#define DBMDX_USECASE_MGR_MODEL_CUSTOM_PARAMS_BITS	29

#define DBMDX_USECASE_MGR_CMD_SET_IDLE		0x0000
#define DBMDX_USECASE_MGR_CMD_LOAD		0x0001
#define DBMDX_USECASE_MGR_CMD_START		0x0002
#define DBMDX_USECASE_MGR_CMD_SET_MODE		0x0004
#define DBMDX_USECASE_MGR_CMD_LOAD_AMODEL	0x0008

#define DBMDX_USECASE_ID_HWREV_00		0x0000
#define DBMDX_USECASE_ID_HWREV_01		0x0001
#define DBMDX_USECASE_ID_HWREV_02		0x0002
#define DBMDX_USECASE_ID_HWREV_03		0x0003
#define DBMDX_USECASE_ID_HWREV_04		0x0004
#define DBMDX_USECASE_ID_HWREV_05		0x0005

#define DBMDX_USECASE_ID_PRJ_MANGO		0x0010
#define DBMDX_USECASE_ID_PRJ_MELON		0x0020
#define DBMDX_USECASE_ID_PRJ_ORANGE		0x0030
#define DBMDX_USECASE_ID_PRJ_KIWI		0x0040

#define DBMDX_USECASE_ID_UC_IDX_00		0x0000
#define DBMDX_USECASE_ID_UC_IDX_01		0x0100
#define DBMDX_USECASE_ID_UC_IDX_02		0x0200
#define DBMDX_USECASE_ID_UC_IDX_03		0x0300
#define DBMDX_USECASE_ID_UC_IDX_04		0x0400
#define DBMDX_USECASE_ID_UC_IDX_05		0x0500
#define DBMDX_USECASE_ID_UC_IDX_06		0x0600
#define DBMDX_USECASE_ID_UC_IDX_07		0x0700
#define DBMDX_USECASE_ID_UC_IDX_08		0x0800

#if HOST_HW_TDM_CONF_ASYNC_DOWN_DATA_DOWN
#define HW_TDM_POLARITY 0x0000000F
#elif HOST_HW_TDM_CONF_ASYNC_DOWN_DATA_UP
#define HW_TDM_POLARITY 0x0000000D
#elif HOST_HW_TDM_CONF_ASYNC_UP_DATA_DOWN
#define HW_TDM_POLARITY 0x00000007
#elif HOST_HW_TDM_CONF_ASYNC_UP_DATA_UP
#define HW_TDM_POLARITY 0x00000005
#else
#define HW_TDM_POLARITY 0x00000005
#endif

enum TDM_TYPE_TX_RX {
	TDM_TYPE_TX = 0,
	TDM_TYPE_RX,
};

enum TDM_INTERFACE {
	TDM_INTERFACE_VA = 0,
	TDM_INTERFACE_VA_VE,
};

struct io_register {
	u32			addr;
	u32			value;
};

enum TDM_CLK_FREQ_CONF {
	TDM_CLK_FREQ_0 = 0,
	TDM_CLK_FREQ_16 = 16,
	TDM_CLK_FREQ_48 = 48,
};

enum MIC_CONFIG_TYPE {
	DO_NOT_CONFIG_MICS = 0,
	MIC_CONFIG_BY_USECASE = 1,
	MIC_CONFIG_BY_USER_MASK = 2,
};

enum START_CMD_TYPE {
	START_CMD_TYPE_TDM = 0,
	START_CMD_TYPE_OPMODE = 1,
};

enum OUTPUT_GAIN_TYPE {
	ASRP_TX_OUT_GAIN = 0x1,
	ASRP_VCPF_OUT_GAIN = 0x2,
	ASRP_RX_OUT_GAIN = 0x4,
};

struct tdm_config {
	u32			tdm_index;
	enum TDM_TYPE_TX_RX	tdm_type;
	u32			tdm_reg_config;
	enum TDM_INTERFACE	tdm_interface;
	u32			num_of_io_reg_configs;
	struct io_register	io_reg_configs[NUM_OF_TDM_CONF_REGS];
};

struct usecase_config {
	const char		*usecase_name;
	u32			id;
	u32			hw_rev;
	bool			send_va_asrp_parms;
	const char		*va_asrp_params_file_name;
	bool			send_va_ve_asrp_parms;
	const char		*va_ve_asrp_params_file_name;
	enum MIC_CONFIG_TYPE	config_mics;
	u32			mic_config[4];
	u32			audio_routing_config[NUM_OF_AUDIO_ROUTING_CONF];
	struct tdm_config	tdm_configs[MAX_NUMBER_OF_TDM_CONFIGS];
	u32			num_of_tdm_configs;
	u32			*va_cfg_values;
	u32			num_of_va_cfg_values;
	u32			*va_post_tdm_cfg_values;
	u32			num_of_va_post_tdm_cfg_values;
	u32			*va_ve_cfg_values;
	u32			num_of_va_ve_cfg_values;
	u32			*va_ve_post_tdm_cfg_values;
	u32			num_of_va_ve_post_tdm_cfg_values;
	bool			send_va_start_cmd;
	enum START_CMD_TYPE	va_start_cmd_type;
	u32			va_start_cmd;
	u32			va_start_i2s_buffering_cmd;
	bool			send_va_ve_start_cmd;
	enum START_CMD_TYPE	va_ve_start_cmd_type;
	u32			va_ve_start_cmd;
	bool			change_clock_src;
	enum TDM_CLK_FREQ_CONF	tdm_clock_freq;
	int			number_of_bits;
	u32			custom_d4_clock_hz;
	u32			custom_d2_clock_hz;
	bool			usecase_requires_amodel;
	u32			usecase_amodel_mode;
	bool			usecase_sets_detection_mode;
	bool			usecase_supports_us_buffering;
	bool			usecase_supports_i2s_buffering;
	bool			va_chip_low_power_mode;
	bool			va_ve_chip_low_power_mode;
	enum OUTPUT_GAIN_TYPE	asrp_output_gain_type;
	u32			num_of_output_channels;
};

#endif
