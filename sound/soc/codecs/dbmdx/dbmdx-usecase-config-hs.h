/*
 * dbmdx-usecase-config-hs.h  --  Preset USE CASE configurations
 *
 * Copyright (C) 2014 DSP Group
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifdef DBMDX_HS_USECASES_SUPPORTED

#ifndef _DBMDX_USECASE_CONFIG_HS_H
#define _DBMDX_USECASE_CONFIG_HS_H

#include "dbmdx-interface.h"
#include "dbmdx-usecase-config-def.h"

/* HS UC1 */
static struct usecase_config config_uc1_hs = {
	.usecase_name = "uc1_hs",
	.id	= 2011,
	.hw_rev = 1,
	.send_va_asrp_parms = false,
	.va_asrp_params_file_name = NULL,
	.send_va_ve_asrp_parms = true,
	.va_ve_asrp_params_file_name = "asrp_params_nr.bin",
	.change_clock_src = false,
	.tdm_clock_freq = TDM_CLK_FREQ_48,
	.number_of_bits = 16,
	.usecase_requires_amodel = false,
	.usecase_amodel_mode = 1,
	.usecase_sets_detection_mode = false,
	.usecase_supports_us_buffering = false,
	.va_chip_low_power_mode = false,
	.va_ve_chip_low_power_mode = false,
	.va_cfg_values = NULL,
	.num_of_va_cfg_values = 0,
	.config_mics = true,
	.mic_config = { 0x0061, 0x0062, 0x0000, 0x0000 },
	.va_ve_cfg_values = (u32 []){ 0x8013FFFF, 0x80340002 },
	.num_of_va_ve_cfg_values = 2,
	.audio_routing_config = { 0x0e10, 0x1eee, 0x2ee3, 0x3eee,
				  0xeeee, 0xeeee, 0xeeee, 0xeeee },
	.tdm_configs = {
		/* DBMD2 TDM0_TX is connected to the HOST */
		{	.tdm_index	= 0x0,
			.tdm_type	= TDM_TYPE_TX,
			.tdm_reg_config	= 0x0bd3,
			.tdm_interface	= TDM_INTERFACE_VA_VE,
			.num_of_io_reg_configs = 3,
			.io_reg_configs	= {
				{		.addr = DBMD2_TDM_0_TX_ADDR,
						.value =
						(0x00800000 | HW_TDM_POLARITY),
				},
				{		.addr = DBMD2_TDM_0_TX_ADDR + 4,
						.value = 0x00072007
				},
				{		.addr = DBMD2_TDM_0_TX_ADDR + 6,
						.value = 0x100F001F
				}
			},
		},
	},

	.num_of_tdm_configs = 1,
	.va_start_cmd_type = START_CMD_TYPE_TDM,
	.send_va_start_cmd = false,
	.va_start_cmd = 0x0000,
	.va_ve_start_cmd_type = START_CMD_TYPE_TDM,
	.send_va_ve_start_cmd = true,
	/* Enable:
			TDM0_TX (D2==>HOST)
	*/
	.va_ve_start_cmd = 0x0208,
};


/* HS UC2*/
static struct usecase_config config_uc2_hs = {
	.usecase_name = "uc2_hs",
	.id	= 2012,
	.hw_rev = 1,
	.send_va_asrp_parms = false,
	.va_asrp_params_file_name = NULL,
	.send_va_ve_asrp_parms = true,
	.va_ve_asrp_params_file_name = "asrp_params_aecnr.bin",
	.change_clock_src = false,
	.tdm_clock_freq = TDM_CLK_FREQ_16,
	.number_of_bits = 16,
	.usecase_requires_amodel = false,
	.usecase_amodel_mode = 1,
	.usecase_sets_detection_mode = false,
	.usecase_supports_us_buffering = false,
	.va_chip_low_power_mode = false,
	.va_ve_chip_low_power_mode = false,
	.va_cfg_values = NULL,
	.num_of_va_cfg_values = 0,
	.config_mics = true,
	.mic_config = { 0x0061, 0x0062, 0x0000, 0x0000 },
	.va_ve_cfg_values = (u32 []){ 0x8013FFFF, 0x80340002 },
	.num_of_va_ve_cfg_values = 2,
	.audio_routing_config = { 0x0210, 0x1e4e, 0x2ee3, 0x3eee,
				  0xeeee, 0xeeee, 0xeeee, 0xeeee },
	.tdm_configs = {
		/* DBMD2 TDM0_RX is connected to the HOST */
		{	.tdm_index	= 0x0,
			.tdm_type	= TDM_TYPE_RX,
			.tdm_reg_config	= 0x9214,
			.tdm_interface	= TDM_INTERFACE_VA_VE,
			.num_of_io_reg_configs = 3,
			.io_reg_configs	= {
				{		.addr = DBMD2_TDM_0_RX_ADDR,
						.value =
						(0x00800000 | HW_TDM_POLARITY),
				},
				{		.addr = DBMD2_TDM_0_RX_ADDR + 4,
						.value = 0x00000027
				},
				{		.addr = DBMD2_TDM_0_RX_ADDR + 6,
						.value = 0x100F001F
				}
			},
		},
		/* DBMD2 TDM0_TX is connected to the HOST */
		{	.tdm_index	= 0x0,
			.tdm_type	= TDM_TYPE_TX,
			.tdm_reg_config	= 0x0313,
			.tdm_interface	= TDM_INTERFACE_VA_VE,
			.num_of_io_reg_configs = 3,
			.io_reg_configs	= {
				{		.addr = DBMD2_TDM_0_TX_ADDR,
						.value =
						(0x00800000 | HW_TDM_POLARITY),
				},
				{		.addr = DBMD2_TDM_0_TX_ADDR + 4,
						.value = 0x00070027
				},
				{		.addr = DBMD2_TDM_0_TX_ADDR + 6,
						.value = 0x100F001F
				}
			},
		},
	},

	.num_of_tdm_configs = 2,
	.va_start_cmd_type = START_CMD_TYPE_TDM,
	.send_va_start_cmd = false,
	.va_start_cmd = 0x0000,
	.va_ve_start_cmd_type = START_CMD_TYPE_TDM,
	.send_va_ve_start_cmd = true,
	/* Enable:
		TDM0_RX (HOST==>D2) (reference)
		TDM0_TX (D2==>HOST)
	*/
	.va_ve_start_cmd = 0x0308,
};

#endif /* _DBMDX_USECASE_CONFIG_HS_H */

#endif /* DBMDX_HS_USECASES_SUPPORTED */
