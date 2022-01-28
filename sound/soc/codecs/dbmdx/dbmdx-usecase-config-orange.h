/*
 * dbmdx-usecase-config-orange.h  --  Preset USE CASE configurations
 *
 * Copyright (C) 2014 DSP Group
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifdef DBMDX_ORANGE_USECASES_SUPPORTED

#ifndef _DBMDX_USECASE_CONFIG_ORANGE_H
#define _DBMDX_USECASE_CONFIG_ORANGE_H

#include "dbmdx-interface.h"
#include "dbmdx-usecase-config-def.h"

#ifndef DBMDX_SUPPORT_OBSOLETE_ORANGE_USECASES

/* Orange UC2 - AECNR Dual Mic 48Khz */
static struct usecase_config config_uc_orange_aec_dual_mic_48k = {
	.usecase_name = "uc_orange_aec_dual_mic_48k",
	.id	= (DBMDX_USECASE_ID_UC_IDX_02 |
			DBMDX_USECASE_ID_PRJ_ORANGE |
			DBMDX_USECASE_ID_HWREV_03),
	.hw_rev = 3,
	.send_va_asrp_parms = false,
	.va_asrp_params_file_name = NULL,
	.send_va_ve_asrp_parms = true,
	.va_ve_asrp_params_file_name = "asrp_params_aecnr_2mics.bin",
	.change_clock_src = false,
	.tdm_clock_freq = TDM_CLK_FREQ_48,
#ifdef AEC_REF_32_TO_16_BIT
	.number_of_bits = 32,
#else
	.number_of_bits = 16,
#endif
	.usecase_requires_amodel = false,
	.usecase_amodel_mode = 1,
	.usecase_sets_detection_mode = false,
	.usecase_supports_us_buffering = false,
	.va_chip_low_power_mode = false,
	.va_ve_chip_low_power_mode = false,
	.asrp_output_gain_type = (ASRP_TX_OUT_GAIN |
					ASRP_VCPF_OUT_GAIN |
					ASRP_RX_OUT_GAIN),

	.va_cfg_values = NULL,
	.num_of_va_cfg_values = 0,
	.num_of_va_post_tdm_cfg_values = 0,
	.usecase_supports_i2s_buffering = false,
	.config_mics = MIC_CONFIG_BY_USECASE,
	.mic_config = {
		(DBMDX_REGV_D2_DM_CLK_SRC_DM0_TDM2_TX |
		 DBMDX_REGV_D2_DM_DATA_SRC_DM0_TDM2_RX |
		 DBMDX_REGV_DDF_AUDIO_ATTENUATION_0dB |
		 DBMDX_REGV_DM_CLK_FREQ_1536_1200_SR_8KHz_16KHz_32KHz_48KHz |
		 DBMDX_REGV_D2_MIC_SRC_RIGHT_DMIC_DM0),
		(DBMDX_REGV_D2_DM_CLK_SRC_DM0_TDM2_TX |
		 DBMDX_REGV_D2_DM_DATA_SRC_DM0_TDM2_RX |
		 DBMDX_REGV_DDF_AUDIO_ATTENUATION_0dB |
		 DBMDX_REGV_DM_CLK_FREQ_1536_1200_SR_8KHz_16KHz_32KHz_48KHz |
		 DBMDX_REGV_D2_MIC_SRC_LEFT_DMIC_DM0),
		0x0000,
		0x0000},
	.va_ve_cfg_values = (u32 []){
		(DBMDX_REGN_ASRP_OUTPUT_ROUTING |
			DBMDX_REGV_ASRP_OUTPUT_SRC_BFPF_1 |
			DBMDX_REGV_ASRP_OUTPUT_DEST_TX_1),
		(DBMDX_REGN_AUDIO_PROCESSING_CONFIG |
			DBMDX_REGV_ALGO1_EN_FW_MODE_2_ONLY),
		(DBMDX_VA_USLEEP | 0x1500) },
	.num_of_va_ve_cfg_values = 3,
	.va_ve_post_tdm_cfg_values = (u32 []){
		(DBMDX_REGN_AUDIO_ROUTING_CONFIG |
			DBMDX_REGV_TDM_SYNC_RIGHT_CH |
			DBMDX_REGV_USE_TDM_MUSIC_TO_SYNC |
			DBMDX_REGV_MUSIC_IN_TDM3 |
			DBMDX_REGV_TDM_SYNC_DELAY_4_CLKS_CYCLES) },
	.num_of_va_ve_post_tdm_cfg_values = 1,

	.audio_routing_config = {
		(DBMDX_REGV_IO_SET_0 |
			DBMDX_REGV_IO_3N_2_NO_CP |
			DBMDX_REGV_IO_3N_1_CP_1 |
			DBMDX_REGV_IO_3N_0_CP_0),
		(DBMDX_REGV_IO_SET_1 |
			DBMDX_REGV_IO_3N_2_NO_CP |
			DBMDX_REGV_IO_3N_1_NO_CP |
			DBMDX_REGV_IO_3N_0_NO_CP),
		(DBMDX_REGV_IO_SET_2 |
			DBMDX_REGV_IO_3N_2_CP_4 |
			DBMDX_REGV_IO_3N_1_NO_CP |
			DBMDX_REGV_IO_3N_0_NO_CP),
		(DBMDX_REGV_IO_SET_3 |
			DBMDX_REGV_IO_3N_2_NO_CP |
			DBMDX_REGV_IO_3N_1_NO_CP |
			DBMDX_REGV_IO_3N_0_NO_CP),
		(DBMDX_REGV_IO_SET_4 |
			DBMDX_REGV_IO_3N_2_NO_CP |
			DBMDX_REGV_IO_3N_1_NO_CP |
			DBMDX_REGV_IO_3N_0_NO_CP),
		(DBMDX_REGV_IO_SET_5 |
			DBMDX_REGV_IO_3N_2_NO_CP |
			DBMDX_REGV_IO_3N_1_NO_CP |
			DBMDX_REGV_IO_3N_0_CP_0),
		(DBMDX_REGV_IO_SET_6 |
			DBMDX_REGV_IO_3N_2_NO_CP |
			DBMDX_REGV_IO_3N_1_NO_CP |
			DBMDX_REGV_IO_3N_0_NO_CP),
		(DBMDX_REGV_IO_SET_7 |
			DBMDX_REGV_IO_3N_2_NO_CP |
			DBMDX_REGV_IO_3N_1_NO_CP |
			DBMDX_REGV_IO_3N_0_NO_CP),
		(DBMDX_REGV_IO_SET_8 |
			DBMDX_REGV_IO_3N_2_NO_CP |
			DBMDX_REGV_IO_3N_1_NO_CP |
			DBMDX_REGV_IO_3N_0_NO_CP) },

	.tdm_configs = {
		/* DBMD2 TDM3_RX is connected to the Host Codec */
		{	.tdm_index	= 0x3,
			.tdm_type	= TDM_TYPE_RX,
			.tdm_reg_config	=
				(DBMDX_REGV_DEMUX_MUX_DISABLE |
				DBMDX_REGV_SAMPLE_WIDTH_16_BIT |
				DBMDX_REGV_NUM_OF_CHANNELS_1_CH |
				DBMDX_REGV_INPUT_OUTPUT_SAMPLE_RATE_48_KHz |
				DBMDX_REGV_RX_TX_I2S_CH_USE_HIGH_WORD_ONLY |
				DBMDX_REGV_RESAMPLE_TYPE_DECIMATION |
				DBMDX_REGV_RESAMPLE_RATIO_3_1 |
				DBMDX_REGV_HW_BLOCK_EN |
				DBMDX_REGV_RX_TX_CP4),
			.tdm_interface	= TDM_INTERFACE_VA_VE,
#ifdef AEC_REF_32_TO_16_BIT
			.num_of_io_reg_configs = 4,
			.io_reg_configs	= {
				{		.addr = DBMD2_TDM_3_RX_ADDR,
						.value = 0x0080405D,
				},
				{		.addr = DBMD2_TDM_3_RX_ADDR + 4,
						.value = 0x00002064
				},
				{		.addr = DBMD2_TDM_3_RX_ADDR + 6,
						.value = 0x103F003F
				},
				{		.addr = DBMD2_TDM_3_RX_ADDR+0xA,
						.value = 0x00000005
				}
			},
#else
			.num_of_io_reg_configs = 3,
			.io_reg_configs	= {
				{		.addr = DBMD2_TDM_3_RX_ADDR,
						.value = 0x00800015,
				},
				{		.addr = DBMD2_TDM_3_RX_ADDR + 4,
						.value = 0x00000007
				},
				{		.addr = DBMD2_TDM_3_RX_ADDR + 6,
						.value = 0x100F001F
				}
			},
#endif /* AEC_REF_32_TO_16_BIT */
		},
		/* DBMD2 TDM3_TX is connected to Host Codec */
		{	.tdm_index	= 0x3,
			.tdm_type	= TDM_TYPE_TX,
			.tdm_reg_config	=
			(DBMDX_REGV_DEMUX_MUX_DISABLE |
			 DBMDX_REGV_SAMPLE_WIDTH_16_BIT |
			 DBMDX_REGV_NUM_OF_CHANNELS_1_CH |
			 DBMDX_REGV_INPUT_OUTPUT_SAMPLE_RATE_48_KHz |
		DBMDX_REGV_RX_TX_I2S_CH_SUM_HIGH_AND_LOW_INTO_ONE_SAMPLE |
			 DBMDX_REGV_RESAMPLE_TYPE_INTERPOLATION |
			 DBMDX_REGV_RESAMPLE_RATIO_3_1 |
			 DBMDX_REGV_HW_BLOCK_EN |
			 DBMDX_REGV_RX_TX_CP0),
			.tdm_interface	= TDM_INTERFACE_VA_VE,
			.num_of_io_reg_configs = 4,
#ifdef AEC_REF_32_TO_16_BIT
			.io_reg_configs	= {
				{		.addr = DBMD2_TDM_3_TX_ADDR,
						.value = 0x00804053
				},
				{		.addr = DBMD2_TDM_3_TX_ADDR + 4,
						.value = 0x00000064
				},
				{		.addr = DBMD2_TDM_3_TX_ADDR + 6,
						.value = 0x101F003F
				},
				{		.addr = DBMD2_TDM_3_TX_ADDR+0xA,
						.value = 0x00000005
				}
			},
#else
			.io_reg_configs	= {
				{		.addr = DBMD2_TDM_3_TX_ADDR,
						.value = 0x00804053
				},
				{		.addr = DBMD2_TDM_3_TX_ADDR + 4,
						.value = 0x00441044
				},
				{		.addr = DBMD2_TDM_3_TX_ADDR + 6,
						.value = 0x100F001F
				},
				{		.addr = DBMD2_TDM_3_TX_ADDR+0xA,
						.value = 0x0000000F
				}
			},
#endif /* AEC_REF_32_TO_16_BIT */
		},
	},

	.num_of_tdm_configs = 2,
	.va_start_cmd_type = START_CMD_TYPE_TDM,
	.send_va_start_cmd = false,
	.va_start_cmd = 0x0000,
	.va_ve_start_cmd_type = START_CMD_TYPE_TDM,
	.send_va_ve_start_cmd = true,
	/* Enable:
		TDM3_RX (HOST==>D2) (reference)
		TDM3_TX (D2==>HOST)
	*/
	.va_ve_start_cmd = (DBMDX_REGV_TDM3_RX_EN |
				DBMDX_REGV_TDM3_TX_EN |
				DBMDX_REGV_PROC_EN_SWITCH_FW_TO_BUFFERING_MODE),
};

#else /* DBMDX_SUPPORT_OBSOLETE_ORANGE_USECASES */

/* Orange UC1 - NR Tripple Mic */
static struct usecase_config config_uc_orange_nr_tripple_mic = {
	.usecase_name = "uc_orange_nr_tripple_mic",
	.id	= (DBMDX_USECASE_ID_UC_IDX_01 |
			DBMDX_USECASE_ID_PRJ_ORANGE |
			DBMDX_USECASE_ID_HWREV_01),
	.hw_rev = 1,
	.send_va_asrp_parms = false,
	.va_asrp_params_file_name = NULL,
	.send_va_ve_asrp_parms = true,
	.va_ve_asrp_params_file_name = "asrp_params_nr.bin",
	.change_clock_src = true,
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
	.config_mics = MIC_CONFIG_BY_USECASE,
	.mic_config = { 0x0063, 0x0061, 0x0062, 0x0000 },
	.va_ve_cfg_values = (u32 []){ 0x8013FFFF, 0x80340002 },
	.num_of_va_ve_cfg_values = 2,
	.audio_routing_config = { 0x0210, 0x1eee, 0x2ee3, 0x3eee,
				  0xeeee, 0xeeee, 0xeeee, 0xeeee },
	.tdm_configs = {
		/* DBMD2 TDM3_TX is connected to the HOST */
		{	.tdm_index	= 0x3,
			.tdm_type	= TDM_TYPE_TX,
			.tdm_reg_config	= 0x313,
			.tdm_interface	= TDM_INTERFACE_VA_VE,
			.num_of_io_reg_configs = 3,
			.io_reg_configs	= {
				{		.addr = DBMD2_TDM_3_TX_ADDR,
						.value =
						(0x00800000 | HW_TDM_POLARITY),
				},
				{		.addr = DBMD2_TDM_3_TX_ADDR + 4,
						.value = 0x00072007
				},
				{		.addr = DBMD2_TDM_3_TX_ADDR + 6,
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
			TDM3_TX (D2==>HOST)
	*/
	.va_ve_start_cmd = 0x8008,
};


/* Orange UC2 - AECNR Triple Mic 48Khz */
static struct usecase_config config_uc_orange_aec_tripple_mic_48k = {
	.usecase_name = "uc_orange_aec_tripple_mic_48k",
	.id	= (DBMDX_USECASE_ID_UC_IDX_02 |
			DBMDX_USECASE_ID_PRJ_ORANGE |
			DBMDX_USECASE_ID_HWREV_01),
	.hw_rev = 1,
	.send_va_asrp_parms = false,
	.va_asrp_params_file_name = NULL,
	.send_va_ve_asrp_parms = true,
	.va_ve_asrp_params_file_name = "asrp_params_aecnr_48khz.bin",
	.change_clock_src = true,
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
	.config_mics = MIC_CONFIG_BY_USECASE,
	.mic_config = { 0x0063, 0x0061, 0x0062, 0x0000 },
	.va_ve_cfg_values = (u32 []){ 0x8013FFFF, 0x80340002 },
	.num_of_va_ve_cfg_values = 2,
	.audio_routing_config = { 0x0210, 0x154e, 0x2e03, 0x3eee,
				  0xeeee, 0xeeee, 0xeeee, 0xeeee },
	.tdm_configs = {
		/* DBMD2 TDM3_RX is connected to the HOST */
		{	.tdm_index	= 0x3,
			.tdm_type	= TDM_TYPE_RX,
			.tdm_reg_config	= 0x9A14,
			.tdm_interface	= TDM_INTERFACE_VA_VE,
			.num_of_io_reg_configs = 3,
			.io_reg_configs	= {
				{		.addr = DBMD2_TDM_3_RX_ADDR,
						.value =
						(0x00800000 | HW_TDM_POLARITY),
				},
				{		.addr = DBMD2_TDM_3_RX_ADDR + 4,
						.value = 0x00000027
				},
				{		.addr = DBMD2_TDM_3_RX_ADDR + 6,
						.value = 0x100F001F
				}
			},
		},
		/* DBMD2 TDM3_TX is connected to the HOST */
		{	.tdm_index	= 0x3,
			.tdm_type	= TDM_TYPE_TX,
			.tdm_reg_config	= 0x0BD0,
			.tdm_interface	= TDM_INTERFACE_VA_VE,
			.num_of_io_reg_configs = 3,
			.io_reg_configs	= {
				{		.addr = DBMD2_TDM_3_TX_ADDR,
						.value =
						(0x00800000 | HW_TDM_POLARITY),
				},
				{		.addr = DBMD2_TDM_3_TX_ADDR + 4,
						.value = 0x00070027
				},
				{		.addr = DBMD2_TDM_3_TX_ADDR + 6,
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
		TDM3_RX (HOST==>D2) (reference)
		TDM3_TX (D2==>HOST)
	*/
	.va_ve_start_cmd = 0xC008,
};


/* Orange UC2 16 KHz - AECNR Triple Mic 16 KHz */
static struct usecase_config config_uc_orange_aec_tripple_mic_16k = {
	.usecase_name = "uc_orange_aec_tripple_mic_16k",
	.id	= (DBMDX_USECASE_ID_UC_IDX_03 |
			DBMDX_USECASE_ID_PRJ_ORANGE |
			DBMDX_USECASE_ID_HWREV_01),
	.hw_rev = 1,
	.send_va_asrp_parms = false,
	.va_asrp_params_file_name = NULL,
	.send_va_ve_asrp_parms = true,
	.va_ve_asrp_params_file_name = "asrp_params_aecnr.bin",
	.change_clock_src = true,
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
	.config_mics = MIC_CONFIG_BY_USECASE,
	.mic_config = { 0x0063, 0x0061, 0x0062, 0x0000 },
	.va_ve_cfg_values = (u32 []){ 0x8013FFFF, 0x80340002 },
	.num_of_va_ve_cfg_values = 2,
	.audio_routing_config = { 0x0210, 0x154e, 0x2e03, 0x3eee,
				  0xeeee, 0xeeee, 0xeeee, 0xeeee },
	.tdm_configs = {
		/* DBMD2 TDM3_RX is connected to the HOST */
		{	.tdm_index	= 0x3,
			.tdm_type	= TDM_TYPE_RX,
			.tdm_reg_config	= 0x9214,
			.tdm_interface	= TDM_INTERFACE_VA_VE,
			.num_of_io_reg_configs = 3,
			.io_reg_configs	= {
				{		.addr = DBMD2_TDM_3_RX_ADDR,
						.value =
						(0x00800000 | HW_TDM_POLARITY),
				},
				{		.addr = DBMD2_TDM_3_RX_ADDR + 4,
						.value = 0x00000027
				},
				{		.addr = DBMD2_TDM_3_RX_ADDR + 6,
						.value = 0x100F001F
				}
			},
		},
		/* DBMD2 TDM3_TX is connected to the HOST */
		{	.tdm_index	= 0x3,
			.tdm_type	= TDM_TYPE_TX,
			.tdm_reg_config	= 0x0313,
			.tdm_interface	= TDM_INTERFACE_VA_VE,
			.num_of_io_reg_configs = 3,
			.io_reg_configs	= {
				{		.addr = DBMD2_TDM_3_TX_ADDR,
						.value =
						(0x00800000 | HW_TDM_POLARITY),
				},
				{		.addr = DBMD2_TDM_3_TX_ADDR + 4,
						.value = 0x00070027
				},
				{		.addr = DBMD2_TDM_3_TX_ADDR + 6,
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
		TDM3_RX (HOST==>D2) (reference)
		TDM3_TX (D2==>HOST)
	*/
	.va_ve_start_cmd = 0xC008,
};


/* Orange UC - Voice Call Triple Mic */
static struct usecase_config config_uc_orange_voice_call_tripple_mic = {
	.usecase_name = "uc_orange_voice_call_tripple_mic",
	.id	= (DBMDX_USECASE_ID_UC_IDX_04 |
			DBMDX_USECASE_ID_PRJ_ORANGE |
			DBMDX_USECASE_ID_HWREV_01),
	.hw_rev = 1,
	.send_va_asrp_parms = false,
	.va_asrp_params_file_name = NULL,
	.send_va_ve_asrp_parms = true,
	.va_ve_asrp_params_file_name = "asrp_params_voice_call.bin",
	.change_clock_src = true,
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
	.config_mics = MIC_CONFIG_BY_USECASE,
	.mic_config = { 0x0063, 0x0061, 0x0062, 0x0000 },
	.va_ve_cfg_values = (u32 []){ 0x8013FFFF, 0x80340002 },
	.num_of_va_ve_cfg_values = 2,
	.audio_routing_config = { 0x0210, 0x1e4e, 0x2ee3, 0x3eee,
				  0xeeee, 0xeeee, 0xeeee, 0xeeee },
	.tdm_configs = {
		/* DBMD2 TDM3_RX is connected to the HOST */
		{	.tdm_index	= 0x3,
			.tdm_type	= TDM_TYPE_RX,
			.tdm_reg_config	= 0x9214,
			.tdm_interface	= TDM_INTERFACE_VA_VE,
			.num_of_io_reg_configs = 3,
			.io_reg_configs	= {
				{		.addr = DBMD2_TDM_3_RX_ADDR,
						.value =
						(0x00800000 | HW_TDM_POLARITY),
				},
				{		.addr = DBMD2_TDM_3_RX_ADDR + 4,
						.value = 0x00000027
				},
				{		.addr = DBMD2_TDM_3_RX_ADDR + 6,
						.value = 0x100F001F
				}
			},
		},
		/* DBMD2 TDM3_TX is connected to the HOST */
		{	.tdm_index	= 0x3,
			.tdm_type	= TDM_TYPE_TX,
			.tdm_reg_config	= 0x0313,
			.tdm_interface	= TDM_INTERFACE_VA_VE,
			.num_of_io_reg_configs = 3,
			.io_reg_configs	= {
				{		.addr = DBMD2_TDM_3_TX_ADDR,
						.value =
						(0x00800000 | HW_TDM_POLARITY),
				},
				{		.addr = DBMD2_TDM_3_TX_ADDR + 4,
						.value = 0x00070027
				},
				{		.addr = DBMD2_TDM_3_TX_ADDR + 6,
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
		TDM3_RX (HOST==>D2) (reference)
		TDM3_TX (D2==>HOST)
	*/
	.va_ve_start_cmd = 0xC008,
};


/* Orange PDM To PCM */
static struct usecase_config config_uc_orange_pdm_to_pcm_mono = {
	.usecase_name = "uc_orange_pdm_to_pcm_mono",
	.id	= (DBMDX_USECASE_ID_UC_IDX_05 |
			DBMDX_USECASE_ID_PRJ_ORANGE |
			DBMDX_USECASE_ID_HWREV_01),
	.hw_rev = 1,
	.send_va_asrp_parms = false,
	.va_asrp_params_file_name = NULL,
	.send_va_ve_asrp_parms = false,
	.va_ve_asrp_params_file_name = NULL,
	.change_clock_src = true,
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
	.config_mics = MIC_CONFIG_BY_USER_MASK,
	.mic_config = { 0x0063, 0x0061, 0x0062, 0x0000 },
	.va_ve_cfg_values = (u32 []){ 0x80230200, 0x80340000, 0x8013FFFF },
	.num_of_va_ve_cfg_values = 3,
	.audio_routing_config = { 0xeeee, 0xeeee, 0xeeee, 0xeeee,
				  0xeeee, 0xeeee, 0xeeee, 0xeeee },
	.tdm_configs = {
		/* DBMD2 TDM3_TX is connected to the HOST */
		{	.tdm_index	= 0x3,
			.tdm_type	= TDM_TYPE_TX,
			.tdm_reg_config	= 0x0B10,
			.tdm_interface	= TDM_INTERFACE_VA_VE,
			.num_of_io_reg_configs = 3,
			.io_reg_configs	= {
				{		.addr = DBMD2_TDM_3_TX_ADDR,
						.value =
						(0x00800000 | HW_TDM_POLARITY),
				},
				{		.addr = DBMD2_TDM_3_TX_ADDR + 4,
						.value = 0x00070027
				},
				{		.addr = DBMD2_TDM_3_TX_ADDR + 6,
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
			TDM3_TX (D2==>HOST)
	*/
	.va_ve_start_cmd = 0x8008,
};

static struct usecase_config config_uc_orange_pdm_to_pcm_stereo = {
	.usecase_name = "uc_orange_pdm_to_pcm_stereo",
	.id	= (DBMDX_USECASE_ID_UC_IDX_06 |
			DBMDX_USECASE_ID_PRJ_ORANGE |
			DBMDX_USECASE_ID_HWREV_01),
	.hw_rev = 1,
	.send_va_asrp_parms = false,
	.va_asrp_params_file_name = NULL,
	.send_va_ve_asrp_parms = false,
	.va_ve_asrp_params_file_name = NULL,
	.change_clock_src = true,
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
	.config_mics = MIC_CONFIG_BY_USER_MASK,
	.mic_config = { 0x0063, 0x0061, 0x0062, 0x0000 },
	.va_ve_cfg_values = (u32 []){ 0x80230200, 0x80340000, 0x8013FFFF },
	.num_of_va_ve_cfg_values = 3,
	.audio_routing_config = { 0xeeee, 0xeeee, 0xeeee, 0xeeee,
				  0xeeee, 0xeeee, 0xeeee, 0xeeee },
	.tdm_configs = {
		/* DBMD2 TDM3_TX is connected to the HOST */
		{	.tdm_index	= 0x3,
			.tdm_type	= TDM_TYPE_TX,
			.tdm_reg_config	= 0x9A10,
			.tdm_interface	= TDM_INTERFACE_VA_VE,
			.num_of_io_reg_configs = 3,
			.io_reg_configs	= {
				{		.addr = DBMD2_TDM_3_TX_ADDR,
						.value =
						(0x00800000 | HW_TDM_POLARITY),
				},
				{		.addr = DBMD2_TDM_3_TX_ADDR + 4,
						.value = 0x00070027
				},
				{		.addr = DBMD2_TDM_3_TX_ADDR + 6,
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
			TDM3_TX (D2==>HOST)
	*/
	.va_ve_start_cmd = 0x8008,
};

/* Orange TDM 3 Loop R */
static struct usecase_config config_uc_orange_tdm_loop_r = {
	.usecase_name = "uc_orange_tdm_loop_r",
	.id	= (DBMDX_USECASE_ID_UC_IDX_07 |
			DBMDX_USECASE_ID_PRJ_ORANGE |
			DBMDX_USECASE_ID_HWREV_01),
	.hw_rev = 1,
	.send_va_asrp_parms = false,
	.va_asrp_params_file_name = NULL,
	.send_va_ve_asrp_parms = false,
	.va_ve_asrp_params_file_name = NULL,
	.change_clock_src = true,
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
	.config_mics = DO_NOT_CONFIG_MICS,
	.mic_config = { 0x0000, 0x0000, 0x0000, 0x0000 },
	.va_ve_cfg_values = (u32 []){ 0x8013FFFF, 0x80340002 },
	.num_of_va_ve_cfg_values = 2,
	.audio_routing_config = { 0x0210, 0x154e, 0x2e03, 0x3eee,
				  0xeeee, 0xeeee, 0xeeee, 0xeeee },
	.tdm_configs = {
		/* DBMD2 TDM3_RX is connected to the HOST */
		{	.tdm_index	= 0x3,
			.tdm_type	= TDM_TYPE_RX,
			.tdm_reg_config	= 0x9A14,
			.tdm_interface	= TDM_INTERFACE_VA_VE,
			.num_of_io_reg_configs = 3,
			.io_reg_configs	= {
				{		.addr = DBMD2_TDM_3_RX_ADDR,
						.value =
						(0x00800000 | HW_TDM_POLARITY),
				},
				{		.addr = DBMD2_TDM_3_RX_ADDR + 4,
						.value = 0x00000027
				},
				{		.addr = DBMD2_TDM_3_RX_ADDR + 6,
						.value = 0x100F001F
				}
			},
		},
		/* DBMD2 TDM3_TX is connected to the HOST */
		{	.tdm_index	= 0x3,
			.tdm_type	= TDM_TYPE_TX,
			.tdm_reg_config	= 0x0B15,
			.tdm_interface	= TDM_INTERFACE_VA_VE,
			.num_of_io_reg_configs = 3,
			.io_reg_configs	= {
				{		.addr = DBMD2_TDM_3_TX_ADDR,
						.value =
						(0x00800000 | HW_TDM_POLARITY),
				},
				{		.addr = DBMD2_TDM_3_TX_ADDR + 4,
						.value = 0x00070027
				},
				{		.addr = DBMD2_TDM_3_TX_ADDR + 6,
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
		TDM3_RX (HOST==>D2) (reference)
		TDM3_TX (D2==>HOST)
	*/
	.va_ve_start_cmd = 0xC008,
};

/* Orange TDM 3 Loop L */
static struct usecase_config config_uc_orange_tdm_loop_l = {
	.usecase_name = "uc_orange_tdm_loop_l",
	.id	= (DBMDX_USECASE_ID_UC_IDX_08 |
			DBMDX_USECASE_ID_PRJ_ORANGE |
			DBMDX_USECASE_ID_HWREV_01),
	.hw_rev = 1,
	.send_va_asrp_parms = false,
	.va_asrp_params_file_name = NULL,
	.send_va_ve_asrp_parms = false,
	.va_ve_asrp_params_file_name = NULL,
	.change_clock_src = true,
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
	.config_mics = DO_NOT_CONFIG_MICS,
	.mic_config = { 0x0000, 0x0000, 0x0000, 0x0000 },
	.va_ve_cfg_values = (u32 []){ 0x8013FFFF, 0x80340002 },
	.num_of_va_ve_cfg_values = 2,
	.audio_routing_config = { 0x0210, 0x154e, 0x2e03, 0x3eee,
				  0xeeee, 0xeeee, 0xeeee, 0xeeee },
	.tdm_configs = {
		/* DBMD2 TDM3_RX is connected to the HOST */
		{	.tdm_index	= 0x3,
			.tdm_type	= TDM_TYPE_RX,
			.tdm_reg_config	= 0x9A14,
			.tdm_interface	= TDM_INTERFACE_VA_VE,
			.num_of_io_reg_configs = 3,
			.io_reg_configs	= {
				{		.addr = DBMD2_TDM_3_RX_ADDR,
						.value =
						(0x00800000 | HW_TDM_POLARITY),
				},
				{		.addr = DBMD2_TDM_3_RX_ADDR + 4,
						.value = 0x00000027
				},
				{		.addr = DBMD2_TDM_3_RX_ADDR + 6,
						.value = 0x100F001F
				}
			},
		},
		/* DBMD2 TDM3_TX is connected to the HOST */
		{	.tdm_index	= 0x3,
			.tdm_type	= TDM_TYPE_TX,
			.tdm_reg_config	= 0x0B14,
			.tdm_interface	= TDM_INTERFACE_VA_VE,
			.num_of_io_reg_configs = 3,
			.io_reg_configs	= {
				{		.addr = DBMD2_TDM_3_TX_ADDR,
						.value =
						(0x00800000 | HW_TDM_POLARITY),
				},
				{		.addr = DBMD2_TDM_3_TX_ADDR + 4,
						.value = 0x00070027
				},
				{		.addr = DBMD2_TDM_3_TX_ADDR + 6,
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
		TDM3_RX (HOST==>D2) (reference)
		TDM3_TX (D2==>HOST)
	*/
	.va_ve_start_cmd = 0xC008,
};
#endif /* DBMDX_SUPPORT_OBSOLETE_ORANGE_USECASES */

#endif /* _DBMDX_USECASE_CONFIG_ORANGE_H */

#endif /* DBMDX_ORANGE_USECASES_SUPPORTED */
