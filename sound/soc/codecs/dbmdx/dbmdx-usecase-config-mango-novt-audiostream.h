/*
 * dbmdx-usecase-config-mango-novt-audiostream.h  --
 * Preset USE CASE configurations for mango configuration without VT
 * and audio streamed to host over TDM
 * Copyright (C) 2014 DSP Group
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifdef DBMDX_MANGO_NOVT_AUDIOSTREAM_USECASES_SUPPORTED

#ifndef _DBMDX_USECASE_CONFIG_MANGO_NOVT_AUDIOSTREAM_H
#define _DBMDX_USECASE_CONFIG_MANGO_NOVT_AUDIOSTREAM_H

#include "dbmdx-interface.h"
#include "dbmdx-usecase-config-def.h"

/* Mango Barge In usecase 48Khz 16 Bit or 16 from 32 Bit with D2 I2S stream  */
static struct usecase_config
	config_uc_mango_barge_in_48k_16b_32b_novt_audiostream = {
#ifdef AEC_REF_32_TO_16_BIT
	.usecase_name = "uc_mango_barge_in_48k_32to16b_novt_audiostream",
#else
	.usecase_name = "uc_mango_barge_in_48k_16b_novt_audiostream",
#endif
	.id	= (DBMDX_USECASE_ID_UC_IDX_05 |
			DBMDX_USECASE_ID_PRJ_MANGO |
			DBMDX_USECASE_ID_HWREV_03),
	.hw_rev = 3,
	.send_va_asrp_parms = false,
	.va_asrp_params_file_name = "asrp_params_qed.bin",
	.send_va_ve_asrp_parms = true,
	.va_ve_asrp_params_file_name = "asrp_params_aecnr.bin",
	.change_clock_src = true,
	.tdm_clock_freq = TDM_CLK_FREQ_48,
#ifdef AEC_REF_32_TO_16_BIT
	.number_of_bits = 32,
#else
	.number_of_bits = 16,
#endif
	.usecase_requires_amodel = false,
	.usecase_amodel_mode = 1,
	.usecase_sets_detection_mode = false,
	.usecase_supports_us_buffering = true,
	.va_chip_low_power_mode = false,
	.va_ve_chip_low_power_mode = false,
		/*	.va_cfg_values = (u32 []){ 0x80230000,
		 *				0x8013ffff,
		 *				0x80340000,
		 *				0x8aaa1500 },
		 */
	.va_cfg_values = (u32 []){
		(DBMDX_REGN_GENERAL_CONFIG_2 |
			DBMDX_REGV_MIC_SAMPLE_RATE_16K |
			DBMDX_REGV_FW_VAD_TYPE_NO_VAD),
		(DBMDX_REGN_AUDIO_STREAMING_SRC_SELECT |
			DBMDX_REGV_NO_STREAM_CH_4 |
			DBMDX_REGV_NO_STREAM_CH_3 |
			DBMDX_REGV_NO_STREAM_CH_2 |
			DBMDX_REGV_NO_STREAM_CH_1),
		(DBMDX_REGN_AUDIO_PROCESSING_CONFIG |
			DBMDX_REGV_VT_CP0 |
			DBMDX_REGV_POST_DET_MODE_SWITCH_TO_STREAMING),
		(DBMDX_VA_USLEEP | 0x1500) },

	.num_of_va_cfg_values = 4,
	.config_mics = MIC_CONFIG_BY_USECASE,
	/* .mic_config = { 0x5496, 0xa481, 0xa493, 0x0000 }, */
	.mic_config = {
		(DBMDX_REGV_DM_CLOCK_SRC_DM0_GPIO6_DM1_GPIO12 |
		 DBMDX_REGV_DM_DATA_SRC_DM0_GPIO5_DM1_GPIO11 |
		 DBMDX_REGV_DDF_AUDIO_ATTENUATION_MINUS_6dB |
		 DBMDX_REGV_DM_CLK_FREQ_2304_2352_SR_16KHz_32KHz_48KHz |
		 DBMDX_REGV_CLOCK_POLARITY_FALLING_EDGE |
		 DBMDX_REGV_DDF_AND_DM_CONFIG_SD_DDF_DM1),
		(DBMDX_REGV_DM_CLOCK_SRC_DM0_GPIO8_DM1_GPIO14 |
		 DBMDX_REGV_DM_DATA_SRC_DM0_GPIO9_DM1_GPIO13 |
		 DBMDX_REGV_DDF_AUDIO_ATTENUATION_MINUS_6dB |
		 DBMDX_REGV_DM_CLK_FREQ_2304_2352_SR_16KHz_32KHz_48KHz |
		 DBMDX_REGV_DDF_AND_DM_CONFIG_DDF0_DM0),
		(DBMDX_REGV_DM_CLOCK_SRC_DM0_GPIO8_DM1_GPIO14 |
		 DBMDX_REGV_DM_DATA_SRC_DM0_GPIO9_DM1_GPIO13 |
		 DBMDX_REGV_DDF_AUDIO_ATTENUATION_MINUS_6dB |
		 DBMDX_REGV_DM_CLK_FREQ_2304_2352_SR_16KHz_32KHz_48KHz |
		 DBMDX_REGV_CLOCK_POLARITY_FALLING_EDGE |
		 DBMDX_REGV_DDF_AND_DM_CONFIG_DDF0_DM1 |
		 DBMDX_REGV_DDF_AND_DM_CONFIG_DDF0_DM0),
		0x0000},
		/* .va_ve_cfg_values = (u32 []){ 0x80230000,
		 *				0x801F0300,
		 *				0x80344002,
		 *				0x8aaa1500 },
		 */
	.va_ve_cfg_values = (u32 []){
		(DBMDX_REGN_GENERAL_CONFIG_2 |
			DBMDX_REGV_MIC_SAMPLE_RATE_16K |
			DBMDX_REGV_FW_VAD_TYPE_NO_VAD),
		(DBMDX_REGN_AUDIO_ROUTING_CONFIG |
			DBMDX_REGV_MUSIC_IN_TDM3),
		(DBMDX_REGN_AUDIO_PROCESSING_CONFIG |
			DBMDX_REGV_EN_48_EC_REF_DECIM |
			DBMDX_REGV_ALGO1_EN_FW_MODE_2_ONLY),
		(DBMDX_VA_USLEEP | 0x1500) },
	.num_of_va_ve_cfg_values = 4,
	/*.va_post_tdm_cfg_values = (u32 []){0x801F7400 },*/
	.va_ve_post_tdm_cfg_values = (u32 []){
		(DBMDX_REGN_AUDIO_ROUTING_CONFIG |
			DBMDX_REGV_TDM_SYNC_RIGHT_CH |
			DBMDX_REGV_USE_TDM_MUSIC_TO_SYNC |
			DBMDX_REGV_MUSIC_IN_TDM3 |
			DBMDX_REGV_TDM_SYNC_DELAY_4_CLKS_CYCLES) },
	.num_of_va_ve_post_tdm_cfg_values = 1,

	/*	.audio_routing_config = { 0x0210, 0x154e, 0x2ee3, 0x3eee,
	 *				 0xeeee, 0xeeee, 0xeeee, 0xeeee },
	 */
	.audio_routing_config = {
		(DBMDX_REGV_IO_SET_0 |
			DBMDX_REGV_IO_3N_2_CP_2 |
			DBMDX_REGV_IO_3N_1_CP_1 |
			DBMDX_REGV_IO_3N_0_CP_0),
		(DBMDX_REGV_IO_SET_1 |
			DBMDX_REGV_IO_3N_2_CP_5 |
			DBMDX_REGV_IO_3N_1_CP_4 |
			DBMDX_REGV_IO_3N_0_NO_CP),
		(DBMDX_REGV_IO_SET_2 |
			DBMDX_REGV_IO_3N_2_NO_CP |
			DBMDX_REGV_IO_3N_1_NO_CP |
			DBMDX_REGV_IO_3N_0_CP_1),
		(DBMDX_REGV_IO_SET_3 |
			DBMDX_REGV_IO_3N_2_NO_CP |
			DBMDX_REGV_IO_3N_1_NO_CP |
			DBMDX_REGV_IO_3N_0_NO_CP),
		DBMDX_UNDEFINED_REGISTER,
		DBMDX_UNDEFINED_REGISTER,
		DBMDX_UNDEFINED_REGISTER,
		DBMDX_UNDEFINED_REGISTER },

	.tdm_configs = {
		/* DBMD2 TDM3_RX is connected to the Host Codec */
		{	.tdm_index	= 0x3,
			.tdm_type	= TDM_TYPE_RX,
			/* .tdm_reg_config	= 0x9a14, */
			.tdm_reg_config	=
				(DBMDX_REGV_DEMUX_MUX_ENABLE |
				DBMDX_REGV_SAMPLE_WIDTH_16_BIT |
				DBMDX_REGV_NUM_OF_CHANNELS_2_CH |
				DBMDX_REGV_INPUT_OUTPUT_SAMPLE_RATE_48_KHz |
				DBMDX_REGV_RX_TX_I2S_CH_USE_I2S_STEREO |
				DBMDX_REGV_RESAMPLE_TYPE_DECIMATION |
				DBMDX_REGV_RESAMPLE_RATIO_NO_RESAMPLING |
				DBMDX_REGV_HW_BLOCK_EN |
				DBMDX_REGV_RX_TX_CP4),
			/* .tdm_reg_config	= 0xda14,	32bit */
			.tdm_interface	= TDM_INTERFACE_VA_VE,
#ifdef AEC_REF_32_TO_16_BIT
			.num_of_io_reg_configs = 4,
			.io_reg_configs	= {
				{		.addr = DBMD2_TDM_3_RX_ADDR,
						.value = 0x0080405D
					/* (0x00804050 | HW_TDM_POLARITY), */
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
						.value =
						(0x00800010 | HW_TDM_POLARITY),
				},
				{		.addr = DBMD2_TDM_3_RX_ADDR + 4,
						.value = 0x00000007
				},
				{		.addr = DBMD2_TDM_3_RX_ADDR + 6,
						.value = 0x100F001F
						/*.value = 0x101F003F 32bit */
				}
			},
#endif /* AEC_REF_32_TO_16_BIT */
		},
		/* DBMD2 TDM3_TX is connected to Host Codec */
		{	.tdm_index	= 0x3,
			.tdm_type	= TDM_TYPE_TX,
			/* .tdm_reg_config	= 0x9ad0, */
			.tdm_reg_config	=
			(DBMDX_REGV_DEMUX_MUX_ENABLE |
			 DBMDX_REGV_SAMPLE_WIDTH_16_BIT |
			 DBMDX_REGV_NUM_OF_CHANNELS_2_CH |
			 DBMDX_REGV_INPUT_OUTPUT_SAMPLE_RATE_48_KHz |
			 DBMDX_REGV_RX_TX_I2S_CH_USE_I2S_STEREO |
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
						.value = 0x00642064
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
		/* DBMD4 TDM1_TX is connected to DBMD2 TDM#_RX */
		{	.tdm_index	= 0x1,
			.tdm_type	= TDM_TYPE_TX,
			/* .tdm_reg_config	= 0xa210, */
			.tdm_reg_config	=
				(DBMDX_REGV_DEMUX_MUX_ENABLE |
				DBMDX_REGV_SAMPLE_WIDTH_16_BIT |
				DBMDX_REGV_NUM_OF_CHANNELS_4_CH |
				DBMDX_REGV_INPUT_OUTPUT_SAMPLE_RATE_16_KHz |
				DBMDX_REGV_RX_TX_I2S_CH_USE_I2S_STEREO |
				DBMDX_REGV_RESAMPLE_RATIO_NO_RESAMPLING |
				DBMDX_REGV_HW_BLOCK_EN |
				DBMDX_REGV_RX_TX_CP0),
			.tdm_interface	= TDM_INTERFACE_VA,
			.num_of_io_reg_configs = 3,
			.io_reg_configs	= {
				{		.addr = DBMD4_TDM_1_TX_ADDR,
						.value = 0x00800010
				},
				{		.addr = DBMD4_TDM_1_TX_ADDR + 4,
						.value = 0x00073007
				},
				{		.addr = DBMD4_TDM_1_TX_ADDR + 6,
						.value = 0x101F003F
				}
			},
		},
		/* DBMD2 TDM0_RX is connected to DBMD4 TDM1_TX */
		{	.tdm_index	= 0x0,
			.tdm_type	= TDM_TYPE_RX,
			/* .tdm_reg_config	= 0xa210, */
			.tdm_reg_config	=
				(DBMDX_REGV_DEMUX_MUX_ENABLE |
				DBMDX_REGV_SAMPLE_WIDTH_16_BIT |
				DBMDX_REGV_NUM_OF_CHANNELS_4_CH |
				DBMDX_REGV_INPUT_OUTPUT_SAMPLE_RATE_16_KHz |
				DBMDX_REGV_RX_TX_I2S_CH_USE_I2S_STEREO |
				DBMDX_REGV_RESAMPLE_RATIO_NO_RESAMPLING |
				DBMDX_REGV_HW_BLOCK_EN |
				DBMDX_REGV_RX_TX_CP0),
			.tdm_interface	= TDM_INTERFACE_VA_VE,
			.num_of_io_reg_configs = 3,
			.io_reg_configs	= {
				{		.addr = DBMD2_TDM_0_RX_ADDR,
						.value = 0x00800015
				},
				{		.addr = DBMD2_TDM_0_RX_ADDR + 4,
						.value = 0x00073007
				},
				{		.addr = DBMD2_TDM_0_RX_ADDR + 6,
						.value = 0x101F003F
				}
			},
		},
	},
	.num_of_tdm_configs = 4,
	.va_start_cmd_type = START_CMD_TYPE_TDM,
	.send_va_start_cmd = true,
	/* Enable:
			TDM1_TX (D4<=>D2)
	*/
	/* .va_start_cmd = 0x0808, */
	.va_start_cmd = (DBMDX_REGV_TDM1_TX_EN |
			DBMDX_REGV_PROC_EN_SWITCH_FW_TO_BUFFERING_MODE),
	.va_ve_start_cmd_type = START_CMD_TYPE_TDM,
	.send_va_ve_start_cmd = true,
	/* Enable:
			TDM3_TX (D2=>CODEC)
			TDM0_RX (D4<=>D2)
			TDM3_RX (codec=>D2)
	*/
	/* .va_ve_start_cmd = 0xC108, */
	.va_ve_start_cmd = (DBMDX_REGV_TDM3_RX_EN |
				DBMDX_REGV_TDM3_TX_EN |
				DBMDX_REGV_TDM0_RX_EN |
				DBMDX_REGV_PROC_EN_SWITCH_FW_TO_BUFFERING_MODE),
};

#endif /* _DBMDX_USECASE_CONFIG_MANGO_NOVT_AUDIOSTREAM_H */

#endif /* DBMDX_MANGO_NOVT_AUDIOSTREAM_USECASES_SUPPORTED */
