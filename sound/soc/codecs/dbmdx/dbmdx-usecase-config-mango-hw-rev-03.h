/*
 * dbmdx-usecase-config-mango-hw-rev-03.h  --  Preset USE CASE configurations
 *
 * Copyright (C) 2014 DSP Group
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifdef DBMDX_MANGO_USECASES_HW_REV_03_SUPPORTED

#ifndef _DBMDX_USECASE_CONFIG_MANGO_HW_REV_03_H
#define _DBMDX_USECASE_CONFIG_MANGO_HW_REV_03_H

#include "dbmdx-interface.h"
#include "dbmdx-usecase-config-def.h"

/* Mango NR Usecase*/
static struct usecase_config config_uc_mango_low_power_hw_rev_03 = {
	.usecase_name = "uc_mango_low_power",
	.id	= (DBMDX_USECASE_ID_UC_IDX_00 |
			DBMDX_USECASE_ID_PRJ_MANGO |
			DBMDX_USECASE_ID_HWREV_03),
	.hw_rev = 3,
	.send_va_asrp_parms = false,
	.va_asrp_params_file_name = "",
	.send_va_ve_asrp_parms = false,
	.va_ve_asrp_params_file_name = "",
	.change_clock_src = false,
	.tdm_clock_freq = TDM_CLK_FREQ_48,
	.number_of_bits = 16,
	.usecase_requires_amodel = true,
	.usecase_amodel_mode = 1,
	.usecase_sets_detection_mode = true,
	.usecase_supports_us_buffering = true,
	.va_chip_low_power_mode = false,
	.va_ve_chip_low_power_mode = true,
	.asrp_output_gain_type = (ASRP_TX_OUT_GAIN |
					ASRP_VCPF_OUT_GAIN |
					ASRP_RX_OUT_GAIN),

	.va_cfg_values = (u32 []){
		(DBMDX_REGN_GENERAL_CONFIG_2 |
			DBMDX_REGV_MIC_SAMPLE_RATE_16K),
		(DBMDX_REGN_AUDIO_PROCESSING_CONFIG |
			DBMDX_REGV_POST_DET_MODE_SWITCH_TO_STREAMING |
			DBMDX_REGV_VT_CP0 |
			DBMDX_REGV_VT_PROC_EN),
		(DBMDX_VA_USLEEP | 0x1500),
		(DBMDX_REGN_AUDIO_STREAMING_SRC_SELECT |
			DBMDX_REGV_NO_STREAM_CH_4 |
			DBMDX_REGV_NO_STREAM_CH_3 |
			DBMDX_REGV_NO_STREAM_CH_2 |
			DBMDX_REGV_STREAM_CH_1_CP_0)},
	.num_of_va_cfg_values = 4,

	.config_mics = MIC_CONFIG_BY_USECASE,
	.mic_config = {
		(DBMDX_REGV_DM_CLOCK_SRC_DM0_GPIO6_DM1_GPIO12 |
		 DBMDX_REGV_DM_DATA_SRC_DM0_GPIO5_DM1_GPIO11 |
		 DBMDX_REGV_DDF_AUDIO_ATTENUATION_6dB_GAIN |
		 DBMDX_REGV_DM_CLK_FREQ_1024_1040_SR_16KHz_32KHz_48KHz |
		 DBMDX_REGV_CLOCK_POLARITY_FALLING_EDGE |
		 DBMDX_REGV_DDF_AND_DM_CONFIG_SD_DDF_DM1),
		0x0000,
		0x0000,
		0x0000},

	.num_of_va_ve_cfg_values = 0,

	.audio_routing_config = {
		DBMDX_UNDEFINED_REGISTER,
		DBMDX_UNDEFINED_REGISTER,
		DBMDX_UNDEFINED_REGISTER,
		DBMDX_UNDEFINED_REGISTER,
		DBMDX_UNDEFINED_REGISTER,
		DBMDX_UNDEFINED_REGISTER,
		DBMDX_UNDEFINED_REGISTER,
		DBMDX_UNDEFINED_REGISTER,
		DBMDX_UNDEFINED_REGISTER,},

	.num_of_tdm_configs = 0,
	.va_start_cmd_type = START_CMD_TYPE_OPMODE,
	.send_va_start_cmd = true,
	.va_start_cmd = (DBMDX_DETECTION),

	.va_ve_start_cmd_type = START_CMD_TYPE_OPMODE,
	.send_va_ve_start_cmd = true,
	.va_ve_start_cmd = (DBMDX_IDLE),
};




/* Mango NR Usecase*/
static struct usecase_config config_uc_mango_nr_hw_rev_03 = {
	.usecase_name = "uc_mango_nr",
	.id	= (DBMDX_USECASE_ID_UC_IDX_01 |
			DBMDX_USECASE_ID_PRJ_MANGO |
			DBMDX_USECASE_ID_HWREV_03),
	.hw_rev = 3,
#ifdef DBMDX_QED_SUPPORTED
	.send_va_asrp_parms = true,
#else
	.send_va_asrp_parms = false,
#endif
	.va_asrp_params_file_name = "asrp_params_qed.bin",
	.send_va_ve_asrp_parms = true,
	.va_ve_asrp_params_file_name = "asrp_params_nr.bin",
#if defined(DBMDX_I2S_STREAMING_SUPPORTED) ||\
			defined(DBMDX_I2S_BUFFERING_SUPPORTED)
	.change_clock_src = true,
#else
	.change_clock_src = false,
#endif
	.tdm_clock_freq = TDM_CLK_FREQ_48,
#ifdef AEC_REF_32_TO_16_BIT
	.number_of_bits = 32,
#else
	.number_of_bits = 16,
#endif
#ifdef DBMDX_I2S_STREAMING_SUPPORTED
	.usecase_requires_amodel = false,
	.usecase_amodel_mode = 1,
	.usecase_sets_detection_mode = false,
	.usecase_supports_us_buffering = false,
#else
	.usecase_requires_amodel = true,
	.usecase_amodel_mode = 1,
	.usecase_sets_detection_mode = true,
	.usecase_supports_us_buffering = true,
#endif
	.va_chip_low_power_mode = false,
	.va_ve_chip_low_power_mode = false,
	.asrp_output_gain_type = (ASRP_TX_OUT_GAIN |
					ASRP_VCPF_OUT_GAIN |
					ASRP_RX_OUT_GAIN),

	.va_cfg_values = (u32 []){
		(DBMDX_REGN_GENERAL_CONFIG_2 |
			DBMDX_REGV_MIC_SAMPLE_RATE_16K |
			DBMDX_REGV_FW_VAD_TYPE_NO_VAD),

#ifdef DBMDX_QED_SUPPORTED
		(DBMDX_REGN_AUDIO_PROCESSING_ROUTING |
			DBMDX_REGV_IO_SET_0 |
			DBMDX_REGV_IO_3N_2_NO_CP |
			DBMDX_REGV_IO_3N_1_NO_CP
			DBMDX_REGV_IO_3N_0_CP_4),

		(DBMDX_REGN_AUDIO_PROCESSING_CONFIG |
			DBMDX_REGV_VT_CP4 |
			DBMDX_REGV_VT_PROC_EN |
			DBMDX_REGV_ALGO1_EN_FW_MODE_1_AND_MODE_2),

#else /* DBMDX_QED_SUPPORTED */
		(DBMDX_REGN_AUDIO_PROCESSING_CONFIG |
#ifdef DBMDX_I2S_STREAMING_SUPPORTED
			DBMDX_REGV_I2S_CATCHUP_MODE_DISABLED),
#else
#ifdef DBMDX_I2S_BUFFERING_SUPPORTED
			DBMDX_REGV_EN_COPY_HIST |
#endif
			DBMDX_REGV_POST_DET_MODE_SWITCH_TO_STREAMING |
			DBMDX_REGV_VT_CP4 |
			DBMDX_REGV_VT_PROC_EN |
#if defined(DBMDX_I2S_BUFFERING_SUPPORTED) && \
			defined(DBMDX_I2S_CATCHUP_SUPPORTED)
			DBMDX_REGV_I2S_CATCHUP_MODE_ENABLED),
#else /* DBMDX_I2S_BUFFERING_SUPPORTED && DBMDX_I2S_CATCHUP_SUPPORTED */
			DBMDX_REGV_I2S_CATCHUP_MODE_DISABLED),
#endif /* DBMDX_I2S_BUFFERING_SUPPORTED && DBMDX_I2S_CATCHUP_SUPPORTED */
#endif /* DBMDX_I2S_STREAMING_SUPPORTED */

#endif /* DBMDX_QED_SUPPORTED */

		(DBMDX_VA_USLEEP | 0x1500),

		(DBMDX_REGN_AUDIO_STREAMING_SRC_SELECT |
			DBMDX_REGV_NO_STREAM_CH_4 |
			DBMDX_REGV_NO_STREAM_CH_3 |
			DBMDX_REGV_NO_STREAM_CH_2 |
#ifdef DBMDX_I2S_STREAMING_SUPPORTED
			DBMDX_REGV_NO_STREAM_CH_1)},
#else
			DBMDX_REGV_STREAM_CH_1_CP_4)},
#endif


#ifdef DBMDX_QED_SUPPORTED
	.num_of_va_cfg_values = 5,
#else /* DBMDX_QED_SUPPORTED */
	.num_of_va_cfg_values = 4,
#endif /* DBMDX_QED_SUPPORTED */

#ifdef DBMDX_I2S_BUFFERING_SUPPORTED
	.va_post_tdm_cfg_values = (u32 []){
		(DBMDX_REGN_AUDIO_ROUTING_CONFIG |
#ifdef AEC_REF_32_TO_16_BIT
			DBMDX_REGV_TDM_SYNC_DELAY_6_CLKS_CYCLES |
#else
			DBMDX_REGV_TDM_SYNC_DELAY_4_CLKS_CYCLES |
#endif
			DBMDX_REGV_TDM_SYNC_RIGHT_CH |
			DBMDX_REGV_USE_TDM_MUSIC_TO_SYNC |
			DBMDX_REGV_MUSIC_IN_TDM0) },
	.num_of_va_post_tdm_cfg_values = 1,
	.usecase_supports_i2s_buffering = true,

#endif /* DBMDX_I2S_BUFFERING_SUPPORTED */

	.config_mics = MIC_CONFIG_BY_USECASE,
	.mic_config = {
		(DBMDX_REGV_DM_CLOCK_SRC_DM0_GPIO6_DM1_GPIO12 |
		 DBMDX_REGV_DM_DATA_SRC_DM0_GPIO5_DM1_GPIO11 |
		 DBMDX_REGV_DDF_AUDIO_ATTENUATION_MINUS_6dB |
		 DBMDX_REGV_DM_CLK_FREQ_1536_1200_SR_8KHz_16KHz_32KHz_48KHz |
		 DBMDX_REGV_CLOCK_POLARITY_FALLING_EDGE |
		 DBMDX_REGV_DDF_AND_DM_CONFIG_SD_DDF_DM1),
		(DBMDX_REGV_DM_CLOCK_SRC_DM0_GPIO8_DM1_GPIO14 |
		 DBMDX_REGV_DM_DATA_SRC_DM0_GPIO9_DM1_GPIO13 |
		 DBMDX_REGV_DDF_AUDIO_ATTENUATION_MINUS_6dB |
		 DBMDX_REGV_DM_CLK_FREQ_1536_1200_SR_8KHz_16KHz_32KHz_48KHz |
		 DBMDX_REGV_DDF_AND_DM_CONFIG_DDF0_DM0),
		(DBMDX_REGV_DM_CLOCK_SRC_DM0_GPIO8_DM1_GPIO14 |
		 DBMDX_REGV_DM_DATA_SRC_DM0_GPIO9_DM1_GPIO13 |
		 DBMDX_REGV_DDF_AUDIO_ATTENUATION_MINUS_6dB |
		 DBMDX_REGV_DM_CLK_FREQ_1536_1200_SR_8KHz_16KHz_32KHz_48KHz |
		 DBMDX_REGV_CLOCK_POLARITY_FALLING_EDGE |
		 DBMDX_REGV_DDF_AND_DM_CONFIG_DDF1_DM0),
		0x0000},

	.va_ve_cfg_values = (u32 []){
		(DBMDX_REGN_ASRP_OUTPUT_ROUTING |
			DBMDX_REGV_ASRP_OUTPUT_SRC_BFPF_1 |
			DBMDX_REGV_ASRP_OUTPUT_DEST_TX_1),
		(DBMDX_REGN_AUDIO_PROCESSING_CONFIG |
			DBMDX_REGV_ALGO1_EN_FW_MODE_2_ONLY),
		(DBMDX_VA_USLEEP | 0x1500) },

	.num_of_va_ve_cfg_values = 3,

#ifdef DBMDX_I2S_STREAMING_SUPPORTED
	.va_ve_post_tdm_cfg_values = (u32 []){
		(DBMDX_REGN_AUDIO_ROUTING_CONFIG |
			DBMDX_REGV_TDM_SYNC_RIGHT_CH |
			DBMDX_REGV_USE_TDM_MUSIC_TO_SYNC |
			DBMDX_REGV_MUSIC_IN_TDM3 |
			DBMDX_REGV_TDM_SYNC_DELAY_4_CLKS_CYCLES) },
	.num_of_va_ve_post_tdm_cfg_values = 1,
#endif

	.audio_routing_config = {
		(DBMDX_REGV_IO_SET_0 |
			DBMDX_REGV_IO_3N_2_CP_2 |
			DBMDX_REGV_IO_3N_1_CP_1 |
			DBMDX_REGV_IO_3N_0_CP_0),
		(DBMDX_REGV_IO_SET_1 |
			DBMDX_REGV_IO_3N_2_NO_CP |
			DBMDX_REGV_IO_3N_1_NO_CP |
			DBMDX_REGV_IO_3N_0_NO_CP),
		(DBMDX_REGV_IO_SET_2 |
			DBMDX_REGV_IO_3N_2_NO_CP |
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
			DBMDX_REGV_IO_3N_0_CP_3),
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
#ifdef DBMDX_I2S_STREAMING_SUPPORTED
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
			 DBMDX_REGV_RX_TX_CP3),
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
#endif /* DBMDX_I2S_STREAMING_SUPPORTED */

#ifdef DBMDX_I2S_BUFFERING_SUPPORTED
		/* DBMD4 TDM0_TX is connected to Host Codec */
		{	.tdm_index	= 0x0,
			.tdm_type	= TDM_TYPE_TX,
			.tdm_reg_config	=
			(DBMDX_REGV_DEMUX_MUX_DISABLE |
			 DBMDX_REGV_SAMPLE_WIDTH_16_BIT |
			 DBMDX_REGV_NUM_OF_CHANNELS_1_CH |
			 DBMDX_REGV_INPUT_OUTPUT_SAMPLE_RATE_48_KHz |
		DBMDX_REGV_RX_TX_I2S_CH_SUM_HIGH_AND_LOW_INTO_ONE_SAMPLE |
#ifdef DBMDX_I2S_CATCHUP_SUPPORTED
			 DBMDX_REGV_RESAMPLE_RATIO_NO_RESAMPLING |
#else /* DBMDX_I2S_CATCHUP_SUPPORTED */
			 DBMDX_REGV_RESAMPLE_RATIO_3_1 |
#endif /* DBMDX_I2S_CATCHUP_SUPPORTED */
			 DBMDX_REGV_RESAMPLE_TYPE_INTERPOLATION |
			 DBMDX_REGV_HW_BLOCK_EN |
			 DBMDX_REGV_RX_TX_CP3),
			.tdm_interface	= TDM_INTERFACE_VA,
			.num_of_io_reg_configs = 4,
#ifdef AEC_REF_32_TO_16_BIT
			.io_reg_configs	= {
				{		.addr = DBMD4_TDM_0_TX_ADDR,
						.value = 0x00804053
				},
				{		.addr = DBMD4_TDM_0_TX_ADDR + 4,
						.value = 0x00000064
				},
				{		.addr = DBMD4_TDM_0_TX_ADDR + 6,
						.value = 0x101F003F
				},
				{		.addr = DBMD4_TDM_0_TX_ADDR+0xA,
						.value = 0x00000005
				}
			},
#else
			.io_reg_configs	= {
				{		.addr = DBMD4_TDM_0_TX_ADDR,
						.value = 0x00804053
				},
				{		.addr = DBMD4_TDM_0_TX_ADDR + 4,
						.value = 0x00241024
				},
				{		.addr = DBMD4_TDM_0_TX_ADDR + 6,
						.value = 0x100F001F
				},
				{		.addr = DBMD4_TDM_0_TX_ADDR+0xA,
						.value = 0x0000000F
				}
			},
#endif /* AEC_REF_32_TO_16_BIT */
		},
#endif /* DBMDX_I2S_BUFFERING_SUPPORTED */
		/* DBMD4 TDM1_TX is connected to DBMD2 TDM#_RX */
		{	.tdm_index	= 0x1,
			.tdm_type	= TDM_TYPE_TX,
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
#ifndef DBMDX_I2S_STREAMING_SUPPORTED
		/* DBMD4 TDM1_RX is connected to DBMD2 TDM#_TX */
		{	.tdm_index	= 0x1,
			.tdm_type	= TDM_TYPE_RX,
			.tdm_reg_config	=
				(DBMDX_REGV_DEMUX_MUX_DISABLE |
				DBMDX_REGV_SAMPLE_WIDTH_16_BIT |
				DBMDX_REGV_NUM_OF_CHANNELS_1_CH |
				DBMDX_REGV_INPUT_OUTPUT_SAMPLE_RATE_16_KHz |
				DBMDX_REGV_RX_TX_I2S_CH_USE_LOW_WORD_ONLY |
				DBMDX_REGV_RESAMPLE_RATIO_NO_RESAMPLING |
				DBMDX_REGV_HW_BLOCK_EN |
				DBMDX_REGV_RX_TX_CP4),
			.tdm_interface	= TDM_INTERFACE_VA,
			.num_of_io_reg_configs = 3,
			.io_reg_configs	= {
				{		.addr = DBMD4_TDM_1_RX_ADDR,
						.value = 0x00800015
				},
				{		.addr = DBMD4_TDM_1_RX_ADDR + 4,
						.value = 0x00070007
				},
				{		.addr = DBMD4_TDM_1_RX_ADDR + 6,
						.value = 0x101F003F
				}
			},
		},
#endif /* DBMDX_I2S_STREAMING_SUPPORTED */
		/* DBMD2 TDM0_RX is connected to DBMD4 TDM1_TX */
		{	.tdm_index	= 0x0,
			.tdm_type	= TDM_TYPE_RX,
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
#ifndef DBMDX_I2S_STREAMING_SUPPORTED
		/* DBMD2 TDM0_TX is connected to DBMD4 TDM1_RX */
		{	.tdm_index	= 0x0,
			.tdm_type	= TDM_TYPE_TX,
			/* .tdm_reg_config	= 0x0013, */
			.tdm_reg_config	=
				(DBMDX_REGV_DEMUX_MUX_DISABLE |
				DBMDX_REGV_SAMPLE_WIDTH_16_BIT |
				DBMDX_REGV_NUM_OF_CHANNELS_1_CH |
				DBMDX_REGV_INPUT_OUTPUT_SAMPLE_RATE_16_KHz |
				DBMDX_REGV_RX_TX_I2S_CH_USE_LOW_WORD_ONLY |
				DBMDX_REGV_RESAMPLE_RATIO_NO_RESAMPLING |
				DBMDX_REGV_HW_BLOCK_EN |
				DBMDX_REGV_RX_TX_CP3),
			.tdm_interface = TDM_INTERFACE_VA_VE,
			.num_of_io_reg_configs = 3,
			.io_reg_configs	= {
				{		.addr = DBMD2_TDM_0_TX_ADDR,
						.value = 0x00800015
				},
				{		.addr = DBMD2_TDM_0_TX_ADDR + 4,
						.value = 0x00072007
				},
				{		.addr = DBMD2_TDM_0_TX_ADDR + 6,
						.value = 0x101F003F
				}
			},
		},
#endif /* DBMDX_I2S_STREAMING_SUPPORTED */
	},
#ifdef DBMDX_I2S_BUFFERING_SUPPORTED
	.num_of_tdm_configs = 5,
#else
#ifdef DBMDX_I2S_STREAMING_SUPPORTED
	.num_of_tdm_configs = 3,
#else
	.num_of_tdm_configs = 4,
#endif
#endif
	.va_start_cmd_type = START_CMD_TYPE_TDM,
	.send_va_start_cmd = true,
	/* Enable:
			TDM1_TX (D4<=>D2)
#ifndef DBMDX_I2S_STREAMING_SUPPORTED
			TDM1_RX (D4<=>D2)
#endif
	*/

#ifdef DBMDX_I2S_STREAMING_SUPPORTED
	.va_start_cmd = (DBMDX_REGV_TDM1_TX_EN |
			DBMDX_REGV_PROC_EN_SWITCH_FW_TO_BUFFERING_MODE),
#else
	.va_start_cmd = (DBMDX_REGV_TDM1_TX_EN |
			DBMDX_REGV_TDM1_RX_EN |
			DBMDX_REGV_PROC_EN_SWITCH_FW_TO_DETECTION_MODE),
#endif

#ifdef DBMDX_I2S_BUFFERING_SUPPORTED
	.va_start_i2s_buffering_cmd = (DBMDX_REGV_TDM0_TX_EN |
					DBMDX_REGV_TDM1_TX_EN |
					DBMDX_REGV_TDM1_RX_EN |
				DBMDX_REGV_PROC_EN_SWITCH_FW_TO_BUFFERING_MODE),
#endif
	.send_va_ve_start_cmd = true,
	.va_ve_start_cmd_type = START_CMD_TYPE_TDM,
	/* Enable:
#ifdef DBMDX_I2S_STREAMING_SUPPORTED
			TDM3_TX (D2<=>HOST)
#else DBMDX_I2S_STREAMING_SUPPORTED
			TDM0_RX (D4<=>D2)
#endif
			TDM0_RX (D4<=>D2)
	*/
#ifdef DBMDX_I2S_STREAMING_SUPPORTED
	.va_ve_start_cmd = (DBMDX_REGV_TDM3_TX_EN |
				DBMDX_REGV_TDM0_RX_EN |
				DBMDX_REGV_PROC_EN_SWITCH_FW_TO_BUFFERING_MODE),
#else
	.va_ve_start_cmd = (DBMDX_REGV_TDM0_TX_EN |
				DBMDX_REGV_TDM0_RX_EN |
				DBMDX_REGV_PROC_EN_SWITCH_FW_TO_BUFFERING_MODE),
#endif
};


/* Mango Barge In usecase 48Khz 16 Bit or 16 from 32 Bit*/
static struct usecase_config config_uc_mango_barge_in_48k_16b_32b_hw_rev_03 = {
#ifdef AEC_REF_32_TO_16_BIT
	.usecase_name = "uc_mango_barge_in_48k_32to16b_no_speaker",
#else
	.usecase_name = "uc_mango_barge_in_48k_16b_no_speaker",
#endif
	.id	= (DBMDX_USECASE_ID_UC_IDX_03 |
			DBMDX_USECASE_ID_PRJ_MANGO |
			DBMDX_USECASE_ID_HWREV_03),
	.hw_rev = 3,
#ifdef DBMDX_QED_SUPPORTED
	.send_va_asrp_parms = true,
#else
	.send_va_asrp_parms = false,
#endif
	.va_asrp_params_file_name = "asrp_params_qed.bin",
	.send_va_ve_asrp_parms = true,
	.va_ve_asrp_params_file_name = "asrp_params_aecnr.bin",
#ifndef VA_VE_TDM_RX_MASTER
	.change_clock_src = true,
#endif
	.tdm_clock_freq = TDM_CLK_FREQ_48,
#ifdef AEC_REF_32_TO_16_BIT
	.number_of_bits = 32,
#else
	.number_of_bits = 16,
#endif
#ifdef DBMDX_I2S_STREAMING_SUPPORTED
	.usecase_requires_amodel = false,
	.usecase_amodel_mode = 1,
	.usecase_sets_detection_mode = false,
	.usecase_supports_us_buffering = false,
#else
	.usecase_requires_amodel = true,
	.usecase_amodel_mode = 1,
	.usecase_sets_detection_mode = true,
	.usecase_supports_us_buffering = true,
#endif
	.va_chip_low_power_mode = false,
	.va_ve_chip_low_power_mode = false,
	.asrp_output_gain_type = (ASRP_TX_OUT_GAIN |
					ASRP_VCPF_OUT_GAIN |
					ASRP_RX_OUT_GAIN),

#if defined(DMBDX_OKG_AMODEL_SUPPORT) && defined(OKG_2_CH_AEC_ONLY_AUDIO_BUF)
	.num_of_output_channels = 2,
#else
	.num_of_output_channels = 1,
#endif
	.va_cfg_values = (u32 []){
#ifdef VA_VE_I2S_MASTER
		(DBMDX_REGN_DSP_CLOCK_CONFIG |
			DBMDX_REGV_USE_PLL_STEP_FROM_REG_0x1E |
			DBMDX_REGV_TL3_DIV_1 |
			DBMDX_REGV_APB_DIV_1 |
			DBMDX_REGV_AHB_DIV_1),
		(DBMDX_VA_USLEEP | 0x2710),
		(DBMDX_REGN_DSP_CLOCK_CONFIG_EXTENSION |
			DBMDX_REGV_CLK_EXT_PLL_STEP_16),
		(DBMDX_VA_MSLEEP | 0x64),
#endif
		(DBMDX_REGN_GENERAL_CONFIG_2 |
			DBMDX_REGV_MIC_SAMPLE_RATE_16K |
			DBMDX_REGV_FW_VAD_TYPE_NO_VAD),

#ifdef DBMDX_QED_SUPPORTED

		(DBMDX_REGN_AUDIO_PROCESSING_ROUTING |
			DBMDX_REGV_IO_SET_0 |
			DBMDX_REGV_IO_3N_2_NO_CP |
			DBMDX_REGV_IO_3N_1_NO_CP
			DBMDX_REGV_IO_3N_0_CP_4),

		(DBMDX_REGN_AUDIO_PROCESSING_CONFIG |
			DBMDX_REGV_VT_CP4 |
			DBMDX_REGV_VT_PROC_EN |
			DBMDX_REGV_ALGO1_EN_FW_MODE_1_AND_MODE_2),

#else /* DBMDX_QED_SUPPORTED */

		(DBMDX_REGN_AUDIO_PROCESSING_CONFIG |
#ifdef DBMDX_I2S_STREAMING_SUPPORTED
			DBMDX_REGV_I2S_CATCHUP_MODE_DISABLED),
#else
#ifdef DBMDX_I2S_BUFFERING_SUPPORTED
			DBMDX_REGV_EN_COPY_HIST |
#endif
			DBMDX_REGV_POST_DET_MODE_SWITCH_TO_STREAMING |
#if defined(DMBDX_OKG_AMODEL_SUPPORT) && defined(OKG_2_CH_AEC_ONLY_AUDIO_BUF)
			DBMDX_REGV_VT_CP2 |
#else
			DBMDX_REGV_VT_CP4 |
			DBMDX_REGV_VT_PROC_EN |
#endif
#if defined(DBMDX_I2S_BUFFERING_SUPPORTED) && \
			defined(DBMDX_I2S_CATCHUP_SUPPORTED)
			DBMDX_REGV_I2S_CATCHUP_MODE_ENABLED),
#else /* DBMDX_I2S_BUFFERING_SUPPORTED && DBMDX_I2S_CATCHUP_SUPPORTED */
			DBMDX_REGV_I2S_CATCHUP_MODE_DISABLED),
#endif /* DBMDX_I2S_BUFFERING_SUPPORTED && DBMDX_I2S_CATCHUP_SUPPORTED */
#endif /* DBMDX_I2S_STREAMING_SUPPORTED */
#endif /* DBMDX_QED_SUPPORTED */

		(DBMDX_VA_USLEEP | 0x1500),

		(DBMDX_REGN_AUDIO_STREAMING_SRC_SELECT |
			DBMDX_REGV_NO_STREAM_CH_4 |
			DBMDX_REGV_NO_STREAM_CH_3 |
#if defined(DMBDX_OKG_AMODEL_SUPPORT) && defined(OKG_2_CH_AEC_ONLY_AUDIO_BUF)
			DBMDX_REGV_STREAM_CH_2_CP_3 |
#else
			DBMDX_REGV_NO_STREAM_CH_2 |
#endif
#ifdef DBMDX_I2S_STREAMING_SUPPORTED
			DBMDX_REGV_NO_STREAM_CH_1)},
#else
			DBMDX_REGV_STREAM_CH_1_CP_4)},
#endif

#ifdef DBMDX_QED_SUPPORTED
#ifdef VA_VE_I2S_MASTER
	.num_of_va_cfg_values = 9,
#else
	.num_of_va_cfg_values = 5,
#endif
#else /* DBMDX_QED_SUPPORTED */
#ifdef VA_VE_I2S_MASTER
	.num_of_va_cfg_values = 8,
#else
	.num_of_va_cfg_values = 4,
#endif
#endif /* DBMDX_QED_SUPPORTED */

#ifdef DBMDX_I2S_BUFFERING_SUPPORTED
	.va_post_tdm_cfg_values = (u32 []){
		(DBMDX_REGN_AUDIO_ROUTING_CONFIG |
#ifdef AEC_REF_32_TO_16_BIT
			DBMDX_REGV_TDM_SYNC_DELAY_6_CLKS_CYCLES |
#else /* AEC_REF_32_TO_16_BIT */
			DBMDX_REGV_TDM_SYNC_DELAY_4_CLKS_CYCLES |
#endif/* AEC_REF_32_TO_16_BIT */
			DBMDX_REGV_TDM_SYNC_RIGHT_CH |
			DBMDX_REGV_USE_TDM_MUSIC_TO_SYNC |
			DBMDX_REGV_MUSIC_IN_TDM0) },
	.num_of_va_post_tdm_cfg_values = 1,
	.usecase_supports_i2s_buffering = true,

#endif /* DBMDX_I2S_BUFFERING_SUPPORTED */

	.config_mics = MIC_CONFIG_BY_USECASE,
	.mic_config = {
		(DBMDX_REGV_DM_CLOCK_SRC_DM0_GPIO6_DM1_GPIO12 |
		 DBMDX_REGV_DM_DATA_SRC_DM0_GPIO5_DM1_GPIO11 |
		 DBMDX_REGV_DDF_AUDIO_ATTENUATION_0dB |
		 DBMDX_REGV_DM_CLK_FREQ_1536_1200_SR_8KHz_16KHz_32KHz_48KHz |
		 DBMDX_REGV_CLOCK_POLARITY_FALLING_EDGE |
		 DBMDX_REGV_DDF_AND_DM_CONFIG_SD_DDF_DM1),
		(DBMDX_REGV_DM_CLOCK_SRC_DM0_GPIO8_DM1_GPIO14 |
		 DBMDX_REGV_DM_DATA_SRC_DM0_GPIO9_DM1_GPIO13 |
		 DBMDX_REGV_DDF_AUDIO_ATTENUATION_0dB |
		 DBMDX_REGV_DM_CLK_FREQ_1536_1200_SR_8KHz_16KHz_32KHz_48KHz |
		 DBMDX_REGV_DDF_AND_DM_CONFIG_DDF0_DM0),
#if defined(DMBDX_OKG_AMODEL_SUPPORT) && defined(OKG_2_CH_AEC_ONLY_AUDIO_BUF)
		0x0000,
#else
		(DBMDX_REGV_DM_CLOCK_SRC_DM0_GPIO8_DM1_GPIO14 |
		 DBMDX_REGV_DM_DATA_SRC_DM0_GPIO9_DM1_GPIO13 |
		 DBMDX_REGV_DDF_AUDIO_ATTENUATION_0dB |
		 DBMDX_REGV_DM_CLK_FREQ_1536_1200_SR_8KHz_16KHz_32KHz_48KHz |
		 DBMDX_REGV_CLOCK_POLARITY_FALLING_EDGE |
		 DBMDX_REGV_DDF_AND_DM_CONFIG_DDF1_DM0),
#endif
		0x0000},

	.va_ve_cfg_values = (u32 []){
		(DBMDX_REGN_ASRP_OUTPUT_ROUTING |
#ifdef USE_VCPF_OUTPUT
			DBMDX_REGV_ASRP_OUTPUT_SRC_VCPF_1 |
#else
			DBMDX_REGV_ASRP_OUTPUT_SRC_BFPF_1 |
#endif
			DBMDX_REGV_ASRP_OUTPUT_DEST_TX_1),
#if defined(DMBDX_OKG_AMODEL_SUPPORT) && defined(OKG_2_CH_AEC_ONLY_AUDIO_BUF)
		(DBMDX_REGN_ASRP_OUTPUT_ROUTING |
			DBMDX_REGV_ASRP_OUTPUT_SRC_AEC_1 |
			DBMDX_REGV_ASRP_OUTPUT_DEST_TX_2),
		(DBMDX_REGN_ASRP_OUTPUT_ROUTING |
			DBMDX_REGV_ASRP_OUTPUT_SRC_AEC_2 |
			DBMDX_REGV_ASRP_OUTPUT_DEST_TX_3),
#endif
		(DBMDX_REGN_AUDIO_PROCESSING_CONFIG |
			DBMDX_REGV_EN_48_EC_REF_DECIM |
			DBMDX_REGV_ALGO1_EN_FW_MODE_2_ONLY),
		(DBMDX_VA_USLEEP | 0x1500) },
#if defined(DMBDX_OKG_AMODEL_SUPPORT) && defined(OKG_2_CH_AEC_ONLY_AUDIO_BUF)
	.num_of_va_ve_cfg_values = 5,
#else
	.num_of_va_ve_cfg_values = 3,
#endif

#ifdef DBMDX_I2S_STREAMING_SUPPORTED
	.va_ve_post_tdm_cfg_values = (u32 []){
		(DBMDX_REGN_AUDIO_ROUTING_CONFIG |
			DBMDX_REGV_TDM_SYNC_RIGHT_CH |
			DBMDX_REGV_USE_TDM_MUSIC_TO_SYNC |
			DBMDX_REGV_MUSIC_IN_TDM3 |
			DBMDX_REGV_TDM_SYNC_DELAY_4_CLKS_CYCLES) },
	.num_of_va_ve_post_tdm_cfg_values = 1,
#endif

	.audio_routing_config = {
		(DBMDX_REGV_IO_SET_0 |
#if defined(DMBDX_OKG_AMODEL_SUPPORT) && defined(OKG_2_CH_AEC_ONLY_AUDIO_BUF)
			DBMDX_REGV_IO_3N_2_NO_CP |
#else
			DBMDX_REGV_IO_3N_2_CP_2 |
#endif
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
			DBMDX_REGV_IO_3N_0_CP_5),
		(DBMDX_REGV_IO_SET_4 |
			DBMDX_REGV_IO_3N_2_NO_CP |
			DBMDX_REGV_IO_3N_1_NO_CP |
			DBMDX_REGV_IO_3N_0_NO_CP),
		(DBMDX_REGV_IO_SET_5 |
#if defined(DMBDX_OKG_AMODEL_SUPPORT) && defined(OKG_2_CH_AEC_ONLY_AUDIO_BUF)
			DBMDX_REGV_IO_3N_2_CP_2 |
			DBMDX_REGV_IO_3N_1_CP_1 |
			DBMDX_REGV_IO_3N_0_CP_0),
#else
			DBMDX_REGV_IO_3N_2_NO_CP |
			DBMDX_REGV_IO_3N_1_NO_CP |
			DBMDX_REGV_IO_3N_0_CP_3),
#endif
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
				(DBMDX_REGV_DEMUX_MUX_ENABLE |
				DBMDX_REGV_SAMPLE_WIDTH_16_BIT |
				DBMDX_REGV_NUM_OF_CHANNELS_2_CH |
				DBMDX_REGV_INPUT_OUTPUT_SAMPLE_RATE_48_KHz |
				DBMDX_REGV_RX_TX_I2S_CH_USE_I2S_STEREO |
				DBMDX_REGV_RESAMPLE_TYPE_DECIMATION |
				DBMDX_REGV_RESAMPLE_RATIO_NO_RESAMPLING |
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
						.value = 0x0080001D,
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
#if defined(DBMDX_I2S_STREAMING_SUPPORTED) || defined(VA_VE_TDM_RX_MASTER) ||\
		defined(VA_VE_I2S_MASTER)
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
			 DBMDX_REGV_RX_TX_CP3),
			.tdm_interface	= TDM_INTERFACE_VA_VE,
			.num_of_io_reg_configs = 4,
#ifdef AEC_REF_32_TO_16_BIT
			.io_reg_configs	= {
				{		.addr = DBMD2_TDM_3_TX_ADDR,
#if defined(VA_VE_TDM_RX_MASTER) || defined(VA_VE_I2S_MASTER)
						.value = 0x00804052
#else
						.value = 0x00804053
#endif
				},
				{		.addr = DBMD2_TDM_3_TX_ADDR + 4,
#if defined(VA_VE_TDM_RX_MASTER) || defined(VA_VE_I2S_MASTER)
						.value = 0x00441044
#else
						.value = 0x00000064
#endif
				},
				{		.addr = DBMD2_TDM_3_TX_ADDR + 6,
						.value = 0x101F003F
				},
				{		.addr = DBMD2_TDM_3_TX_ADDR+0xA,
#if defined(VA_VE_TDM_RX_MASTER) || defined(VA_VE_I2S_MASTER)
						.value = 0x0000000F
#else
						.value = 0x00000005
#endif
				}
			},
#else
			.io_reg_configs	= {
				{		.addr = DBMD2_TDM_3_TX_ADDR,
#if defined(VA_VE_TDM_RX_MASTER) || defined(VA_VE_I2S_MASTER)
						.value = 0x00804052
#else
						.value = 0x00804053
#endif
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
#endif /* DBMDX_I2S_STREAMING_SUPPORTED) || VA_VE_TDM_RX_MASTER */

#ifdef DBMDX_I2S_BUFFERING_SUPPORTED
		/* DBMD4 TDM0_TX is connected to Host Codec */
		{	.tdm_index	= 0x0,
			.tdm_type	= TDM_TYPE_TX,
			.tdm_reg_config	=
			(DBMDX_REGV_DEMUX_MUX_DISABLE |
			 DBMDX_REGV_SAMPLE_WIDTH_16_BIT |
			 DBMDX_REGV_NUM_OF_CHANNELS_1_CH |
			 DBMDX_REGV_INPUT_OUTPUT_SAMPLE_RATE_48_KHz |
		DBMDX_REGV_RX_TX_I2S_CH_SUM_HIGH_AND_LOW_INTO_ONE_SAMPLE |
#ifdef DBMDX_I2S_CATCHUP_SUPPORTED
			 DBMDX_REGV_RESAMPLE_RATIO_NO_RESAMPLING |
#else /* DBMDX_I2S_CATCHUP_SUPPORTED */
			 DBMDX_REGV_RESAMPLE_RATIO_3_1 |
#endif /* DBMDX_I2S_CATCHUP_SUPPORTED */
			 DBMDX_REGV_RESAMPLE_TYPE_INTERPOLATION |
			 DBMDX_REGV_HW_BLOCK_EN |
			 DBMDX_REGV_RX_TX_CP3),
			.tdm_interface	= TDM_INTERFACE_VA,
			.num_of_io_reg_configs = 4,
#ifdef AEC_REF_32_TO_16_BIT
			.io_reg_configs	= {
				{		.addr = DBMD4_TDM_0_TX_ADDR,
						.value = 0x00804053
				},
				{		.addr = DBMD4_TDM_0_TX_ADDR + 4,
						.value = 0x00000064
				},
				{		.addr = DBMD4_TDM_0_TX_ADDR + 6,
						.value = 0x101F003F
				},
				{		.addr = DBMD4_TDM_0_TX_ADDR+0xA,
						.value = 0x00000005
				}
			},
#else
			.io_reg_configs	= {
				{		.addr = DBMD4_TDM_0_TX_ADDR,
						.value = 0x00804053
				},
				{		.addr = DBMD4_TDM_0_TX_ADDR + 4,
						.value = 0x00241024
				},
				{		.addr = DBMD4_TDM_0_TX_ADDR + 6,
						.value = 0x100F001F
				},
				{		.addr = DBMD4_TDM_0_TX_ADDR+0xA,
						.value = 0x0000000F
				}
			},
#endif /* AEC_REF_32_TO_16_BIT */
		},
#endif /* DBMDX_I2S_BUFFERING_SUPPORTED */
		/* DBMD4 TDM1_TX is connected to DBMD2 TDM#_RX */
		{	.tdm_index	= 0x1,
			.tdm_type	= TDM_TYPE_TX,
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
#ifdef VA_VE_I2S_MASTER
						.value = 0x00800015
#else
						.value = 0x00800010
#endif
				},
				{		.addr = DBMD4_TDM_1_TX_ADDR + 4,
						.value = 0x00073007
				},
				{		.addr = DBMD4_TDM_1_TX_ADDR + 6,
						.value = 0x101F003F
				}
			},
		},
#ifndef DBMDX_I2S_STREAMING_SUPPORTED
		/* DBMD4 TDM1_RX is connected to DBMD2 TDM#_TX */
		{	.tdm_index	= 0x1,
			.tdm_type	= TDM_TYPE_RX,
			.tdm_reg_config	=
#if defined(DMBDX_OKG_AMODEL_SUPPORT) && defined(OKG_2_CH_AEC_ONLY_AUDIO_BUF)
				(DBMDX_REGV_DEMUX_MUX_ENABLE |
				DBMDX_REGV_SAMPLE_WIDTH_16_BIT |
				DBMDX_REGV_NUM_OF_CHANNELS_4_CH |
				DBMDX_REGV_INPUT_OUTPUT_SAMPLE_RATE_16_KHz |
				DBMDX_REGV_RX_TX_I2S_CH_USE_I2S_STEREO |
				DBMDX_REGV_RESAMPLE_RATIO_NO_RESAMPLING |
				DBMDX_REGV_HW_BLOCK_EN |
				DBMDX_REGV_RX_TX_CP2),
#else
				(DBMDX_REGV_DEMUX_MUX_DISABLE |
				DBMDX_REGV_SAMPLE_WIDTH_16_BIT |
				DBMDX_REGV_NUM_OF_CHANNELS_1_CH |
				DBMDX_REGV_INPUT_OUTPUT_SAMPLE_RATE_16_KHz |
				DBMDX_REGV_RX_TX_I2S_CH_USE_LOW_WORD_ONLY |
				DBMDX_REGV_RESAMPLE_RATIO_NO_RESAMPLING |
				DBMDX_REGV_HW_BLOCK_EN |
				DBMDX_REGV_RX_TX_CP4),
#endif
			.tdm_interface	= TDM_INTERFACE_VA,
			.num_of_io_reg_configs = 3,
			.io_reg_configs	= {
				{		.addr = DBMD4_TDM_1_RX_ADDR,
						.value = 0x00800015
				},
				{		.addr = DBMD4_TDM_1_RX_ADDR + 4,
#if defined(DMBDX_OKG_AMODEL_SUPPORT) && defined(OKG_2_CH_AEC_ONLY_AUDIO_BUF)
						.value = 0x00000027
#else
						.value = 0x00000007
#endif
				},
				{		.addr = DBMD4_TDM_1_RX_ADDR + 6,
						.value = 0x101F003F
				}
			},
		},
#endif /* DBMDX_I2S_STREAMING_SUPPORTED */
		/* DBMD2 TDM0_RX is connected to DBMD4 TDM1_TX */
		{	.tdm_index	= 0x0,
			.tdm_type	= TDM_TYPE_RX,
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
#ifndef DBMDX_I2S_STREAMING_SUPPORTED
		/* DBMD2 TDM0_TX is connected to DBMD4 TDM1_RX */
		{	.tdm_index	= 0x0,
			.tdm_type	= TDM_TYPE_TX,
			.tdm_reg_config	=
#if defined(DMBDX_OKG_AMODEL_SUPPORT) && defined(OKG_2_CH_AEC_ONLY_AUDIO_BUF)
				(DBMDX_REGV_DEMUX_MUX_ENABLE |
				DBMDX_REGV_SAMPLE_WIDTH_16_BIT |
				DBMDX_REGV_NUM_OF_CHANNELS_4_CH |
				DBMDX_REGV_INPUT_OUTPUT_SAMPLE_RATE_16_KHz |
				DBMDX_REGV_RX_TX_I2S_CH_USE_I2S_STEREO |
				DBMDX_REGV_RESAMPLE_RATIO_NO_RESAMPLING |
				DBMDX_REGV_HW_BLOCK_EN |
				DBMDX_REGV_RX_TX_CP0),
#elif defined(VA_VE_I2S_MASTER)
				(DBMDX_REGV_DEMUX_MUX_ENABLE |
				DBMDX_REGV_SAMPLE_WIDTH_16_BIT |
				DBMDX_REGV_NUM_OF_CHANNELS_4_CH |
				DBMDX_REGV_INPUT_OUTPUT_SAMPLE_RATE_16_KHz |
				DBMDX_REGV_RX_TX_I2S_CH_USE_I2S_STEREO |
				DBMDX_REGV_RESAMPLE_RATIO_NO_RESAMPLING |
				DBMDX_REGV_HW_BLOCK_EN |
				DBMDX_REGV_RX_TX_CP3),
#else
				(DBMDX_REGV_DEMUX_MUX_DISABLE |
				DBMDX_REGV_SAMPLE_WIDTH_16_BIT |
				DBMDX_REGV_NUM_OF_CHANNELS_1_CH |
				DBMDX_REGV_INPUT_OUTPUT_SAMPLE_RATE_16_KHz |
				DBMDX_REGV_RX_TX_I2S_CH_USE_LOW_WORD_ONLY |
				DBMDX_REGV_RESAMPLE_RATIO_NO_RESAMPLING |
				DBMDX_REGV_HW_BLOCK_EN |
				DBMDX_REGV_RX_TX_CP3),
#endif
			.tdm_interface	= TDM_INTERFACE_VA_VE,
			.num_of_io_reg_configs = 3,
			.io_reg_configs	= {
				{		.addr = DBMD2_TDM_0_TX_ADDR,
#ifdef VA_VE_I2S_MASTER
						.value = 0x00800010
#else
						.value = 0x00800015
#endif
				},
				{		.addr = DBMD2_TDM_0_TX_ADDR + 4,
#if defined(DMBDX_OKG_AMODEL_SUPPORT) && defined(OKG_2_CH_AEC_ONLY_AUDIO_BUF)
						.value = 0x00000027
#elif defined(VA_VE_I2S_MASTER)
						.value = 0x00073007
#else
						.value = 0x00072007
#endif
				},
				{		.addr = DBMD2_TDM_0_TX_ADDR + 6,
						.value = 0x101F003F
				}
			},
		},
#endif /* DBMDX_I2S_STREAMING_SUPPORTED */
	},

#ifdef DBMDX_I2S_BUFFERING_SUPPORTED

#if defined(VA_VE_TDM_RX_MASTER) || defined(VA_VE_I2S_MASTER)
	.num_of_tdm_configs = 7,
#else /* !(#if defined(VA_VE_TDM_RX_MASTER) || defined(VA_VE_I2S_MASTER)) */
	.num_of_tdm_configs = 6,
#endif /* !(#if defined(VA_VE_TDM_RX_MASTER) || defined(VA_VE_I2S_MASTER)) */

#else /* DBMDX_I2S_BUFFERING_SUPPORTED */

#if defined(VA_VE_TDM_RX_MASTER) || defined(VA_VE_I2S_MASTER)
	.num_of_tdm_configs = 6,
#elif defined(DBMDX_I2S_STREAMING_SUPPORTED)
	.num_of_tdm_configs = 4,
#else /* !DBMDX_I2S_STREAMING_SUPPORTED && !VA_VE_TDM_RX_MASTER */
	.num_of_tdm_configs = 5,
#endif /* !DBMDX_I2S_STREAMING_SUPPORTED && !VA_VE_TDM_RX_MASTER */

#endif/*  DBMDX_I2S_BUFFERING_SUPPORTED */

	.va_start_cmd_type = START_CMD_TYPE_TDM,
	.send_va_start_cmd = true,
	/* Enable:
			TDM1_TX (D4<=>D2)
			TDM1_RX (D4<=>D2)
	*/
#ifdef DBMDX_I2S_STREAMING_SUPPORTED
	.va_start_cmd = (DBMDX_REGV_TDM1_TX_EN |
			DBMDX_REGV_PROC_EN_SWITCH_FW_TO_BUFFERING_MODE),
#else
	.va_start_cmd = (DBMDX_REGV_TDM1_TX_EN |
			DBMDX_REGV_TDM1_RX_EN |
			DBMDX_REGV_PROC_EN_SWITCH_FW_TO_DETECTION_MODE),
#endif
#ifdef DBMDX_I2S_BUFFERING_SUPPORTED
	.va_start_i2s_buffering_cmd = (DBMDX_REGV_TDM0_TX_EN |
					DBMDX_REGV_TDM1_TX_EN |
					DBMDX_REGV_TDM1_RX_EN |
				DBMDX_REGV_PROC_EN_SWITCH_FW_TO_BUFFERING_MODE),
#endif
	.va_ve_start_cmd_type = START_CMD_TYPE_TDM,
	.send_va_ve_start_cmd = true,
	/* Enable:
			TDM3_RX (codec=>D2)
#ifdef DBMDX_I2S_STREAMING_SUPPORTED || VA_VE_TDM_RX_MASTER
			TDM3_TX (D2=>Host Codec)
#endif
#ifndef DBMDX_I2S_STREAMING_SUPPORTED
			TDM0_TX (D4<=>D2)
#endif
			TDM0_RX (D4<=>D2)
	*/
	.va_ve_start_cmd = (DBMDX_REGV_TDM3_RX_EN |
#if defined(DBMDX_I2S_STREAMING_SUPPORTED) || defined(VA_VE_TDM_RX_MASTER) ||\
		defined(VA_VE_I2S_MASTER)
				DBMDX_REGV_TDM3_TX_EN |
#endif
#ifndef DBMDX_I2S_STREAMING_SUPPORTED
				DBMDX_REGV_TDM0_TX_EN |
#endif
				DBMDX_REGV_TDM0_RX_EN |
				DBMDX_REGV_PROC_EN_SWITCH_FW_TO_BUFFERING_MODE),
};


/* Mango VC usecase 48Khz 16 Bit or 16 from 32 Bit*/
static struct usecase_config config_uc_mango_vc_48k_16b_32b_hw_rev_03 = {
#ifdef VC_SRATE_16KHZ
#ifdef AEC_REF_32_TO_16_BIT
	.usecase_name = "uc_mango_vc_16k_32to16b",
#else
	.usecase_name = "uc_mango_vc_16k_16b",
#endif
#else
#ifdef AEC_REF_32_TO_16_BIT
	.usecase_name = "uc_mango_vc_48k_32to16b",
#else
	.usecase_name = "uc_mango_vc_48k_16b",
#endif
#endif
	.id	= (DBMDX_USECASE_ID_UC_IDX_04 |
			DBMDX_USECASE_ID_PRJ_MANGO |
			DBMDX_USECASE_ID_HWREV_03),
	.hw_rev = 3,
	.send_va_asrp_parms = false,
	.va_asrp_params_file_name = "asrp_params_qed.bin",
	.send_va_ve_asrp_parms = true,
#ifdef VT_WHILE_VC_SUPPORTED
	.va_ve_asrp_params_file_name = "asrp_params_vc_with_vt.bin",
#else
	.va_ve_asrp_params_file_name = "asrp_params_vc.bin",
#endif

	.change_clock_src = true,
#ifdef VC_SRATE_16KHZ
	.tdm_clock_freq = TDM_CLK_FREQ_16,
#else
	.tdm_clock_freq = TDM_CLK_FREQ_48,
#endif
#ifdef AEC_REF_32_TO_16_BIT
	.number_of_bits = 32,
#else
	.number_of_bits = 16,
#endif

#ifdef VT_WHILE_VC_SUPPORTED
	.usecase_requires_amodel = true,
#else
	.usecase_requires_amodel = false,
#endif
	.usecase_amodel_mode = 1,
#ifdef VT_WHILE_VC_SUPPORTED
	.usecase_sets_detection_mode = true,
#endif
	.usecase_supports_us_buffering = true,
	.va_chip_low_power_mode = false,
	.va_ve_chip_low_power_mode = false,
	.asrp_output_gain_type = (ASRP_TX_OUT_GAIN |
					ASRP_VCPF_OUT_GAIN |
					ASRP_RX_OUT_GAIN),

	.va_cfg_values = (u32 []){
		(DBMDX_REGN_GENERAL_CONFIG_2 |
			DBMDX_REGV_MIC_SAMPLE_RATE_16K |
			DBMDX_REGV_FW_VAD_TYPE_NO_VAD),

#if defined(DBMDX_I2S_BUFFERING_SUPPORTED) || defined(VT_WHILE_VC_SUPPORTED)
		(DBMDX_REGN_AUDIO_PROCESSING_CONFIG |
#ifdef DBMDX_I2S_BUFFERING_SUPPORTED
			DBMDX_REGV_EN_COPY_HIST |
#endif
			DBMDX_REGV_POST_DET_MODE_SWITCH_TO_STREAMING |
			DBMDX_REGV_VT_CP4 |
#ifdef VT_WHILE_VC_SUPPORTED
			DBMDX_REGV_VT_PROC_EN |
#endif
#if defined(DBMDX_I2S_BUFFERING_SUPPORTED) && \
	defined(DBMDX_I2S_CATCHUP_SUPPORTED) && defined(VT_WHILE_VC_SUPPORTED)
			DBMDX_REGV_I2S_CATCHUP_MODE_ENABLED),
#else
			DBMDX_REGV_I2S_CATCHUP_MODE_DISABLED),
#endif

		(DBMDX_VA_USLEEP | 0x1500),
#endif
		(DBMDX_REGN_AUDIO_STREAMING_SRC_SELECT |
			DBMDX_REGV_NO_STREAM_CH_4 |
			DBMDX_REGV_NO_STREAM_CH_3 |
			DBMDX_REGV_NO_STREAM_CH_2 |
#ifdef VT_WHILE_VC_SUPPORTED
			DBMDX_REGV_STREAM_CH_1_CP_4)},
#else
			DBMDX_REGV_NO_STREAM_CH_1)},
#endif
#if defined(DBMDX_I2S_BUFFERING_SUPPORTED) || defined(VT_WHILE_VC_SUPPORTED)
	.num_of_va_cfg_values = 4,
#else
	.num_of_va_cfg_values = 2,
#endif

#ifdef DBMDX_I2S_BUFFERING_SUPPORTED
	.va_post_tdm_cfg_values = (u32 []){
		(DBMDX_REGN_AUDIO_ROUTING_CONFIG |
#ifdef AEC_REF_32_TO_16_BIT
			DBMDX_REGV_TDM_SYNC_DELAY_6_CLKS_CYCLES |
#else /* AEC_REF_32_TO_16_BIT */
			DBMDX_REGV_TDM_SYNC_DELAY_4_CLKS_CYCLES |
#endif/* AEC_REF_32_TO_16_BIT */
			DBMDX_REGV_TDM_SYNC_RIGHT_CH |
			DBMDX_REGV_USE_TDM_MUSIC_TO_SYNC |
#if defined(DBMDX_I2S_CATCHUP_SUPPORTED) && defined(VC_RX_PROCESS_SUPPORTED)
			DBMDX_REGV_EN_RX_PROCESS |
#endif
			DBMDX_REGV_MUSIC_IN_TDM0) },
	.num_of_va_post_tdm_cfg_values = 1,
	.usecase_supports_i2s_buffering = true,

#endif /* DBMDX_I2S_BUFFERING_SUPPORTED */

	.config_mics = MIC_CONFIG_BY_USECASE,
	.mic_config = {
#ifndef USE_EVBS_CONFIG
		(DBMDX_REGV_DM_CLOCK_SRC_DM0_GPIO6_DM1_GPIO12 |
		 DBMDX_REGV_DM_DATA_SRC_DM0_GPIO5_DM1_GPIO11 |
		 DBMDX_REGV_DDF_AUDIO_ATTENUATION_0dB |
		 DBMDX_REGV_DM_CLK_FREQ_1536_1200_SR_8KHz_16KHz_32KHz_48KHz |
		 DBMDX_REGV_CLOCK_POLARITY_FALLING_EDGE |
		 DBMDX_REGV_DDF_AND_DM_CONFIG_SD_DDF_DM1),
		(DBMDX_REGV_DM_CLOCK_SRC_DM0_GPIO8_DM1_GPIO14 |
		 DBMDX_REGV_DM_DATA_SRC_DM0_GPIO9_DM1_GPIO13 |
		 DBMDX_REGV_DDF_AUDIO_ATTENUATION_0dB |
		 DBMDX_REGV_DM_CLK_FREQ_1536_1200_SR_8KHz_16KHz_32KHz_48KHz |
		 DBMDX_REGV_CLOCK_POLARITY_RISING_EDGE |
		 DBMDX_REGV_DDF_AND_DM_CONFIG_DDF0_DM0),
		(DBMDX_REGV_DM_CLOCK_SRC_DM0_GPIO8_DM1_GPIO14 |
		 DBMDX_REGV_DM_DATA_SRC_DM0_GPIO9_DM1_GPIO13 |
		 DBMDX_REGV_DDF_AUDIO_ATTENUATION_0dB |
		 DBMDX_REGV_DM_CLK_FREQ_1536_1200_SR_8KHz_16KHz_32KHz_48KHz |
		 DBMDX_REGV_CLOCK_POLARITY_FALLING_EDGE |
		 DBMDX_REGV_DDF_AND_DM_CONFIG_DDF0_DM0),
#else
		(DBMDX_REGV_DM_CLOCK_SRC_DM0_GPIO8_DM1_GPIO14 |
		 DBMDX_REGV_DM_DATA_SRC_DM0_GPIO9_DM1_GPIO13 |
		 DBMDX_REGV_DDF_AUDIO_ATTENUATION_0dB |
		 DBMDX_REGV_DM_CLK_FREQ_1536_1200_SR_8KHz_16KHz_32KHz_48KHz |
		 DBMDX_REGV_CLOCK_POLARITY_FALLING_EDGE |
		 DBMDX_REGV_DDF_AND_DM_CONFIG_DDF0_DM0),
		(DBMDX_REGV_DM_CLOCK_SRC_DM0_GPIO6_DM1_GPIO12 |
		 DBMDX_REGV_DM_DATA_SRC_DM0_GPIO5_DM1_GPIO11 |
		 DBMDX_REGV_DDF_AUDIO_ATTENUATION_0dB |
		 DBMDX_REGV_DM_CLK_FREQ_1536_1200_SR_8KHz_16KHz_32KHz_48KHz |
		 DBMDX_REGV_CLOCK_POLARITY_RISING_EDGE |
		 DBMDX_REGV_DDF_AND_DM_CONFIG_DDF0_DM1),
		(DBMDX_REGV_DM_CLOCK_SRC_DM0_GPIO6_DM1_GPIO12 |
		 DBMDX_REGV_DM_DATA_SRC_DM0_GPIO5_DM1_GPIO11 |
		 DBMDX_REGV_DDF_AUDIO_ATTENUATION_0dB |
		 DBMDX_REGV_DM_CLK_FREQ_1536_1200_SR_8KHz_16KHz_32KHz_48KHz |
		 DBMDX_REGV_CLOCK_POLARITY_FALLING_EDGE |
		 DBMDX_REGV_DDF_AND_DM_CONFIG_DDF1_DM1),
#endif
	0x0000},

	.va_ve_cfg_values = (u32 []){
#ifdef VT_WHILE_VC_SUPPORTED
		(DBMDX_REGN_ASRP_OUTPUT_ROUTING |
#ifdef USE_VCPF_OUTPUT
			DBMDX_REGV_ASRP_OUTPUT_SRC_VCPF_1 |
#else
			DBMDX_REGV_ASRP_OUTPUT_SRC_BFPF_1 |
#endif
			DBMDX_REGV_ASRP_OUTPUT_DEST_TX_1),
		(DBMDX_REGN_ASRP_OUTPUT_ROUTING |
			DBMDX_REGV_ASRP_OUTPUT_SRC_VCPF_1 |
			DBMDX_REGV_ASRP_OUTPUT_DEST_TX_2),
#else /* VT_WHILE_VC_SUPPORTED */
		(DBMDX_REGN_ASRP_OUTPUT_ROUTING |
			DBMDX_REGV_ASRP_OUTPUT_SRC_VCPF_1 |
			DBMDX_REGV_ASRP_OUTPUT_DEST_TX_1),
#endif /* VT_WHILE_VC_SUPPORTED */
		(DBMDX_REGN_AUDIO_PROCESSING_CONFIG |
#ifndef VC_SRATE_16KHZ
			DBMDX_REGV_EN_48_EC_REF_DECIM |
#endif
			DBMDX_REGV_ALGO1_EN_FW_MODE_2_ONLY),
		(DBMDX_VA_USLEEP | 0x1500) },
#if defined(VT_WHILE_VC_SUPPORTED)
	.num_of_va_ve_cfg_values = 4,
#else
	.num_of_va_ve_cfg_values = 3,
#endif

#ifdef DBMDX_VC_I2S_STREAMING_SUPPORTED
	.va_ve_post_tdm_cfg_values = (u32 []){
		(DBMDX_REGN_AUDIO_ROUTING_CONFIG |
			DBMDX_REGV_TDM_SYNC_RIGHT_CH |
			DBMDX_REGV_USE_TDM_MUSIC_TO_SYNC |
			DBMDX_REGV_MUSIC_IN_TDM3 |
			DBMDX_REGV_TDM_SYNC_DELAY_4_CLKS_CYCLES) },
	.num_of_va_ve_post_tdm_cfg_values = 1,
#endif

	.audio_routing_config = {
		(DBMDX_REGV_IO_SET_0 |
			DBMDX_REGV_IO_3N_2_CP_2 |
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
			DBMDX_REGV_IO_3N_0_CP_5),
		(DBMDX_REGV_IO_SET_4 |
			DBMDX_REGV_IO_3N_2_NO_CP |
			DBMDX_REGV_IO_3N_1_NO_CP |
			DBMDX_REGV_IO_3N_0_NO_CP),
		(DBMDX_REGV_IO_SET_5 |
			DBMDX_REGV_IO_3N_2_NO_CP |
#if defined(VT_WHILE_VC_SUPPORTED)
			DBMDX_REGV_IO_3N_1_CP_1 |
#else
			DBMDX_REGV_IO_3N_1_NO_CP |
#endif
			DBMDX_REGV_IO_3N_0_CP_0),

		(DBMDX_REGV_IO_SET_6 |
			DBMDX_REGV_IO_3N_2_NO_CP |
			DBMDX_REGV_IO_3N_1_NO_CP |
			DBMDX_REGV_IO_3N_0_NO_CP),
		(DBMDX_REGV_IO_SET_7 |
#if defined(VC_RX_PROCESS_SUPPORTED) && defined(VT_WHILE_VC_SUPPORTED)
			DBMDX_REGV_IO_3N_2_CP_2 |
#elif defined(VC_RX_PROCESS_SUPPORTED)
			DBMDX_REGV_IO_3N_2_CP_1 |
#else
			DBMDX_REGV_IO_3N_2_NO_CP |
#endif
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
				(DBMDX_REGV_DEMUX_MUX_ENABLE |
				DBMDX_REGV_SAMPLE_WIDTH_16_BIT |
				DBMDX_REGV_NUM_OF_CHANNELS_2_CH |
				DBMDX_REGV_INPUT_OUTPUT_SAMPLE_RATE_48_KHz |
				DBMDX_REGV_RX_TX_I2S_CH_USE_I2S_STEREO |
				DBMDX_REGV_RESAMPLE_TYPE_DECIMATION |
				DBMDX_REGV_RESAMPLE_RATIO_NO_RESAMPLING |
				DBMDX_REGV_HW_BLOCK_EN |
				DBMDX_REGV_RX_TX_CP4),
			.tdm_interface	= TDM_INTERFACE_VA_VE,
#ifdef AEC_REF_32_TO_16_BIT
			.num_of_io_reg_configs = 4,
			.io_reg_configs	= {
				{		.addr = DBMD2_TDM_3_RX_ADDR,
						.value = 0x0080405D
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
						.value = 0x00800015
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
#ifdef DBMDX_VC_I2S_STREAMING_SUPPORTED

#ifdef VC_RX_PROCESS_SUPPORTED
		/* DBMD2 TDM2_TX is connected to SPEAKER */
		{	.tdm_index	= 0x2,
			.tdm_type	= TDM_TYPE_TX,
			.tdm_reg_config	=
			(DBMDX_REGV_DEMUX_MUX_DISABLE |
			 DBMDX_REGV_SAMPLE_WIDTH_16_BIT |
			DBMDX_REGV_NUM_OF_CHANNELS_1_CH |
		DBMDX_REGV_RX_TX_I2S_CH_SUM_HIGH_AND_LOW_INTO_ONE_SAMPLE |
#ifdef VC_SRATE_16KHZ
			 DBMDX_REGV_INPUT_OUTPUT_SAMPLE_RATE_16_KHz |
			 DBMDX_REGV_RESAMPLE_TYPE_DECIMATION |
			 DBMDX_REGV_RESAMPLE_RATIO_NO_RESAMPLING |
#else
			 DBMDX_REGV_INPUT_OUTPUT_SAMPLE_RATE_48_KHz |
			 DBMDX_REGV_RESAMPLE_TYPE_INTERPOLATION |
			 DBMDX_REGV_RESAMPLE_RATIO_3_1 |
#endif
			 DBMDX_REGV_HW_BLOCK_EN |
#if defined(VT_WHILE_VC_SUPPORTED)
			 DBMDX_REGV_RX_TX_CP2),
#else
			 DBMDX_REGV_RX_TX_CP1),
#endif
			.tdm_interface	= TDM_INTERFACE_VA_VE,
			.num_of_io_reg_configs = 4,
#ifdef AEC_REF_32_TO_16_BIT
			.io_reg_configs	= {
				{		.addr = DBMD2_TDM_2_TX_ADDR,
						.value = 0x00804053
				},
				{		.addr = DBMD2_TDM_2_TX_ADDR + 4,
						.value = 0x00000064
				},
				{		.addr = DBMD2_TDM_2_TX_ADDR + 6,
						.value = 0x101F003F
				},
				{		.addr = DBMD2_TDM_2_TX_ADDR+0xA,
						.value = 0x00000005
				}
			},
#else
			.io_reg_configs	= {
				{		.addr = DBMD2_TDM_2_TX_ADDR,
						.value = 0x00804053
				},
				{		.addr = DBMD2_TDM_2_TX_ADDR + 4,
						.value = 0x00241024
				},
				{		.addr = DBMD2_TDM_2_TX_ADDR + 6,
						.value = 0x100F001F
				},
				{		.addr = DBMD2_TDM_2_TX_ADDR+0xA,
						.value = 0x0000000F
				}
			},
#endif /* AEC_REF_32_TO_16_BIT */
		},
#endif /* VC_RX_PROCESS_SUPPORTED */

		/* DBMD2 TDM3_TX is connected to Host Codec */
		{	.tdm_index	= 0x3,
			.tdm_type	= TDM_TYPE_TX,
			.tdm_reg_config	=
			(DBMDX_REGV_DEMUX_MUX_DISABLE |
			 DBMDX_REGV_SAMPLE_WIDTH_16_BIT |
			DBMDX_REGV_NUM_OF_CHANNELS_1_CH |
		DBMDX_REGV_RX_TX_I2S_CH_SUM_HIGH_AND_LOW_INTO_ONE_SAMPLE |
#ifdef VC_SRATE_16KHZ
			 DBMDX_REGV_INPUT_OUTPUT_SAMPLE_RATE_16_KHz |
			 DBMDX_REGV_RESAMPLE_TYPE_DECIMATION |
			 DBMDX_REGV_RESAMPLE_RATIO_NO_RESAMPLING |
#else
			 DBMDX_REGV_INPUT_OUTPUT_SAMPLE_RATE_48_KHz |
			 DBMDX_REGV_RESAMPLE_TYPE_INTERPOLATION |
			 DBMDX_REGV_RESAMPLE_RATIO_3_1 |
#endif
			 DBMDX_REGV_HW_BLOCK_EN |
#if defined(VT_WHILE_VC_SUPPORTED)
			 DBMDX_REGV_RX_TX_CP1),
#else
			 DBMDX_REGV_RX_TX_CP0),
#endif
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
						.value = 0x00241024
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
#endif /* DBMDX_VC_I2S_STREAMING_SUPPORTED */

#ifdef DBMDX_I2S_BUFFERING_SUPPORTED
		/* DBMD4 TDM0_TX is connected to Host Codec */
		{	.tdm_index	= 0x0,
			.tdm_type	= TDM_TYPE_TX,
			.tdm_reg_config	=
#if defined(DBMDX_I2S_CATCHUP_SUPPORTED) || \
		defined(VC_RX_PROCESS_SUPPORTED) || \
		defined(VT_WHILE_VC_SUPPORTED)
			(DBMDX_REGV_DEMUX_MUX_ENABLE |
			 DBMDX_REGV_NUM_OF_CHANNELS_2_CH |
			 DBMDX_REGV_RX_TX_I2S_CH_USE_I2S_STEREO |
#else
			(DBMDX_REGV_DEMUX_MUX_DISABLE |
			 DBMDX_REGV_NUM_OF_CHANNELS_1_CH |
		DBMDX_REGV_RX_TX_I2S_CH_SUM_HIGH_AND_LOW_INTO_ONE_SAMPLE |
#endif
#ifdef VC_SRATE_16KHZ
			 DBMDX_REGV_INPUT_OUTPUT_SAMPLE_RATE_16_KHz |
#else
			 DBMDX_REGV_INPUT_OUTPUT_SAMPLE_RATE_48_KHz |
#endif
#if defined(DBMDX_I2S_CATCHUP_SUPPORTED) || defined(VC_SRATE_16KHZ)
			 DBMDX_REGV_RESAMPLE_RATIO_NO_RESAMPLING |
#else
			 DBMDX_REGV_RESAMPLE_RATIO_3_1 |
#endif
#ifdef VC_SRATE_16KHZ
			 DBMDX_REGV_RESAMPLE_TYPE_DECIMATION |
#else
			 DBMDX_REGV_RESAMPLE_TYPE_INTERPOLATION |
#endif
			 DBMDX_REGV_SAMPLE_WIDTH_16_BIT |
			 DBMDX_REGV_HW_BLOCK_EN |
			 DBMDX_REGV_RX_TX_CP4),
			.tdm_interface	= TDM_INTERFACE_VA,
			.num_of_io_reg_configs = 4,
#ifdef AEC_REF_32_TO_16_BIT
			.io_reg_configs	= {
				{		.addr = DBMD4_TDM_0_TX_ADDR,
						.value = 0x00804053
				},
				{		.addr = DBMD4_TDM_0_TX_ADDR + 4,
						.value = 0x00000064
				},
				{		.addr = DBMD4_TDM_0_TX_ADDR + 6,
						.value = 0x101F003F
				},
				{		.addr = DBMD4_TDM_0_TX_ADDR+0xA,
						.value = 0x00000005
				}
			},
#else
			.io_reg_configs	= {
				{		.addr = DBMD4_TDM_0_TX_ADDR,
						.value = 0x00804053
				},
				{		.addr = DBMD4_TDM_0_TX_ADDR + 4,
						.value = 0x00241024
				},
				{		.addr = DBMD4_TDM_0_TX_ADDR + 6,
						.value = 0x100F001F
				},
				{		.addr = DBMD4_TDM_0_TX_ADDR+0xA,
						.value = 0x0000000F
				}
			},
#endif /* AEC_REF_32_TO_16_BIT */
		},
#endif /* DBMDX_I2S_BUFFERING_SUPPORTED */
		/* DBMD4 TDM1_TX is connected to DBMD2 TDM#_RX */
		{	.tdm_index	= 0x1,
			.tdm_type	= TDM_TYPE_TX,
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
#if defined(DBMDX_I2S_BUFFERING_SUPPORTED) || defined(VT_WHILE_VC_SUPPORTED)
		/* DBMD4 TDM1_RX is connected to DBMD2 TDM#_TX */
		{	.tdm_index	= 0x1,
			.tdm_type	= TDM_TYPE_RX,
			.tdm_reg_config	=
#ifdef DBMDX_I2S_BUFFERING_SUPPORTED
				(DBMDX_REGV_DEMUX_MUX_ENABLE |
#else
				(DBMDX_REGV_DEMUX_MUX_DISABLE |
#endif
#if defined(DBMDX_VC_I2S_STREAMING_SUPPORTED)
				DBMDX_REGV_NUM_OF_CHANNELS_1_CH |
#elif defined(VT_WHILE_VC_SUPPORTED) && defined(VC_RX_PROCESS_SUPPORTED)
				DBMDX_REGV_NUM_OF_CHANNELS_4_CH |
#else
				DBMDX_REGV_NUM_OF_CHANNELS_2_CH |
#endif
#ifdef DBMDX_I2S_BUFFERING_SUPPORTED
				DBMDX_REGV_RX_TX_I2S_CH_USE_I2S_STEREO	 |
#else
				DBMDX_REGV_RX_TX_I2S_CH_USE_LOW_WORD_ONLY |
#endif
				DBMDX_REGV_SAMPLE_WIDTH_16_BIT |
				DBMDX_REGV_INPUT_OUTPUT_SAMPLE_RATE_16_KHz |
				DBMDX_REGV_RESAMPLE_RATIO_NO_RESAMPLING |
				DBMDX_REGV_HW_BLOCK_EN |
				DBMDX_REGV_RX_TX_CP4),
			.tdm_interface	= TDM_INTERFACE_VA,
			.num_of_io_reg_configs = 3,
			.io_reg_configs	= {
				{		.addr = DBMD4_TDM_1_RX_ADDR,
						.value = 0x00800015
				},
				{		.addr = DBMD4_TDM_1_RX_ADDR + 4,
#if defined(DBMDX_I2S_BUFFERING_SUPPORTED) && defined(VT_WHILE_VC_SUPPORTED) &&\
		defined(VC_RX_PROCESS_SUPPORTED)
						.value = 0x00000027
#else
						.value = 0x00000007
#endif
				},
				{		.addr = DBMD4_TDM_1_RX_ADDR + 6,
						.value = 0x101F003F
				}
			},
		},
#endif /* DBMDX_I2S_BUFFERING_SUPPORTED || DBMDX_VC_I2S_STREAMING_SUPPORTED */
		/* DBMD2 TDM0_RX is connected to DBMD4 TDM1_TX */
		{	.tdm_index	= 0x0,
			.tdm_type	= TDM_TYPE_RX,
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
#if defined(VT_WHILE_VC_SUPPORTED) || \
		(defined(DBMDX_I2S_BUFFERING_SUPPORTED))
		/* DBMD2 TDM0_TX is connected to DBMD4 TDM1_RX */
		{	.tdm_index	= 0x0,
			.tdm_type	= TDM_TYPE_TX,
			.tdm_reg_config	=
#ifdef DBMDX_I2S_BUFFERING_SUPPORTED
				(DBMDX_REGV_DEMUX_MUX_ENABLE |
#else
				(DBMDX_REGV_DEMUX_MUX_DISABLE |
#endif
#if defined(DBMDX_VC_I2S_STREAMING_SUPPORTED)
				DBMDX_REGV_NUM_OF_CHANNELS_1_CH |
#elif defined(VT_WHILE_VC_SUPPORTED) && defined(VC_RX_PROCESS_SUPPORTED)
				DBMDX_REGV_NUM_OF_CHANNELS_4_CH |
#else
				DBMDX_REGV_NUM_OF_CHANNELS_2_CH |
#endif
#ifdef DBMDX_I2S_BUFFERING_SUPPORTED
				DBMDX_REGV_RX_TX_I2S_CH_USE_I2S_STEREO	 |
#else
				DBMDX_REGV_RX_TX_I2S_CH_USE_LOW_WORD_ONLY |
#endif
				DBMDX_REGV_SAMPLE_WIDTH_16_BIT |
				DBMDX_REGV_INPUT_OUTPUT_SAMPLE_RATE_16_KHz |
				DBMDX_REGV_RESAMPLE_RATIO_NO_RESAMPLING |
				DBMDX_REGV_HW_BLOCK_EN |
				DBMDX_REGV_RX_TX_CP0),
			.tdm_interface	= TDM_INTERFACE_VA_VE,
			.num_of_io_reg_configs = 3,
			.io_reg_configs	= {
				{		.addr = DBMD2_TDM_0_TX_ADDR,
						.value = 0x00800015
				},
				{		.addr = DBMD2_TDM_0_TX_ADDR + 4,
#if defined(DBMDX_VC_I2S_STREAMING_SUPPORTED)
						.value = 0x00072007
#elif defined(VT_WHILE_VC_SUPPORTED) && defined(VC_RX_PROCESS_SUPPORTED)
						.value = 0x00000027
#else
						.value = 0x00000007
#endif
				},
				{		.addr = DBMD2_TDM_0_TX_ADDR + 6,
						.value = 0x101F003F
				}
			},
		},
#endif
	},
#ifdef DBMDX_I2S_BUFFERING_SUPPORTED
	.num_of_tdm_configs = 6,

#elif defined(DBMDX_VC_I2S_STREAMING_SUPPORTED)

#if defined(VT_WHILE_VC_SUPPORTED) && defined(VC_RX_PROCESS_SUPPORTED)
	.num_of_tdm_configs = 7,
#elif defined(VT_WHILE_VC_SUPPORTED)
	.num_of_tdm_configs = 6,
#elif defined(VC_RX_PROCESS_SUPPORTED)
	.num_of_tdm_configs = 5,
#else
	.num_of_tdm_configs = 4,
#endif
#else /* DBMDX_VC_I2S_STREAMING_SUPPORTED */
	.num_of_tdm_configs = 4,
#endif/*  DBMDX_I2S_BUFFERING_SUPPORTED */

	.va_start_cmd_type = START_CMD_TYPE_TDM,
	.send_va_start_cmd = true,
/* TDM Lines Description:
 *		TDM1_TX (D4=>D2)
 *		TDM1_RX (D4<=D2)
 *		TDM0_TX (D4=>CODEC)
 */
	.va_start_cmd = (DBMDX_REGV_TDM1_TX_EN |
#ifdef DBMDX_I2S_BUFFERING_SUPPORTED
			 DBMDX_REGV_TDM0_TX_EN |
#endif
#if defined(DBMDX_I2S_BUFFERING_SUPPORTED) || defined(VT_WHILE_VC_SUPPORTED)
			 DBMDX_REGV_TDM1_RX_EN |
#endif
#ifdef VT_WHILE_VC_SUPPORTED
			DBMDX_REGV_PROC_EN_SWITCH_FW_TO_DETECTION_MODE),
#else
			DBMDX_REGV_PROC_EN_SWITCH_FW_TO_BUFFERING_MODE),
#endif

#ifdef DBMDX_I2S_BUFFERING_SUPPORTED
	.va_start_i2s_buffering_cmd = (DBMDX_REGV_TDM0_TX_EN |
					DBMDX_REGV_TDM1_TX_EN |
					DBMDX_REGV_TDM1_RX_EN |
				DBMDX_REGV_PROC_EN_SWITCH_FW_TO_BUFFERING_MODE),
#endif
	.va_ve_start_cmd_type = START_CMD_TYPE_TDM,
	.send_va_ve_start_cmd = true,
/* TDM Lines Description:
 *			TDM3_RX (codec=>D2)
 *
 *			TDM3_TX (D2=>Host Codec)
 *			TDM2_TX (D2=>Speaker) RX Process
 *			TDM0_TX (D2=>D4)
 *			TDM0_RX (D2<=D4)
 */
	.va_ve_start_cmd = (DBMDX_REGV_TDM3_RX_EN |
				DBMDX_REGV_TDM0_RX_EN |
#ifdef DBMDX_VC_I2S_STREAMING_SUPPORTED
				DBMDX_REGV_TDM3_TX_EN |
#ifdef VC_RX_PROCESS_SUPPORTED
				DBMDX_REGV_TDM2_TX_EN |
#endif
#ifdef VT_WHILE_VC_SUPPORTED
				DBMDX_REGV_TDM0_TX_EN |
#endif
#else /*  DBMDX_VC_I2S_STREAMING_SUPPORTED */
				DBMDX_REGV_TDM0_TX_EN |
#endif
				DBMDX_REGV_PROC_EN_SWITCH_FW_TO_BUFFERING_MODE),
};



#endif /* _DBMDX_USECASE_CONFIG_MANGO_HW_REV_03_H */

#endif /* DBMDX_MANGO_USECASES_HW_REV_03_SUPPORTED */
