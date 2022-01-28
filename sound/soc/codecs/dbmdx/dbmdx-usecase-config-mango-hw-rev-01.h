/*
 * dbmdx-usecase-config-mango_hwrev_01.h  --  Preset USE CASE configurations
 *
 * Copyright (C) 2014 DSP Group
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifdef DBMDX_MANGO_USECASES_HW_REV_01_SUPPORTED

#ifndef _DBMDX_USECASE_CONFIG_MANGO_HW_REV_01_H
#define _DBMDX_USECASE_CONFIG_MANGO_HW_REV_01_H

#include "dbmdx-interface.h"
#include "dbmdx-usecase-config-def.h"

/* Mango NR Usecase*/
static struct usecase_config config_uc_mango_nr_hw_rev_01 = {
	.usecase_name = "uc_mango_nr",
	.id	= (DBMDX_USECASE_ID_UC_IDX_01 |
			DBMDX_USECASE_ID_PRJ_MANGO |
			DBMDX_USECASE_ID_HWREV_01),
	.hw_rev = 1,
#ifdef DBMDX_QED_SUPPORTED
	.send_va_asrp_parms = true,
#else
	.send_va_asrp_parms = false,
#endif
	.va_asrp_params_file_name = "asrp_params_qed.bin",
	.send_va_ve_asrp_parms = true,
	.va_ve_asrp_params_file_name = "asrp_params_nr.bin",
	.change_clock_src = false,
	.usecase_requires_amodel = true,
	.usecase_amodel_mode = 1,
	.usecase_sets_detection_mode = true,
	.usecase_supports_us_buffering = true,
	.va_chip_low_power_mode = false,
	.va_ve_chip_low_power_mode = false,
#ifdef DBMDX_QED_SUPPORTED
	.va_cfg_values = (u32 []){ 0x80230000, 0x80110004, 0x8013fff4,
					0x80340443, 0x8aaa1500 },
	.num_of_va_cfg_values = 5,
#else
	.va_cfg_values = (u32 []){ 0x80230000, 0x8013fff4,
					0x80340440, 0x8aaa1500 },
	.num_of_va_cfg_values = 4,
#endif
	.config_mics = MIC_CONFIG_BY_USECASE,
	.mic_config = { 0x4096, 0xa081, 0xa093, 0x0000 },
	.va_ve_cfg_values = (u32 []){ 0x80230000, 0x80340002, 0x8aaa1500 },
	.num_of_va_ve_cfg_values = 3,
	.audio_routing_config = { 0x0210, 0x1eee, 0x2ee3, 0x3eee,
				  0xeeee, 0xeeee, 0xeeee, 0xeeee },

	.tdm_configs = {
		/* DBMD4 TDM1_TX is connected to DBMD2 TDM#_RX */
		{	.tdm_index	= 0x1,
			.tdm_type	= TDM_TYPE_TX,
			.tdm_reg_config	= 0xa210,
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
		/* DBMD4 TDM1_RX is connected to DBMD2 TDM#_TX */
		{	.tdm_index	= 0x1,
			.tdm_type	= TDM_TYPE_RX,
			.tdm_reg_config	= 0x0014,
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
		/* DBMD2 TDM3_RX is connected to DBMD4 TDM1_TX */
		{	.tdm_index	= 0x3,
			.tdm_type	= TDM_TYPE_RX,
			.tdm_reg_config	= 0xa210,
			.tdm_interface	= TDM_INTERFACE_VA_VE,
			.num_of_io_reg_configs = 3,
			.io_reg_configs	= {
				{		.addr = DBMD2_TDM_3_RX_ADDR,
						.value = 0x00800015
				},
				{		.addr = DBMD2_TDM_3_RX_ADDR + 4,
						.value = 0x00073007
				},
				{		.addr = DBMD2_TDM_3_RX_ADDR + 6,
						.value = 0x101F003F
				}
			},
		},
		/* DBMD2 TDM3_TX is connected to DBMD4 TDM1_RX */
		{	.tdm_index	= 0x3,
			.tdm_type	= TDM_TYPE_TX,
			.tdm_reg_config	= 0x0013,
			.tdm_interface	= TDM_INTERFACE_VA_VE,
			.num_of_io_reg_configs = 3,
			.io_reg_configs	= {
				{		.addr = DBMD2_TDM_3_TX_ADDR,
						.value = 0x00800015
				},
				{		.addr = DBMD2_TDM_3_TX_ADDR + 4,
						.value = 0x00072007
				},
				{		.addr = DBMD2_TDM_3_TX_ADDR + 6,
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
			TDM1_RX (D4<=>D2)
	*/
	.va_start_cmd = 0x0C04,
	.send_va_ve_start_cmd = true,
	.va_ve_start_cmd_type = START_CMD_TYPE_TDM,
	/* Enable:
			TDM3_TX (D4<=>D2)
			TDM3_RX (D4<=>D2)
	*/
	.va_ve_start_cmd = 0xC008,
};


/* Mango Barge In usecase 48Khz 16 Bit or 16 from 32 Bit*/
static struct usecase_config config_uc_mango_barge_in_48k_16b_32b_hw_rev_01 = {
#ifdef DBMD2_DRIVES_DCLASS_SPEAKER
#ifdef AEC_REF_32_TO_16_BIT
	.usecase_name = "uc_mango_barge_in_48k_32to16b",
#else
	.usecase_name = "uc_mango_barge_in_48k_16b",
#endif
#else
#ifdef AEC_REF_32_TO_16_BIT
	.usecase_name = "uc_mango_barge_in_48k_32to16b_no_speaker",
#else
	.usecase_name = "uc_mango_barge_in_48k_16b_no_speaker",
#endif
#endif
	.id	= (DBMDX_USECASE_ID_UC_IDX_03 |
			DBMDX_USECASE_ID_PRJ_MANGO |
			DBMDX_USECASE_ID_HWREV_01),
	.hw_rev = 1,
#ifdef DBMDX_QED_SUPPORTED
	.send_va_asrp_parms = true,
#else
	.send_va_asrp_parms = false,
#endif
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
	.usecase_requires_amodel = true,
	.usecase_amodel_mode = 1,
	.usecase_sets_detection_mode = true,
	.usecase_supports_us_buffering = true,
	.va_chip_low_power_mode = false,
	.va_ve_chip_low_power_mode = false,
#ifdef DBMDX_QED_SUPPORTED
	.va_cfg_values = (u32 []){ 0x80230000, 0x80110EE4, 0x8013fff4,
					0x80340443, 0x8aaa1500 },
	.num_of_va_cfg_values = 5,
#else
	.va_cfg_values = (u32 []){ 0x80230000, 0x8013fff4,
					0x80340440, 0x8aaa1500 },
	.num_of_va_cfg_values = 4,
#endif
	.config_mics = MIC_CONFIG_BY_USECASE,
	.mic_config = { 0x4496, 0xa481, 0xa493, 0x0000 },
	.va_ve_cfg_values = (u32 []){ 0x80230200, 0x80344002, 0x8aaa1500 },
	.num_of_va_ve_cfg_values = 3,
	.audio_routing_config = { 0x0210, 0x154e, 0x2e03, 0x3eee,
				  0xeeee, 0xeeee, 0xeeee, 0xeeee },
	.tdm_configs = {
		/* DBMD4 TDM1_TX is connected to DBMD2 TDM#_RX */
		{	.tdm_index	= 0x1,
			.tdm_type	= TDM_TYPE_TX,
			.tdm_reg_config	= 0xa210,
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
		/* DBMD4 TDM1_RX is connected to DBMD2 TDM#_TX */
		{	.tdm_index	= 0x1,
			.tdm_type	= TDM_TYPE_RX,
			.tdm_reg_config	= 0x0014,
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
		/* DBMD2 TDM0_RX is connected to the Host Codec */
		{	.tdm_index	= 0x0,
			.tdm_type	= TDM_TYPE_RX,
			.tdm_reg_config	= 0x9a14,
			/* .tdm_reg_config	= 0xda14,	32bit */
			.tdm_interface	= TDM_INTERFACE_VA_VE,
#ifdef AEC_REF_32_TO_16_BIT
			.num_of_io_reg_configs = 4,
			.io_reg_configs	= {
				{		.addr = DBMD2_TDM_0_RX_ADDR,
						.value =
						(0x00804050 | HW_TDM_POLARITY),
				},
				{		.addr = DBMD2_TDM_0_RX_ADDR + 4,
						.value = 0x00002064
				},
				{		.addr = DBMD2_TDM_0_RX_ADDR + 6,
						.value = 0x101F003F
				},
				{		.addr = DBMD2_TDM_0_RX_ADDR+0xA,
						.value = 0x00000005
				}
			},
#else
			.num_of_io_reg_configs = 3,
			.io_reg_configs	= {
				{		.addr = DBMD2_TDM_0_RX_ADDR,
						.value =
						(0x00800010 | HW_TDM_POLARITY),
				},
				{		.addr = DBMD2_TDM_0_RX_ADDR + 4,
						.value = 0x00002007
				},
				{		.addr = DBMD2_TDM_0_RX_ADDR + 6,
						.value = 0x100F001F
						/*.value = 0x101F003F 32bit */
				}
			},
#endif /* AEC_REF_32_TO_16_BIT */
		},
#ifdef DBMD2_DRIVES_DCLASS_SPEAKER
		/* DBMD2 TDM1_TX is connected to the DClass */
		{	.tdm_index	= 0x1,
			.tdm_type	= TDM_TYPE_TX,
			.tdm_reg_config	= 0x9a14,
			/* .tdm_reg_config	= 0xda14,	32bit */
			.tdm_interface	= TDM_INTERFACE_VA_VE,
#ifdef AEC_REF_32_TO_16_BIT
			.num_of_io_reg_configs = 4,
			.io_reg_configs	= {
				{		.addr = DBMD2_TDM_1_TX_ADDR,
						.value =
						(0x00804050 | HW_TDM_POLARITY),
				},
				{		.addr = DBMD2_TDM_1_TX_ADDR + 4,
						.value = 0x00000064
				},
				{		.addr = DBMD2_TDM_1_TX_ADDR + 6,
						.value = 0x101F003F
				},
				{		.addr = DBMD2_TDM_1_TX_ADDR+0xA,
						.value = 0x00000005
				}
			},
#else
			.num_of_io_reg_configs = 3,
			.io_reg_configs	= {
				{		.addr = DBMD2_TDM_1_TX_ADDR,
						.value =
						(0x00800010 | HW_TDM_POLARITY),
				},
				{		.addr = DBMD2_TDM_1_TX_ADDR + 4,
						.value = 0x00070027
				},
				{		.addr = DBMD2_TDM_1_TX_ADDR + 6,
						.value = 0x100F001F
						/*.value = 0x101F003F 32bit */
				}
			},
#endif /* AEC_REF_32_TO_16_BIT */
		},
#endif
		/* DBMD2 TDM3_RX is connected to DBMD4 TDM1_TX */
		{	.tdm_index	= 0x3,
			.tdm_type	= TDM_TYPE_RX,
			.tdm_reg_config	= 0xa210,
			.tdm_interface	= TDM_INTERFACE_VA_VE,
			.num_of_io_reg_configs = 3,
			.io_reg_configs	= {
				{		.addr = DBMD2_TDM_3_RX_ADDR,
						.value = 0x00800015
				},
				{		.addr = DBMD2_TDM_3_RX_ADDR + 4,
						.value = 0x00073007
				},
				{		.addr = DBMD2_TDM_3_RX_ADDR + 6,
						.value = 0x101F003F
				}
			},
		},
		/* DBMD2 TDM3_TX is connected to DBMD4 TDM1_RX */
		{	.tdm_index	= 0x3,
			.tdm_type	= TDM_TYPE_TX,
			.tdm_reg_config	= 0x0013,
			.tdm_interface	= TDM_INTERFACE_VA_VE,
			.num_of_io_reg_configs = 3,
			.io_reg_configs	= {
				{		.addr = DBMD2_TDM_3_TX_ADDR,
						.value = 0x00800015
				},
				{		.addr = DBMD2_TDM_3_TX_ADDR + 4,
						.value = 0x00072007
				},
				{		.addr = DBMD2_TDM_3_TX_ADDR + 6,
						.value = 0x101F003F
				}
			},
		},
	},
#ifdef DBMD2_DRIVES_DCLASS_SPEAKER
	.num_of_tdm_configs = 6,
#else
	.num_of_tdm_configs = 5,
#endif /* DBMD2_DRIVES_DCLASS_SPEAKER */
	.va_start_cmd_type = START_CMD_TYPE_TDM,
	.send_va_start_cmd = true,
	/* Enable:
			TDM1_TX (D4<=>D2)
			TDM1_RX (D4<=>D2)
	*/
	.va_start_cmd = 0x0C04,
	.va_ve_start_cmd_type = START_CMD_TYPE_TDM,
	.send_va_ve_start_cmd = true,
	/* Enable:
			TDM1_TX (D2=>DClass)
			TDM3_TX (D4<=>D2)
			TDM3_RX (D4<=>D2)
			TDM0_RX (codec=>D2)
	*/
#ifdef DBMD2_DRIVES_DCLASS_SPEAKER
	.va_ve_start_cmd = 0xC908,
#else
	.va_ve_start_cmd = 0xC108,
#endif /* DBMD2_DRIVES_DCLASS_SPEAKER */
};


#endif /* _DBMDX_USECASE_CONFIG_MANGO_HW_REV_01_H */

#endif /* DBMDX_MANGO_USECASES_HW_REV_01_SUPPORTED */
