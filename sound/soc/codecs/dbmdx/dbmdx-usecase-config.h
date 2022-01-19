/*
 * dbmdx-usecase-config  --  Preset USE CASE configurations
 *
 * Copyright (C) 2014 DSP Group
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _DBMDX_USECASE_CONFIG_H
#define _DBMDX_USECASE_CONFIG_H

#include "dbmdx-usecase-config-def.h"

#ifdef DBMDX_MANGO_USECASES_HW_REV_01_SUPPORTED
#include "dbmdx-usecase-config-mango-hw-rev-01.h"
#endif

#ifdef DBMDX_MANGO_USECASES_HW_REV_02_SUPPORTED
#include "dbmdx-usecase-config-mango-hw-rev-02.h"
#endif

#ifdef DBMDX_MANGO_USECASES_HW_REV_03_SUPPORTED
#include "dbmdx-usecase-config-mango-hw-rev-03.h"
#endif

#ifdef DBMDX_MANGO_NOVT_AUDIOSTREAM_USECASES_SUPPORTED
#include "dbmdx-usecase-config-mango-novt-audiostream.h"
#endif

#ifdef DBMDX_ORANGE_USECASES_SUPPORTED
#include "dbmdx-usecase-config-orange.h"
#endif

#ifdef DBMDX_HS_USECASES_SUPPORTED
#include "dbmdx-usecase-config-hs.h"
#endif

#ifdef DBMDX_MELON_USECASES_SUPPORTED
#include "dbmdx-usecase-config-melon.h"
#endif

static struct usecase_config *usecases_map[] = {
#ifdef DBMDX_MANGO_USECASES_HW_REV_01_SUPPORTED
	&config_uc_mango_nr_hw_rev_01,
	&config_uc_mango_barge_in_48k_16b_32b_hw_rev_01,
#endif
#ifdef DBMDX_MANGO_USECASES_HW_REV_02_SUPPORTED
	&config_uc_mango_nr_hw_rev_02,
	&config_uc_mango_barge_in_48k_16b_32b_hw_rev_02,
#endif
#ifdef DBMDX_MANGO_USECASES_HW_REV_03_SUPPORTED
	&config_uc_mango_low_power_hw_rev_03,
	&config_uc_mango_nr_hw_rev_03,
	&config_uc_mango_barge_in_48k_16b_32b_hw_rev_03,
	&config_uc_mango_vc_48k_16b_32b_hw_rev_03,
#endif
#ifdef DBMDX_MANGO_NOVT_AUDIOSTREAM_USECASES_SUPPORTED
			&config_uc_mango_barge_in_48k_16b_32b_novt_audiostream,
#endif
#ifdef DBMDX_ORANGE_USECASES_SUPPORTED
#ifndef DBMDX_SUPPORT_OBSOLETE_ORANGE_USECASES
	&config_uc_orange_aec_dual_mic_48k,
#else
	&config_uc_orange_nr_tripple_mic,
	&config_uc_orange_aec_tripple_mic_16k,
	&config_uc_orange_aec_tripple_mic_48k,
	&config_uc_orange_voice_call_tripple_mic,
	&config_uc_orange_pdm_to_pcm_mono,
	&config_uc_orange_pdm_to_pcm_stereo,
	&config_uc_orange_tdm_loop_r,
	&config_uc_orange_tdm_loop_l,
#endif
#endif
#ifdef DBMDX_HS_USECASES_SUPPORTED
	&config_uc1_hs,
	&config_uc2_hs,
#endif
#ifdef DBMDX_MELON_USECASES_SUPPORTED
	&config_uc_melon_low_power,
	&config_uc_melon_nr_dual_mic,
	&config_uc_melon_barge_in_1mic,
	&config_uc_melon_barge_in_2mic,
#ifdef DBMDX_I2S_BUFFERING_SUPPORTED
	&config_uc_melon_aec_i2s_streaming_1mic,
	&config_uc_melon_ga_2mic_aec,
	&config_uc_melon_ga_2mic_aec_test_adc_out,
#endif
#endif

};

static struct usecase_config config_usecase_external = {
	.usecase_name = NULL,
	.va_asrp_params_file_name = NULL,
	.va_ve_asrp_params_file_name = NULL,
	.num_of_va_cfg_values = 0,
	.va_cfg_values = NULL,
	.num_of_va_ve_cfg_values = 0,
	.va_ve_cfg_values = NULL,
};

#endif
