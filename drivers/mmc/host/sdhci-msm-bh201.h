/*
 * Copyright (c) 2015, 2018, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#ifndef __SDHCI_MSM_BH201_H__
#define __SDHCI_MSM_BH201_H__

#include "sdhci-msm.h"

#define GPIO_SD_CARD_PWR	(12)
#define GPIO_SD_CARD_PRESENT (69)

#define	SD_FNC_AM_SDR12		0x0
#define	SD_FNC_AM_SDR25		0x1
#define	SD_FNC_AM_HS		0x1
#define	SD_FNC_AM_DS		0x0

#define	SD_FNC_AM_SDR50		0x2
#define	SD_FNC_AM_SDR104	0x3
#define	SD_FNC_AM_DDR50		0x4

#define BIT_PASS_MASK  (0x7ff )
#define SDR104_MANUAL_INJECT 0x3ff
#define SDR50_MANUAL_INJECT  0x77f

#define  TRUNING_RING_IDX(x)  ((x)% TUNING_PHASE_SIZE)
#define  GET_IDX_VALUE(tb, x)  (tb &(1 << (x)))
#define  GENERATE_IDX_VALUE( x)  (1 << (x))
#define  GET_TRUNING_RING_IDX_VALUE(tb, x)  (tb &(1 << TRUNING_RING_IDX(x)))
#define  GENERATE_TRUNING_RING_IDX_VALUE( x)  (1 << TRUNING_RING_IDX(x))

// tb list
u32 all_selb_failed_tb_get(struct sdhci_host *host, int sela);
void all_selb_failed_tb_update(struct sdhci_host *host, int sela, u32 val);

u32 tx_selb_failed_tb_get(struct sdhci_host *host, int sela);
void tx_selb_failed_tb_update(struct sdhci_host *host, int sela, u32 val);

u32 tx_selb_failed_history_get(struct sdhci_host *host);
void tx_selb_failed_history_update(struct sdhci_host *host , u32 val);

void _ggc_reset_tuning_result_for_dll(struct sdhci_host * host);
void ggc_reset_selx_failed_tb(struct sdhci_host *host);
//dbg dump
void vct_2_string(u8 * tb, u32 n);
void dump_selx_vct_with_comment(char * info, u32 vct);
//func
int get_best_window_phase(u32 vct, int * max_pass_win)  ;
int get_best_window_phase_shift_left(u32 vct, int * pmax_pass_win);
bool  get_refine_sel(struct sdhci_host *host, u32 tga, u32 tgb, u32 * rfa, u32 * rfb);
bool selx_failure_point_exist(u32 val);
int get_selx_weight(u32  val);
void tx_selb_calculate_valid_phase_range(u32 val, int* start, int *pass_cnt);
int get_selb_failure_point(int start, u64 raw_tx_selb, int tuning_cnt);
void tx_selb_inject_policy(struct sdhci_host *host, int tx_selb);
void ggc_tuning_result_reset(struct sdhci_host *host); //TQY
void set_gg_reg_cur_val(u8 * data);
void get_gg_reg_cur_val(u8 * data);
void ggc_dll_voltage_init(struct sdhci_host *  host);
void ggc_chip_init(struct sdhci_host *host);

#ifdef CONFIG_MMC_SDHCI_MSM_BH201
int sdhci_bht_execute_tuning(struct sdhci_host *host, u32 opcode);
#else
inline int sdhci_bht_execute_tuning(struct sdhci_host *host, u32 opcode)
{
	return 0;
}
#endif

//0: error, 1: error+info
#define ERROR 		(0)
#define INFO 		(1)

#define Print_Level ERROR

#define DbgErr pr_err

#if (Print_Level >= INFO)
	#define DbgInfo  pr_info
#else
	#define DbgInfo(...) do{}while(0)
#endif

#endif //__SDHCI_MSM_BH201_H__
