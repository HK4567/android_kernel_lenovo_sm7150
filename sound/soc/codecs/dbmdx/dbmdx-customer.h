/*
 * dbmdx-customer.h  --  DBMDX customer api
 *
 * Copyright (C) 2014 DSP Group
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _DBMDX_CUSTOMER_API_H
#define _DBMDX_CUSTOMER_API_H

#include "dbmdx-interface.h"

unsigned long customer_dbmdx_clk_get_rate(struct dbmdx_private *p,
					enum dbmdx_clocks clk);
long customer_dbmdx_clk_set_rate(struct dbmdx_private *p,
				enum dbmdx_clocks clk);
int customer_dbmdx_clk_enable(struct dbmdx_private *p, enum dbmdx_clocks clk);
void customer_dbmdx_clk_disable(struct dbmdx_private *p, enum dbmdx_clocks clk);
void dbmdx_uart_clk_enable(struct dbmdx_private *p, bool enable);

int customer_dbmdx_va_chip_probe(struct dbmdx_private *p);
int customer_dbmdx_common_probe(struct dbmdx_private *p);

#ifdef DBMDX_VA_VE_SUPPORT

int customer_dbmdx_va_ve_chip_probe(struct dbmdx_private *p);

int customer_dbmdx_load_va_ve_fw_on_exit(struct dbmdx_private *p);

int customer_dbmdx_idle_usecase_config_on_init(struct dbmdx_private *p);

int customer_dbmdx_idle_usecase_config_on_exit(struct dbmdx_private *p);

int customer_dbmdx_send_usecase_config_on_init(struct dbmdx_private *p,
						struct usecase_config *usecase);

int customer_dbmdx_send_usecase_config_on_va_reg_cfg(struct dbmdx_private *p,
						struct usecase_config *usecase);

int customer_dbmdx_send_usecase_config_on_va_ve_reg_cfg(
						struct dbmdx_private *p,
						struct usecase_config *usecase);

int customer_dbmdx_send_usecase_config_on_pre_tdm(struct dbmdx_private *p,
						struct usecase_config *usecase);

int customer_dbmdx_send_usecase_config_on_post_tdm(struct dbmdx_private *p,
						struct usecase_config *usecase);

int customer_dbmdx_send_usecase_config_on_post_tdm_va_ve_reg(
						struct dbmdx_private *p,
						struct usecase_config *usecase);

int customer_dbmdx_send_usecase_config_on_post_tdm_va_reg(
						struct dbmdx_private *p,
						struct usecase_config *usecase);

int customer_dbmdx_send_usecase_config_on_exit(struct dbmdx_private *p,
						struct usecase_config *usecase);

int customer_dbmdx_start_usecase_on_init(struct dbmdx_private *p,
						struct usecase_config *usecase);

int customer_dbmdx_start_usecase_on_exit(struct dbmdx_private *p,
						struct usecase_config *usecase);

int customer_dbmdx_start_usecase_in_mode_on_init(struct dbmdx_private *p,
						struct usecase_config *usecase);

int customer_dbmdx_start_usecase_in_mode_on_exit(struct dbmdx_private *p,
						struct usecase_config *usecase);

int customer_dbmdx_va_pre_init(struct dbmdx_private *p);

int customer_dbmdx_va_post_init(struct dbmdx_private *p);

int customer_dbmdx_va_ve_pre_init(struct dbmdx_private *p);

int customer_dbmdx_va_ve_post_init(struct dbmdx_private *p);

#endif

#endif
