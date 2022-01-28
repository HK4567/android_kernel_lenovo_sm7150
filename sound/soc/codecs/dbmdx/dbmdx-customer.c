/*
 * DSPG DBMDX codec driver customer interface
 *
 * Copyright (C) 2014 DSP Group
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define DEBUG

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/module.h>

#include "dbmdx-customer.h"
#ifdef DBMDX_VA_VE_SUPPORT
#include "dbmdx-usecase-config-def.h"
#endif


#ifdef CONFIG_OF
#include <linux/of_gpio.h>
#endif /* CONFIG_OF */


unsigned long customer_dbmdx_clk_get_rate(
		struct dbmdx_private *p, enum dbmdx_clocks clk)
{
	dev_dbg(p->dev, "%s: %s\n",
		__func__,
		clk == DBMDX_CLK_CONSTANT ? "constant" : "master");
	return 0;
}
EXPORT_SYMBOL(customer_dbmdx_clk_get_rate);

long customer_dbmdx_clk_set_rate(
		struct dbmdx_private *p, enum dbmdx_clocks clk)
{
	dev_dbg(p->dev, "%s: %s\n",
		__func__,
		clk == DBMDX_CLK_CONSTANT ? "constant" : "master");
	return 0;
}
EXPORT_SYMBOL(customer_dbmdx_clk_set_rate);

int customer_dbmdx_clk_enable(struct dbmdx_private *p, enum dbmdx_clocks clk)
{
	dev_dbg(p->dev, "%s: %s\n",
		__func__,
		clk == DBMDX_CLK_CONSTANT ? "constant" : "master");
	return 0;
}
EXPORT_SYMBOL(customer_dbmdx_clk_enable);

void customer_dbmdx_clk_disable(struct dbmdx_private *p, enum dbmdx_clocks clk)
{
	dev_dbg(p->dev, "%s: %s\n",
		__func__,
		clk == DBMDX_CLK_CONSTANT ? "constant" : "master");
}
EXPORT_SYMBOL(customer_dbmdx_clk_disable);

int customer_dbmdx_va_chip_probe(struct dbmdx_private *p)
{
	dev_dbg(p->dev, "%s\n",	__func__);
	return 0;

}
EXPORT_SYMBOL(customer_dbmdx_va_chip_probe);

int customer_dbmdx_common_probe(struct dbmdx_private *p)
{
	dev_dbg(p->dev, "%s\n",	__func__);
	return 0;
}
EXPORT_SYMBOL(customer_dbmdx_common_probe);

#ifdef DBMDX_VA_VE_SUPPORT

int customer_dbmdx_load_va_ve_fw_on_exit(struct dbmdx_private *p)
{
	dev_dbg(p->dev, "%s\n",	__func__);
	return 0;
}
EXPORT_SYMBOL(customer_dbmdx_load_va_ve_fw_on_exit);

int customer_dbmdx_va_ve_chip_probe(struct dbmdx_private *p)
{
	dev_dbg(p->dev, "%s\n",	__func__);
	return 0;
}
EXPORT_SYMBOL(customer_dbmdx_va_ve_chip_probe);

int customer_dbmdx_idle_usecase_config_on_init(struct dbmdx_private *p)
{
	dev_dbg(p->dev, "%s\n",	__func__);
	return 0;
}
EXPORT_SYMBOL(customer_dbmdx_idle_usecase_config_on_init);

int customer_dbmdx_idle_usecase_config_on_exit(struct dbmdx_private *p)
{
	dev_dbg(p->dev, "%s\n",	__func__);
	return 0;

}
EXPORT_SYMBOL(customer_dbmdx_idle_usecase_config_on_exit);


int customer_dbmdx_send_usecase_config_on_init(struct dbmdx_private *p,
						struct usecase_config *usecase)
{
	dev_dbg(p->dev, "%s\n",	__func__);
	return 0;
}
EXPORT_SYMBOL(customer_dbmdx_send_usecase_config_on_init);

int customer_dbmdx_send_usecase_config_on_va_reg_cfg(struct dbmdx_private *p,
						struct usecase_config *usecase)
{
	dev_dbg(p->dev, "%s\n",	__func__);
	return 0;
}
EXPORT_SYMBOL(customer_dbmdx_send_usecase_config_on_va_reg_cfg);

int customer_dbmdx_send_usecase_config_on_va_ve_reg_cfg(struct dbmdx_private *p,
						struct usecase_config *usecase)
{
	dev_dbg(p->dev, "%s\n",	__func__);
	return 0;
}
EXPORT_SYMBOL(customer_dbmdx_send_usecase_config_on_va_ve_reg_cfg);

int customer_dbmdx_send_usecase_config_on_pre_tdm(struct dbmdx_private *p,
						struct usecase_config *usecase)
{
	dev_dbg(p->dev, "%s\n",	__func__);
	return 0;
}
EXPORT_SYMBOL(customer_dbmdx_send_usecase_config_on_pre_tdm);

int customer_dbmdx_send_usecase_config_on_post_tdm(struct dbmdx_private *p,
						struct usecase_config *usecase)
{
	dev_dbg(p->dev, "%s\n",	__func__);
	return 0;
}
EXPORT_SYMBOL(customer_dbmdx_send_usecase_config_on_post_tdm);

int customer_dbmdx_send_usecase_config_on_post_tdm_va_ve_reg(
						struct dbmdx_private *p,
						struct usecase_config *usecase)
{
	dev_dbg(p->dev, "%s\n",	__func__);
	return 0;
}
EXPORT_SYMBOL(customer_dbmdx_send_usecase_config_on_post_tdm_va_ve_reg);

int customer_dbmdx_send_usecase_config_on_post_tdm_va_reg(
						struct dbmdx_private *p,
						struct usecase_config *usecase)
{
	dev_dbg(p->dev, "%s\n",	__func__);
	return 0;
}
EXPORT_SYMBOL(customer_dbmdx_send_usecase_config_on_post_tdm_va_reg);

int customer_dbmdx_send_usecase_config_on_exit(struct dbmdx_private *p,
						struct usecase_config *usecase)
{
	dev_dbg(p->dev, "%s\n",	__func__);
	return 0;
}
EXPORT_SYMBOL(customer_dbmdx_send_usecase_config_on_exit);

int customer_dbmdx_start_usecase_on_init(struct dbmdx_private *p,
						struct usecase_config *usecase)
{
	dev_dbg(p->dev, "%s\n",	__func__);
	return 0;
}
EXPORT_SYMBOL(customer_dbmdx_start_usecase_on_init);

int customer_dbmdx_start_usecase_on_exit(struct dbmdx_private *p,
						struct usecase_config *usecase)
{
	dev_dbg(p->dev, "%s\n",	__func__);
	return 0;
}
EXPORT_SYMBOL(customer_dbmdx_start_usecase_on_exit);

int customer_dbmdx_start_usecase_in_mode_on_init(struct dbmdx_private *p,
						struct usecase_config *usecase)
{
	dev_dbg(p->dev, "%s\n",	__func__);
	return 0;
}
EXPORT_SYMBOL(customer_dbmdx_start_usecase_in_mode_on_init);

int customer_dbmdx_start_usecase_in_mode_on_exit(struct dbmdx_private *p,
						struct usecase_config *usecase)
{
	dev_dbg(p->dev, "%s\n",	__func__);
	return 0;
}
EXPORT_SYMBOL(customer_dbmdx_start_usecase_in_mode_on_exit);

int customer_dbmdx_va_pre_init(struct dbmdx_private *p)
{
	dev_dbg(p->dev, "%s\n",	__func__);
	return 0;
}
EXPORT_SYMBOL(customer_dbmdx_va_pre_init);

int customer_dbmdx_va_post_init(struct dbmdx_private *p)
{
	dev_dbg(p->dev, "%s\n",	__func__);
	return 0;
}
EXPORT_SYMBOL(customer_dbmdx_va_post_init);

int customer_dbmdx_va_ve_pre_init(struct dbmdx_private *p)
{
	dev_dbg(p->dev, "%s\n",	__func__);
	return 0;
}
EXPORT_SYMBOL(customer_dbmdx_va_ve_pre_init);

int customer_dbmdx_va_ve_post_init(struct dbmdx_private *p)
{
	dev_dbg(p->dev, "%s\n",	__func__);
	return 0;
}
EXPORT_SYMBOL(customer_dbmdx_va_ve_post_init);

#endif
