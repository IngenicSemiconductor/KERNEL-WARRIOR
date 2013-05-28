/*
 * drivers/power/ricoh618-battery.c
 *
 * Charger driver for RICOH RN5T618 power management chip.
 *
 * Copyright (C) 2012-2013 RICOH COMPANY,LTD
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/string.h>
#include <linux/power_supply.h>
#include <linux/mfd/ricoh618.h>
#include <linux/mfd/pmu-common.h>
#include <linux/power/ricoh618_battery.h>
#include <linux/power/ricoh618_battery_init.h>
#include <linux/delay.h>
#include <linux/workqueue.h>

//define for function
//if FG function is enabled, please define below item.
#define FUEL_GAGE_FUNCTION_ENABLE
/* #define ENABLE_LOW_BATTERY_DETECTION */
#define ENABLE_FACTORY_MODE
#define ENABLE_FG_KEEP_ON_MODE
/* #define ENABLE_INTERRUPT_IN_SLEEP */

/* define for current limit. unit is mA */
#define RICOH618_MAX_CHARGE_CURRENT	0
#define RICOH618_MAX_ADP_CURRENT	0
#define RICOH618_MAX_USB_CURRENT	0
#define RICOH618_CHARGE_COMPLETION_CURRENT	200	/* mA Range 50~200
							 * (Step 50) */
#define RICOH618_FULL_CHARGING_VOLTAGE		0	/* mv can set 4050,
							 * 4100, 4150, 4200,
							 * 4350(default 4100) */
#define	RICOH618_RE_CHARGING_VOLTAGE		0	/* mv can set 3850,
							 * 3900, 3950, 4000,
							 * 4100(default 3900) */


/* FG setting */
#define CUTOFF_VOL              3600   /* mV "0" means cutoff
                            * voltage = original
                            * OCV table value */
#define RICOH618_REL1_SEL_VALUE         64  /* mv Range 0~240
                            * (Step 16) */

enum int_type {
	SYS_INT  = 0x01,
	DCDC_INT = 0x02,
	ADC_INT  = 0x08,
	GPIO_INT = 0x10,
	CHG_INT	 = 0x40,
};

#ifdef FUEL_GAGE_FUNCTION_ENABLE
/* define for FG parameter */
#define RICOH618_MONITOR_START_TIME		15
#define RICOH618_FG_RESET_TIME			6
#define RICOH618_FG_STABLE_TIME			120
#define RICOH618_DISPLAY_UPDATE_TIME		60
#define RICOH618_SOCA_DISP_UPDATE_TIME		60
#define RICOH618_MAX_RESET_SOC_DIFF		5

/* define for FG status */
enum {
	RICOH618_SOCA_START,
	RICOH618_SOCA_UNSTABLE,
	RICOH618_SOCA_FG_RESET,
	RICOH618_SOCA_DISP,
	RICOH618_SOCA_STABLE,
	RICOH618_SOCA_ZERO,
};
#endif

#ifdef ENABLE_LOW_BATTERY_DETECTION
#define LOW_BATTERY_DETECTION_TIME		10
#endif

struct ricoh618_soca_info {
	int Rbat;
	int n_cap;
	int ocv_table[11];
	int soc;		/* Latest FG SOC value */
	int displayed_soc;
	int status;		/* SOCA status 0: Not initial; 5: Finished */
	int stable_count;
	int chg_status;		/* chg_status */
	int soc_delta;		/* soc delta for status3(DISP) */
	int cc_delta;
	int last_soc;
	int ready_fg;
	int reset_count;
	int reset_soc[3];
};


struct ricoh618_battery_info {
	struct device      *dev;
	struct power_supply	battery;
	struct power_supply	ac;
	struct power_supply	usb;
	struct delayed_work	monitor_work;
	struct delayed_work	displayed_work;
	struct delayed_work	charge_stable_work;

	struct delayed_work	changed_work;
	struct workqueue_struct * monitor_wqueue;

#ifdef ENABLE_LOW_BATTERY_DETECTION
	struct delayed_work	low_battery_work;
#endif
	struct work_struct	irq_work;	/* for Charging & VADP */
	struct work_struct	usb_irq_work;	/* for VUSB */

	struct workqueue_struct *workqueue;	/* for Charging & VUSB/VADP */
#ifdef ENABLE_FACTORY_MODE
	struct delayed_work	factory_mode_work;
	struct workqueue_struct *factory_mode_wqueue;
#endif


	struct mutex		lock;
	unsigned long       monitor_time;
	int		irq_ac;
	int		irq_usb;
	int		adc_vdd_mv;
	int		multiple;
	int		alarm_vol_mv;
	int		status;
	int		min_voltage;
	int		max_voltage;
	int		cur_voltage;
	int		capacity;
	int		battery_temp;
	int		time_to_empty;
	int		time_to_full;
	int		chg_ctr;
	int		chg_stat1;
	unsigned	present:1;
	u16		delay;
	struct		ricoh618_soca_info *soca;
	int		first_pwon;
	bool		entry_factory_mode;
};
int g_soc;
int g_fg_on_mode;

static void ricoh618_battery_work(struct work_struct *work)
{
	struct ricoh618_battery_info *info = container_of(work,
	struct ricoh618_battery_info, monitor_work.work);

	power_supply_changed(&info->battery);
	queue_delayed_work(info->monitor_wqueue, &info->monitor_work,
			   info->monitor_time);
}

#ifdef FUEL_GAGE_FUNCTION_ENABLE
static int measure_vbatt_FG(struct ricoh618_battery_info *info, int *data);
static int measure_Ibatt_FG(struct ricoh618_battery_info *info, int *data);
static int calc_capacity(struct ricoh618_battery_info *info);
static int get_OCV_init_Data(struct ricoh618_battery_info *info, int index);
static int get_OCV_voltage(struct ricoh618_battery_info *info, int index);
static int get_check_fuel_gauge_reg(struct ricoh618_battery_info *info,
					 int Reg_h, int Reg_l, int enable_bit);
static int calc_capacity_in_period(struct ricoh618_battery_info *info,
					 int *cc_cap, bool *is_charging);

/* check charge status.
 * if CHG not Complete && SOC == 100 -> Stop charge
 * if CHG Complete && SOC =! 100     -> SOC reset
 */
static int check_charge_status(struct ricoh618_battery_info *info)
{
	uint8_t status;
	uint8_t supply_state;
	uint8_t charge_state;
	int ret = 0;
	int current_SOC;

	/* get  power supply status */
	ret = ricoh618_read(info->dev->parent, CHGSTATE_REG, &status);
	if (ret < 0) {
		dev_err(info->dev,
			 "Error in reading the control register\n");
		return ret;
	}


	charge_state = (status & 0x1F);
	supply_state = ((status & 0xC0) >> 6);

	current_SOC = (info->soca->displayed_soc + 50)/100;

	if (charge_state == CHG_STATE_CHG_COMPLETE) {
		/* check SOC */
		if (current_SOC != 100) {
			ret = ricoh618_write(info->dev->parent,
						 FG_CTRL_REG, 0x51);
			if (ret < 0) {
				dev_err(info->dev, "Error in writing the control register\n");
				return ret;
			}
			info->soca->ready_fg = 0;

			info->soca->displayed_soc = 100 * 100;

			info->soca->status = RICOH618_SOCA_STABLE;
		}

	} else {	/* chg not complete */
		if (current_SOC == 100) {
			ret = ricoh618_clr_bits(info->dev->parent,
						 CHGCTL1_REG, 0x03);
			if (ret < 0) {
				dev_err(info->dev, "Error in writing the control register\n");
				return ret;
			}
			info->soca->status = RICOH618_SOCA_STABLE;
		} else {
			ret = ricoh618_set_bits(info->dev->parent,
						 CHGCTL1_REG, 0x03);
			if (ret < 0) {
				dev_err(info->dev, "Error in writing the control register\n");
				return ret;
			}
		}
	}
	return ret;
}

static int calc_ocv(struct ricoh618_battery_info *info)
{
	int Vbat = 0;
	int Ibat = 0;
	int ret;
	int ocv;

	ret = measure_vbatt_FG(info, &Vbat);
	ret = measure_Ibatt_FG(info, &Ibat);

	ocv = Vbat - Ibat * info->soca->Rbat;

	return ocv;
}

/**
* Calculate Capacity in a period
* - read CC_SUM & FA_CAP from Coulom Counter
* -  and calculate Capacity.
* @cc_cap: capacity in a period, unit 0.01%
* @is_charging: Flag of charging current direction
*               TRUE : charging (plus)
*               FALSE: discharging (minus)
**/
static int calc_capacity_in_period(struct ricoh618_battery_info *info,
					 int *cc_cap, bool *is_charging)
{
	int err;
	uint8_t cc_sum_reg[4];
	uint8_t cc_clr[4] = {0, 0, 0, 0};
	uint8_t fa_cap_reg[2];
	uint16_t fa_cap;
	uint32_t cc_sum;

	*is_charging = true;	/* currrent state initialize -> charging */

	if (info->entry_factory_mode)
		return 0;

	/* Disable Charging/Completion Interrupt */
	err = ricoh618_set_bits(info->dev->parent,
					RICOH618_INT_MSK_CHGSTS1, 0x01);
	if (err < 0)
		goto out;

	/* In suspend - disable charging */
	err = ricoh618_set_bits(info->dev->parent, RICOH618_CHG_CTL1, 0x08);
	if (err < 0)
		goto out;
	/* CC_pause enter */
	err = ricoh618_write(info->dev->parent, CC_CTRL_REG, 0x01);
	if (err < 0)
		goto out;
	/* Read CC_SUM */
	err = ricoh618_bulk_reads(info->dev->parent,
					CC_SUMREG3_REG, 4, cc_sum_reg);
	if (err < 0)
		goto out;

	/* CC_SUM <- 0 */
	err = ricoh618_bulk_writes(info->dev->parent,
					CC_SUMREG3_REG, 4, cc_clr);
	if (err < 0)
		goto out;

	/* CC_pause exist */
	err = ricoh618_write(info->dev->parent, CC_CTRL_REG, 0);
	if (err < 0)
		goto out;
	/* out suspend - enable charging */
	err = ricoh618_clr_bits(info->dev->parent, RICOH618_CHG_CTL1, 0x08);
	if (err < 0)
		goto out;

	udelay(1000);

	/* Clear Charging Interrupt status */
	err = ricoh618_clr_bits(info->dev->parent,
					RICOH618_INT_IR_CHGSTS1, 0x01);
	if (err < 0)
		goto out;

	/* ricoh618_read(info->dev->parent, RICOH618_INT_IR_CHGSTS1, &val);
	printk("INT_IR_CHGSTS1 = 0x%x\n",val); */

	/* Enable Charging Interrupt */
	err = ricoh618_clr_bits(info->dev->parent,
					RICOH618_INT_MSK_CHGSTS1, 0x01);
	if (err < 0)
		goto out;

	/* Read FA_CAP */
	err = ricoh618_bulk_reads(info->dev->parent,
				 FA_CAP_H_REG, 2, fa_cap_reg);
	if (err < 0)
		goto out;

	/* fa_cap = *(uint16_t*)fa_cap_reg & 0x7fff; */
	fa_cap = (fa_cap_reg[0] << 8 | fa_cap_reg[1]) & 0x7fff;

	/* calculation  two's complement of CC_SUM */
	cc_sum = cc_sum_reg[0] << 24 | cc_sum_reg[1] << 16 |
				cc_sum_reg[2] << 8 | cc_sum_reg[3];

	/* cc_sum = *(uint32_t*)cc_sum_reg; */
	if (cc_sum & 0x80000000) {
		cc_sum = (cc_sum^0xffffffff)+0x01;
		*is_charging = false;		/* discharge */
	}

	*cc_cap = cc_sum*25/9/fa_cap;	/* CC_SUM/3600/FA_CAP */

	return 0;
out:
	dev_err(info->dev, "Error !!-----\n");
	return err;
}

static void ricoh618_displayed_work(struct work_struct *work)
{
	int err;
	uint8_t val;
	int soc_round;
	int last_soc_round;
	int last_disp_round;
	int displayed_soc_temp;
	int cc_cap = 0;
	bool is_charging = true;

	struct ricoh618_battery_info *info = container_of(work,
	struct ricoh618_battery_info, displayed_work.work);

	if (info->entry_factory_mode) {
		info->soca->status = RICOH618_SOCA_STABLE;
		info->soca->displayed_soc = -EINVAL;
		info->soca->ready_fg = 0;
		return;
	}

	mutex_lock(&info->lock);

	if ((RICOH618_SOCA_START == info->soca->status)
		 || (RICOH618_SOCA_STABLE == info->soca->status))
		info->soca->ready_fg = 1;

	if (RICOH618_SOCA_STABLE == info->soca->status) {
		info->soca->soc = calc_capacity(info) * 100;
		info->soca->displayed_soc = info->soca->soc;
	} else if (RICOH618_SOCA_DISP == info->soca->status) {

		info->soca->soc = calc_capacity(info) * 100;

		soc_round = info->soca->soc / 100;
		last_soc_round = info->soca->last_soc / 100;
		last_disp_round = (info->soca->displayed_soc + 50) / 100;

		info->soca->soc_delta =
			info->soca->soc_delta + (soc_round - last_soc_round);

		info->soca->last_soc = info->soca->soc;

		/* six case */
		if (last_disp_round == soc_round) {
			/* if SOC == DISPLAY move to stable */
			info->soca->displayed_soc = info->soca->soc ;
			info->soca->status = RICOH618_SOCA_STABLE;

		} else if ((soc_round == 100) || (soc_round == 0)) {
			/* if SOC is 0% or 100% , finish display state*/
			info->soca->displayed_soc = info->soca->soc ;
			info->soca->status = RICOH618_SOCA_STABLE;

		} else if ((info->soca->chg_status) ==
				(POWER_SUPPLY_STATUS_CHARGING)) {
			/* Charge */
			if (last_disp_round < soc_round) {
				/* Case 1 : Charge, Display < SOC */
				if (info->soca->soc_delta >= 1) {
					info->soca->displayed_soc
						= (last_disp_round
						+ info->soca->soc_delta)*100;
					info->soca->soc_delta = 0;
				} else {
					info->soca->displayed_soc
						= (last_disp_round + 1)*100;
				}

				if (last_disp_round >= soc_round) {
					info->soca->displayed_soc
						= info->soca->soc ;
					info->soca->status
						= RICOH618_SOCA_STABLE;
				}
			} else if (last_disp_round > soc_round) {
				/* Case 2 : Charge, Display > SOC */
				if (info->soca->soc_delta >= 3) {
					info->soca->displayed_soc =
						(last_disp_round + 1)*100;
					info->soca->soc_delta = 0;
				}
				if (last_disp_round <= soc_round) {
					info->soca->displayed_soc
						= info->soca->soc ;
					info->soca->status
					= RICOH618_SOCA_STABLE;
				}
			}
		} else {
			/* Dis-Charge */
			if (last_disp_round > soc_round) {
				/* Case 3 : Dis-Charge, Display > SOC */
				if (info->soca->soc_delta <= -1) {
					info->soca->displayed_soc
						= (last_disp_round
						+ info->soca->soc_delta)*100;
					info->soca->soc_delta = 0;
				} else {
					info->soca->displayed_soc
						= (last_disp_round - 1)*100;
				}
				if (last_disp_round <= soc_round) {
					info->soca->displayed_soc
						= info->soca->soc ;
					info->soca->status
						= RICOH618_SOCA_STABLE;
				}
			} else if (last_disp_round < soc_round) {
				/* dis Charge, Display < SOC */
				if (info->soca->soc_delta <= -3) {
					info->soca->displayed_soc
						= (last_disp_round - 1)*100;
					info->soca->soc_delta = 0;
				}
				if (last_disp_round >= soc_round) {
					info->soca->displayed_soc
						= info->soca->soc ;
					info->soca->status
						= RICOH618_SOCA_STABLE;
				}
			}
		}
	} else if (RICOH618_SOCA_UNSTABLE == info->soca->status
		 || RICOH618_SOCA_FG_RESET == info->soca->status) {
		/* No update */
	} else if (RICOH618_SOCA_START == info->soca->status) {
		err = ricoh618_read(info->dev->parent, PSWR_REG, &val);
		val &= 0x7f;
		if (info->first_pwon) {
			info->soca->soc = calc_capacity(info) * 100;
			if ((info->soca->soc == 0) && (calc_ocv(info)
					< get_OCV_voltage(info, 0))) {
				info->soca->displayed_soc = 0;
				info->soca->status = RICOH618_SOCA_ZERO;
			} else {
				info->soca->displayed_soc = info->soca->soc;
				info->soca->status = RICOH618_SOCA_UNSTABLE;
			}
		} else if (g_fg_on_mode && (val == 0x7f)) {
			info->soca->soc = calc_capacity(info) * 100;
			if ((info->soca->soc == 0) && (calc_ocv(info)
					< get_OCV_voltage(info, 0))) {
				info->soca->displayed_soc = 0;
				info->soca->status = RICOH618_SOCA_ZERO;
			} else {
				info->soca->displayed_soc = info->soca->soc;
				info->soca->status = RICOH618_SOCA_STABLE;
			}
		} else {
			info->soca->soc = val * 100;
			if ((err < 0) || (val == 0)) {
				dev_err(info->dev,
					 "Error in reading PSWR_REG %d\n", err);
				info->soca->soc
					 = calc_capacity(info) * 100 + 50;
			}

			err = calc_capacity_in_period(info, &cc_cap,
								 &is_charging);
			if (err < 0)
				dev_err(info->dev, "Read cc_sum Error !!-----\n");

			info->soca->cc_delta
				 = (is_charging == true) ? cc_cap : -cc_cap;
			if (calc_ocv(info) < get_OCV_voltage(info, 0)) {
				info->soca->displayed_soc = 0;
				info->soca->status = RICOH618_SOCA_ZERO;
			} else {
				displayed_soc_temp
				       = info->soca->soc + info->soca->cc_delta;
				displayed_soc_temp
					 = min(10000, displayed_soc_temp);
				displayed_soc_temp = max(0, displayed_soc_temp);
				info->soca->displayed_soc = displayed_soc_temp;
				info->soca->status = RICOH618_SOCA_UNSTABLE;
			}
		}
	} else if (RICOH618_SOCA_ZERO == info->soca->status) {
		if (calc_ocv(info) > get_OCV_voltage(info, 0)) {
			err = ricoh618_write(info->dev->parent,
							 FG_CTRL_REG, 0x51);
			if (err < 0)
				dev_err(info->dev, "Error in writing the control register\n");
			info->soca->status = RICOH618_SOCA_STABLE;
			info->soca->ready_fg = 0;
		}
		info->soca->displayed_soc = 0;
	}

	/* Ceck charge status */
	err = check_charge_status(info);
	if (err < 0)
		dev_err(info->dev, "Error in writing the control register\n");

	if (g_fg_on_mode
		 && (info->soca->status == RICOH618_SOCA_STABLE)) {
		err = ricoh618_write(info->dev->parent, PSWR_REG, 0x7f);
		if (err < 0)
			dev_err(info->dev, "Error in writing PSWR_REG\n");
		g_soc = 0x7F;
	} else {
		val = (info->soca->displayed_soc + 50)/100;
		val &= 0x7f;
		err = ricoh618_write(info->dev->parent, PSWR_REG, val);
		if (err < 0)
			dev_err(info->dev, "Error in writing PSWR_REG\n");

		g_soc = (info->soca->displayed_soc + 50)/100;

		err = calc_capacity_in_period(info, &cc_cap, &is_charging);
		if (err < 0)
			dev_err(info->dev, "Read cc_sum Error !!-----\n");
	}

	if (0 == info->soca->ready_fg)
		queue_delayed_work(info->monitor_wqueue, &info->displayed_work,
					 RICOH618_FG_RESET_TIME * HZ);
	else if (RICOH618_SOCA_DISP == info->soca->status)
		queue_delayed_work(info->monitor_wqueue, &info->displayed_work,
					 RICOH618_SOCA_DISP_UPDATE_TIME * HZ);
	else
		queue_delayed_work(info->monitor_wqueue, &info->displayed_work,
					 RICOH618_DISPLAY_UPDATE_TIME * HZ);

	mutex_unlock(&info->lock);

	return;
}

static void ricoh618_stable_charge_countdown_work(struct work_struct *work)
{
	int ret;
	int max = 0;
	int min = 100;
	int i;
	struct ricoh618_battery_info *info = container_of(work,
		struct ricoh618_battery_info, charge_stable_work.work);

	if (info->entry_factory_mode)
		return;

	mutex_lock(&info->lock);
	if (RICOH618_SOCA_FG_RESET == info->soca->status)
		info->soca->ready_fg = 1;

	if (2 <= info->soca->stable_count) {
		if (3 == info->soca->stable_count
			&& RICOH618_SOCA_FG_RESET == info->soca->status) {
			ret = ricoh618_write(info->dev->parent,
							 FG_CTRL_REG, 0x51);
			if (ret < 0)
				dev_err(info->dev, "Error in writing the control register\n");
			info->soca->ready_fg = 0;
		}
		info->soca->stable_count = info->soca->stable_count - 1;
		queue_delayed_work(info->monitor_wqueue,
					 &info->charge_stable_work,
					 RICOH618_FG_STABLE_TIME * HZ / 10);
	} else if (0 >= info->soca->stable_count) {
		/* Finished queue, ignore */
	} else if (1 == info->soca->stable_count) {
		if (RICOH618_SOCA_UNSTABLE == info->soca->status) {
			/* Judge if FG need reset or Not */
			info->soca->soc = calc_capacity(info) * 100;
			if (info->chg_ctr != 0) {
				queue_delayed_work(info->monitor_wqueue,
					 &info->charge_stable_work,
					 RICOH618_FG_STABLE_TIME * HZ / 10);
				mutex_unlock(&info->lock);
				return;
			}
			/* Do reset setting */
			ret = ricoh618_write(info->dev->parent,
						 FG_CTRL_REG, 0x51);
			if (ret < 0)
				dev_err(info->dev, "Error in writing the control register\n");

			info->soca->ready_fg = 0;
			info->soca->status = RICOH618_SOCA_FG_RESET;

			/* Delay for addition Reset Time (6s) */
			queue_delayed_work(info->monitor_wqueue,
					 &info->charge_stable_work,
					 RICOH618_FG_RESET_TIME*HZ);
		} else if (RICOH618_SOCA_FG_RESET == info->soca->status) {
			info->soca->reset_soc[2] = info->soca->reset_soc[1];
			info->soca->reset_soc[1] = info->soca->reset_soc[0];
			info->soca->reset_soc[0] = calc_capacity(info) * 100;
			info->soca->reset_count++;

			if (info->soca->reset_count > 10) {
				/* Reset finished; */
				info->soca->soc = info->soca->reset_soc[0];
				info->soca->stable_count = 0;
				goto adjust;
			}

			for (i = 0; i < 3; i++) {
				if (max < info->soca->reset_soc[i]/100)
					max = info->soca->reset_soc[i]/100;
				if (min > info->soca->reset_soc[i]/100)
					min = info->soca->reset_soc[i]/100;
			}

			if ((info->soca->reset_count > 3) && ((max - min)
					< RICOH618_MAX_RESET_SOC_DIFF)) {
				/* Reset finished; */
				info->soca->soc = info->soca->reset_soc[0];
				info->soca->stable_count = 0;
				goto adjust;
			} else {
				/* Do reset setting */
				ret = ricoh618_write(info->dev->parent,
							 FG_CTRL_REG, 0x51);
				if (ret < 0)
					dev_err(info->dev, "Error in writing the control register\n");

				info->soca->ready_fg = 0;

				/* Delay for addition Reset Time (6s) */
				queue_delayed_work(info->monitor_wqueue,
						 &info->charge_stable_work,
						 RICOH618_FG_RESET_TIME*HZ);
			}
		/* Finished queue From now, select FG as result; */
		} else if (RICOH618_SOCA_START == info->soca->status) {
			/* Normal condition */
		} else { /* other state ZERO/DISP/STABLE */
			info->soca->stable_count = 0;
		}

		mutex_unlock(&info->lock);
		return;

adjust:
		info->soca->last_soc = info->soca->soc;
		info->soca->status = RICOH618_SOCA_DISP;

	}
	mutex_unlock(&info->lock);
	return;
}

/* Initial setting of FuelGauge SOCA function */
static int ricoh618_init_fgsoca(struct ricoh618_battery_info *info)
{
	int i;
	int err;

	for (i = 0; i <= 10; i = i+1) {
		info->soca->ocv_table[i] = get_OCV_voltage(info, i);
		printk(KERN_INFO "PMU: %s : * %d0%% voltage = %d uV\n",
				 __func__, i, info->soca->ocv_table[i]);
	}

	if (info->first_pwon) {
		err = ricoh618_write(info->dev->parent,
						 FG_CTRL_REG, 0x51);
		if (err < 0)
			dev_err(info->dev, "Error in writing the control register\n");
	}

	/* Rbat : Transfer */
	info->soca->Rbat = get_OCV_init_Data(info, 12) * 1000 / 512
							 * 5000 / 4095;
	info->soca->n_cap = get_OCV_init_Data(info, 11);

	info->soca->displayed_soc = 0;
	info->soca->ready_fg = 0;
	info->soca->soc_delta = 0;
	info->soca->status = RICOH618_SOCA_START;
	/* stable count down 11->2, 1: reset; 0: Finished; */
	info->soca->stable_count = 11;

#ifdef ENABLE_FG_KEEP_ON_MODE
	g_fg_on_mode = 1;
#else
	g_fg_on_mode = 0;
#endif

	/* Start first Display job */
	queue_delayed_work(info->monitor_wqueue, &info->displayed_work,
						   RICOH618_FG_RESET_TIME*HZ);

	/* Start first Waiting stable job */
	queue_delayed_work(info->monitor_wqueue, &info->charge_stable_work,
		   RICOH618_FG_STABLE_TIME*HZ/10);

	printk(KERN_INFO "PMU: %s : * Rbat = %d mOhm   n_cap = %d mAH\n",
			 __func__, info->soca->Rbat, info->soca->n_cap);
	return 1;
}
#endif

static void ricoh618_changed_work(struct work_struct *work)
{
	struct ricoh618_battery_info *info = container_of(work,
	struct ricoh618_battery_info, changed_work.work);

	power_supply_changed(&info->battery);

	return;
}

#ifdef ENABLE_FACTORY_MODE
/*------------------------------------------------------*/
/* Factory Mode						*/
/*    Check Battery exist or not			*/
/*    If not, disabled Rapid to Complete State change	*/
/*------------------------------------------------------*/
static int ricoh618_factory_mode(struct ricoh618_battery_info *info)
{
	int ret = 0;
	uint8_t val = 0;

	ret = ricoh618_read(info->dev->parent, RICOH618_INT_MON_CHGCTR, &val);
	if (ret < 0) {
		dev_err(info->dev, "Error in reading the control register\n");
		return ret;
	}
	if (!(val & 0x01)) /* No Adapter connected */
		return ret;

	/* Rapid to Complete State change disable */
	ret = ricoh618_write(info->dev->parent, RICOH618_CHG_CTL1, 0xe3);
	if (ret < 0) {
		dev_err(info->dev, "Error in writing the control register\n");
		return ret;
	}

	/* Wait 1s for checking Charging State */
	queue_delayed_work(info->factory_mode_wqueue, &info->factory_mode_work,
			 1*HZ);

	return ret;
}

static void check_charging_state_work(struct work_struct *work)
{
	struct ricoh618_battery_info *info = container_of(work,
		struct ricoh618_battery_info, factory_mode_work.work);

	int ret = 0;
	uint8_t val = 0;
	int chargeCurrent = 0;

	ret = ricoh618_read(info->dev->parent, CHGSTATE_REG, &val);
	if (ret < 0) {
		dev_err(info->dev, "Error in reading the control register\n");
		return;
	}


	chargeCurrent = get_check_fuel_gauge_reg(info, CC_AVERAGE1_REG,
						 CC_AVERAGE0_REG, 0x3fff);
	if (chargeCurrent < 0) {
		dev_err(info->dev, "Error in reading the FG register\n");
		return;
	}

	/* Repid State && Charge Current about 0mA */
	if (((chargeCurrent > 0x3ffc && chargeCurrent < 0x3fff)
		|| chargeCurrent < 0x05) && val == 0x43) {
		printk(KERN_INFO "PMU:%s --- No battery !! Enter Factory mode ---\n"
				, __func__);
		info->entry_factory_mode = true;
		return;	/* Factory Mode */
	}

	/* Return Normal Mode --> Rapid to Complete State change enable */
	ret = ricoh618_write(info->dev->parent, RICOH618_CHG_CTL1, 0xa3);
	if (ret < 0) {
		dev_err(info->dev, "Error in writing the control register\n");
		return;
	}
	printk(KERN_INFO "PMU:%s --- Battery exist !! Return Normal mode ---0x%2x\n"
			, __func__, val);

	return;
}
#endif /* ENABLE_FACTORY_MODE */


static int Calc_Linear_Interpolation(int x0, int y0, int x1, int y1, int y)
{
	int	alpha;
	int x;

	alpha = (y - y0)*100 / (y1 - y0);

	x = ((100 - alpha) * x0 + alpha * x1) / 100;

	return x;
}
static int ricoh618_set_OCV_table(struct ricoh618_battery_info *info)
{
	int		ret = 0;
	int		ocv_table[11];
	int		i, j;
	int		available_cap;
	int		temp;
	int		start_par;
	int		percent_step;
	int		OCV_percent_new[11];

	printk("----------- start set OCV  \n");
	if (CUTOFF_VOL == 0) {	/* normal version */
	} else {	/*Slice cutoff voltage version. */

		/* get ocv table. this table is calculated by Apprication */
		for (i = 0; i <= 10; i = i+1) {
			temp = (battery_init_para[i*2]<<8)
				 | (battery_init_para[i*2+1]);
			/* conversion unit 1 Unit is 1.22mv (5000/4095 mv) */
			temp = ((temp * 50000 * 10 / 4095) + 5) / 10;
			ocv_table[i] = temp;
		}


		/* Check Start % */
		for (i = 1; i < 11; i++) {
			if (ocv_table[i] >= CUTOFF_VOL * 10) {
				/* unit is 0.001% */
				start_par = Calc_Linear_Interpolation(
					(i-1)*1000, ocv_table[i-1], i*1000,
					 ocv_table[i], (CUTOFF_VOL * 10));
				i = 11;
			}
		}
		/* calc new ocv percent */
		percent_step = (10000 - start_par) / 10;

		for (i = 0; i < 11; i++) {
			OCV_percent_new[i]
				 = start_par + percent_step*(i - 0);
		}

		/* calc new ocv voltage */
		for (i = 0; i < 11; i++) {
			for (j = 1; j < 11; j++) {
				if (1000*j >= OCV_percent_new[i]) {
					temp = Calc_Linear_Interpolation(
						ocv_table[j-1], (j-1)*1000,
						 ocv_table[j] , j*1000,
						 OCV_percent_new[i]);

					temp = temp * 4095 / 50000;

					battery_init_para[i*2 + 1] = temp;
					battery_init_para[i*2] = temp >> 8;

					j = 11;
				}
			}
		}

		/* calc available capacity */
		/* get avilable capacity */
		/* battery_init_para23-24 is designe capacity */
		available_cap = (battery_init_para[22]<<8)
					 | (battery_init_para[23]);

		available_cap = available_cap
			 * ((10000 - start_par) / 100) / 100 ;


		battery_init_para[23] =  available_cap;
		battery_init_para[22] =  available_cap >> 8;

	}
	ret = ricoh618_bulk_writes_bank1(info->dev->parent,
			 BAT_INIT_TOP_REG, 32, battery_init_para);
	if (ret < 0) {
		dev_err(info->dev, "batterry initialize error\n");
		return ret;
	}

	return 1;
}

//Initial setting of battery
static int ricoh618_init_battery(struct ricoh618_battery_info *info)
{
	int ret = 0;
      //Need to implement initial setting of batery and error
      //////////////////////////////
#ifdef FUEL_GAGE_FUNCTION_ENABLE
	uint8_t val = 0;
	/* set kanwa state */
	if (RICOH618_REL1_SEL_VALUE > 240)
		val = 0x0F;
	else
		val = RICOH618_REL1_SEL_VALUE / 16 ;

	val = 0x20 + val;

	ret = ricoh618_write_bank1(info->dev->parent, BAT_REL_SEL_REG, val);
	if (ret < 0) {
		dev_err(info->dev, "Error in writing the OCV Tabler\n");
		return ret;
	}


	ret = ricoh618_read(info->dev->parent, FG_CTRL_REG, &val);
	if(ret<0){
		dev_err(info->dev, "Error in reading the control register\n");
		return ret;
	}

	val = (val & 0x10) >> 4;//check FG_ACC DATA

	/*if(val == 0)
	{//set initial setting of battery
		ret = ricoh618_bulk_writes_bank1(info->dev->parent, BAT_INIT_TOP_REG, 32, battery_init_para);
		if(ret<0){
			dev_err(info->dev, "batterry initialize error\n");
			return ret;
		}

	}*/
	ret = ricoh618_set_OCV_table(info);
	if (ret < 0) {
		dev_err(info->dev, "Error in writing the OCV Tabler\n");
		return ret;
	}

	ret = ricoh618_write(info->dev->parent, FG_CTRL_REG, 0x11);
	if(ret<0){
		dev_err(info->dev, "Error in writing the control register\n");
		return ret;
	}

#endif

      //////////////////////////////

	if(info->alarm_vol_mv < 2700 || info->alarm_vol_mv > 3400){
		dev_err(info->dev, "alarm_vol_mv is out of range!\n");
		return -1;
	}

	return ret;
}

/* Initial setting of charger */
static int ricoh618_init_charger(struct ricoh618_battery_info *info)
{
	int err;
	uint8_t val;
	uint8_t val2;

	info->chg_ctr = 0;
	info->chg_stat1 = 0;

	/* In suspend - disable charging */
	err = ricoh618_clr_bits(info->dev->parent, CHGCTL1_REG, 0x03);
	if (err < 0) {
		dev_err(info->dev, "Error in writing the control register\n");
		goto free_device;
	}

	if (RICOH618_MAX_ADP_CURRENT != 0) {
		/* Change ADP Current to 2.5A. */
		err = ricoh618_write(info->dev->parent, REGISET1_REG,
					 (RICOH618_MAX_ADP_CURRENT-1)/100);
		if (err < 0) {
			dev_err(info->dev, "Error in writing INT_MSK_CHGSTS1 %d\n",
									 err);
			goto free_device;
		}
	}

	if (RICOH618_MAX_USB_CURRENT != 0) {
		/* Set Max Change USB Current (0xB7) */
		err = ricoh618_write(info->dev->parent, REGISET2_REG,
					 (RICOH618_MAX_USB_CURRENT-1)/100);
		if (err < 0) {
			dev_err(info->dev,
			 "Error in writing RICOH618_MAX_USB_CURRENT %d\n", err);
			goto free_device;
		}
	}

	/* Set Charge competion current    (0xB8) */
	/* this value for bit 4-0 */
	if (RICOH618_MAX_CHARGE_CURRENT != 0) {
		val = (RICOH618_MAX_CHARGE_CURRENT-1)/100;
	} else {
		err = ricoh618_read(info->dev->parent, CHGISET_REG, &val);
		if (err < 0) {
			dev_err(info->dev,
			"Error in read RICOH618_MAX_CHARGE_CURRENT %d\n", err);
			goto free_device;
		}
		val &= 0x3F;
	}

	/* Set Charge competion current    (0xB8) */
	/* this value for bit 7-6 */
	if (RICOH618_CHARGE_COMPLETION_CURRENT != 0) {
		val2 = (RICOH618_CHARGE_COMPLETION_CURRENT - 50) / 50;
	} else {
		err = ricoh618_read(info->dev->parent, CHGISET_REG, &val2);
		if (err < 0) {
			dev_err(info->dev,
			"Error in read RICOH618_MAX_CHARGE_CURRENT %d\n", err);
			goto free_device;
		}
		val2 &= 0xC0;
	}
	val = val + (val2 << 6);
	err = ricoh618_write(info->dev->parent, CHGISET_REG, val);
	if (err < 0) {
		dev_err(info->dev,
		 "Error in writing RICOH618_MAX_CHARGE_CURRENT %d\n", err);
		goto free_device;
	}

	/* Change Charger Voltege to 4.2V. Recharge Point to 4.1V */
	/* for define FULL charging voltage (bit 6~4)*/
	if (RICOH618_FULL_CHARGING_VOLTAGE != 0) {
		if (RICOH618_FULL_CHARGING_VOLTAGE < 4050)
			val2 = 0x00;
		else if (RICOH618_FULL_CHARGING_VOLTAGE > 4200)
			val2 = 0x04;
		else
			val2 = (RICOH618_FULL_CHARGING_VOLTAGE - 4050) / 50;
	} else {
		err = ricoh618_read(info->dev->parent, BATSET2_REG, &val2);
		if (err < 0) {
			dev_err(info->dev,
			"Error in read RICOH618_FULL_CHARGE_VOLTAGE %d\n", err);
			goto free_device;
		}
		val2 &= 0x70;
	}

	/* for define re-charging voltage (bit 2~0)*/
	if (RICOH618_RE_CHARGING_VOLTAGE != 0) {
		if (RICOH618_RE_CHARGING_VOLTAGE < 3850)
			val = 0x00;
		else if (RICOH618_RE_CHARGING_VOLTAGE > 4000)
			val = 0x04;
		else
			val = (RICOH618_RE_CHARGING_VOLTAGE - 3850) / 50;
	} else {
		err = ricoh618_read(info->dev->parent, BATSET2_REG, &val);
		if (err < 0) {
			dev_err(info->dev,
			"Error in read RICOH618_RE_CHARGE_VOLTAGE %d\n", err);
			goto free_device;
		}
		val &= 0x07;
	}

	val = val + (val2 << 4);

	err = ricoh618_write(info->dev->parent, BATSET2_REG, val);
	if (err < 0) {
		dev_err(info->dev, "Error in writing RICOH618_RE_CHARGE_VOLTAGE %d\n",
									 err);
		goto free_device;
	}

	/* out suspend - enable charging */
	err = ricoh618_set_bits(info->dev->parent, CHGCTL1_REG, 0x03);
	if (err < 0) {
		dev_err(info->dev, "Error in writing the control register\n");
		goto free_device;
	}

	/* Set rising edge setting ([1:0]=01b)for INT in charging */
	/*  and rising edge setting ([3:2]=01b)for charge completion */
	err = ricoh618_read(info->dev->parent, RICOH618_CHG_STAT_DETMOD1, &val);
	if (err < 0) {
		dev_err(info->dev, "Error in reading CHG_STAT_DETMOD1 %d\n",
								 err);
		goto free_device;
	}
	val &= 0xf0;
	val |= 0x05;
	err = ricoh618_write(info->dev->parent, RICOH618_CHG_STAT_DETMOD1, val);
	if (err < 0) {
		dev_err(info->dev, "Error in writing CHG_STAT_DETMOD1 %d\n",
								 err);
		goto free_device;
	}

	/* Unmask In charging/charge completion */
	err = ricoh618_write(info->dev->parent, RICOH618_INT_MSK_CHGSTS1, 0xfc);
	if (err < 0) {
		dev_err(info->dev, "Error in writing INT_MSK_CHGSTS1 %d\n",
								 err);
		goto free_device;
	}

	/* Set both edge for VUSB([3:2]=11b)/VADP([1:0]=11b) detect */
	err = ricoh618_read(info->dev->parent, RICOH618_CHG_CTRL_DETMOD1, &val);
	if (err < 0) {
		dev_err(info->dev, "Error in reading CHG_CTRL_DETMOD1 %d\n",
								 err);
		goto free_device;
	}
	val &= 0xf0;
	val |= 0x0f;
	err = ricoh618_write(info->dev->parent, RICOH618_CHG_CTRL_DETMOD1, val);
	if (err < 0) {
		dev_err(info->dev, "Error in writing CHG_CTRL_DETMOD1 %d\n",
								 err);
		goto free_device;
	}

	/* Unmask In VUSB/VADP completion */
	err = ricoh618_write(info->dev->parent, RICOH618_INT_MSK_CHGCTR, 0xfc);
	if (err < 0) {
		dev_err(info->dev, "Error in writing INT_MSK_CHGSTS1 %d\n",
								 err);
		goto free_device;
	}

#ifdef ENABLE_LOW_BATTERY_DETECTION
	/* Set ADRQ=00 to stop ADC */
	ricoh618_write(info->dev->parent, RICOH618_ADC_CNT3, 0x0);
	/* Enable VSYS threshold Low interrupt */
	ricoh618_write(info->dev->parent, RICOH618_INT_EN_ADC1, 0x10);
	/* Set ADC auto conversion interval 250ms */
	ricoh618_write(info->dev->parent, RICOH618_ADC_CNT2, 0x0);
	/* Enable VSYS pin conversion in auto-ADC */
	ricoh618_write(info->dev->parent, RICOH618_ADC_CNT1, 0x10);
	/* Set VSYS threshold low voltage = 3.50v */
	ricoh618_write(info->dev->parent, RICOH618_ADC_VSYS_THL, 0x77);
	/* Start auto-mode & average 4-time conversion mode for ADC */
	ricoh618_write(info->dev->parent, RICOH618_ADC_CNT3, 0x28);
	/* Enable master ADC INT */
	ricoh618_set_bits(info->dev->parent, RICOH618_INTC_INTEN, ADC_INT);
#endif

free_device:
	return err;
}


static int get_power_supply_status(struct ricoh618_battery_info *info)
{
	uint8_t status;
	uint8_t supply_state;
	uint8_t charge_state;
	int temp;
	int ret = 0;

	//get  power supply status
	ret = ricoh618_read(info->dev->parent, CHGSTATE_REG, &status);
	if(ret<0){
		dev_err(info->dev, "Error in reading the control register\n");
		return ret;
	}

	charge_state = (status & 0x1F);
	supply_state = (status & 0xC0);

	printk("-----> ricoh618 charge_state is 0x%x supply_state is 0x%x !!!!!!\n", charge_state, supply_state);
	if (supply_state == SUPPLY_STATE_BAT)
	{
		temp = POWER_SUPPLY_STATUS_DISCHARGING;
	}else{
		switch (charge_state){
		case	CHG_STATE_CHG_OFF:
				temp = POWER_SUPPLY_STATUS_DISCHARGING;
				break;
		case	CHG_STATE_CHG_READY_VADP:
				temp = POWER_SUPPLY_STATUS_NOT_CHARGING;
				break;
		case	CHG_STATE_CHG_TRICKLE:
				temp = POWER_SUPPLY_STATUS_CHARGING;
				break;
		case	CHG_STATE_CHG_RAPID:
				temp = POWER_SUPPLY_STATUS_CHARGING;
				break;
		case	CHG_STATE_CHG_COMPLETE:
				temp = POWER_SUPPLY_STATUS_FULL;
				break;
		case	CHG_STATE_SUSPEND:
				temp = POWER_SUPPLY_STATUS_DISCHARGING;
				break;
		case	CHG_STATE_VCHG_OVER_VOL:
				temp = POWER_SUPPLY_STATUS_DISCHARGING;
				break;
		case	CHG_STATE_BAT_ERROR:
				temp = POWER_SUPPLY_STATUS_NOT_CHARGING;
				break;
		case	CHG_STATE_NO_BAT:
				temp = POWER_SUPPLY_STATUS_NOT_CHARGING;
				break;
		case	CHG_STATE_BAT_OVER_VOL:
				temp = POWER_SUPPLY_STATUS_NOT_CHARGING;
				break;
		case	CHG_STATE_BAT_TEMP_ERR:
				temp = POWER_SUPPLY_STATUS_NOT_CHARGING;
				break;
		case	CHG_STATE_DIE_ERR:
				temp = POWER_SUPPLY_STATUS_NOT_CHARGING;
				break;
		case	CHG_STATE_DIE_SHUTDOWN:
				temp = POWER_SUPPLY_STATUS_DISCHARGING;
				break;
		case	CHG_STATE_NO_BAT2:
				temp = POWER_SUPPLY_STATUS_NOT_CHARGING;
				break;
		case	CHG_STATE_CHG_READY_VUSB:
				temp = POWER_SUPPLY_STATUS_NOT_CHARGING;
				break;
		default:
				temp = POWER_SUPPLY_STATUS_UNKNOWN;
				break;
		}
	}

	return temp;
}

static void charger_irq_work(struct work_struct *work)
{
	struct ricoh618_battery_info *info
		 = container_of(work, struct ricoh618_battery_info, irq_work);
	int ret = 0;
	printk(KERN_INFO "PMU:%s In\n", __func__);

	power_supply_changed(&info->battery);

	mutex_lock(&info->lock);
	info->chg_ctr = 0;
	info->chg_stat1 = 0;

	/* Enable Interrupt for VADP */
	ret = ricoh618_clr_bits(info->dev->parent,
					 RICOH618_INT_MSK_CHGCTR, 0x01);
	if (ret < 0)
		dev_err(info->dev,
			 "%s(): Error in enable charger mask INT %d\n",
			 __func__, ret);

	/* Enable Interrupt for Charging & complete */
	ret = ricoh618_write(info->dev->parent, RICOH618_INT_MSK_CHGSTS1, 0xfc);
	if (ret < 0)
		dev_err(info->dev,
			 "%s(): Error in enable charger mask INT %d\n",
			 __func__, ret);

	mutex_unlock(&info->lock);
	printk(KERN_INFO "PMU:%s Out\n", __func__);
}

#ifdef ENABLE_LOW_BATTERY_DETECTION
static void low_battery_irq_work(struct work_struct *work)
{
	struct ricoh618_battery_info *info = container_of(work,
		 struct ricoh618_battery_info, low_battery_work.work);

	int ret = 0;

	printk(KERN_INFO "PMU:%s In\n", __func__);

	power_supply_changed(&info->battery);

	/* Enable VADP threshold Low interrupt */
	ricoh618_write(info->dev->parent, RICOH618_INT_EN_ADC1, 0x10);
	if (ret < 0)
		dev_err(info->dev,
			 "%s(): Error in enable adc mask INT %d\n",
			 __func__, ret);
}
#endif

static irqreturn_t charger_in_isr(int irq, void *battery_info)
{
	struct ricoh618_battery_info *info = battery_info;
	printk(KERN_INFO "PMU:%s\n", __func__);

	info->chg_stat1 |= 0x01;
	queue_work(info->workqueue, &info->irq_work);
	return IRQ_HANDLED;
}

static irqreturn_t charger_complete_isr(int irq, void *battery_info)
{
	struct ricoh618_battery_info *info = battery_info;
	printk(KERN_INFO "PMU:%s\n", __func__);

	info->chg_stat1 |= 0x02;
	queue_work(info->workqueue, &info->irq_work);

	return IRQ_HANDLED;
}

/*static irqreturn_t charger_usb_isr(int irq, void *battery_info)
{
	struct ricoh618_battery_info *info = battery_info;
	printk(KERN_INFO "PMU:%s\n", __func__);

	info->chg_ctr |= 0x02;
	queue_work(info->workqueue, &info->usb_irq_work);

	if (RICOH618_SOCA_UNSTABLE == info->soca->status
		|| RICOH618_SOCA_FG_RESET == info->soca->status)
		info->soca->stable_count = 11;

	return IRQ_HANDLED;
}

static irqreturn_t charger_adp_isr(int irq, void *battery_info)
{
	struct ricoh618_battery_info *info = battery_info;
	printk(KERN_INFO "PMU:%s\n", __func__);

	info->chg_ctr |= 0x01;
	queue_work(info->workqueue, &info->irq_work);

	if (RICOH618_SOCA_UNSTABLE == info->soca->status
		|| RICOH618_SOCA_FG_RESET == info->soca->status)
		info->soca->stable_count = 11;

	return IRQ_HANDLED;
}*/


#ifdef ENABLE_LOW_BATTERY_DETECTION
/*************************************************************/
/* for Detecting Low Battery                                 */
/*************************************************************/

static irqreturn_t adc_vsysl_isr(int irq, void *battery_info)
{

	struct ricoh618_battery_info *info = battery_info;

	printk(KERN_INFO "PMU:%s\n", __func__);

	queue_delayed_work(info->monitor_wqueue, &info->low_battery_work,
					LOW_BATTERY_DETECTION_TIME*HZ);

	return IRQ_HANDLED;
}
#endif



#ifdef	FUEL_GAGE_FUNCTION_ENABLE
static int get_check_fuel_gauge_reg(struct ricoh618_battery_info * info, int Reg_h, int Reg_l, int enable_bit)
{
	uint8_t get_data_h, get_data_l;
	int old_data, current_data;
	int i;
	int ret = 0;

	old_data = 0;

	for( i = 0; i < 5 ; i++)
	{
		ret = ricoh618_read(info->dev->parent, Reg_h, &get_data_h);
		if (ret < 0){
			dev_err(info->dev, "Error in reading the control register\n");
			return ret;
		}

		ret = ricoh618_read(info->dev->parent, Reg_l, &get_data_l);
		if (ret < 0){
			dev_err(info->dev, "Error in reading the control register\n");
			return ret;
		}

		current_data =((get_data_h & 0xff) << 8) | (get_data_l & 0xff);
		current_data = (current_data & enable_bit);

		if(current_data == old_data)
		{
			return current_data;
		}else{

			old_data = current_data;
		}
	}

	return current_data;
}

static int calc_capacity(struct ricoh618_battery_info *info)
{
	uint8_t capacity;
	int temp;
	int ret = 0;

	//get remaining battery capacity from fuel gauge
	ret = ricoh618_read(info->dev->parent, SOC_REG, &capacity);
	if(ret<0){
		dev_err(info->dev, "Error in reading the control register\n");
		return ret;
	}

	temp = capacity;

	return temp;		//Unit is 1%
}

static int get_buttery_temp(struct ricoh618_battery_info *info)
{
	uint8_t sign_bit;
	int ret = 0;

	ret = get_check_fuel_gauge_reg(info, TEMP_1_REG, TEMP_2_REG, 0x0fff);
	if(ret<0){
		dev_err(info->dev, "Error in reading the fuel gauge control register\n");
		return ret;
	}

	ret = ( ret & 0x07ff);

	ret = ricoh618_read(info->dev->parent, TEMP_1_REG, &sign_bit);
	if(ret<0){
		dev_err(info->dev, "Error in reading the control register\n");
		return ret;
	}

	sign_bit = ( (sign_bit & 0x08) >> 3 ); 							//bit3 of 0xED(TEMP_1) is sign_bit

	if(sign_bit ==0){ //positive value part
		ret = ret * 625  / 1000;						//conversion unit // 1 unit is 0.0625 degree and retun unit should be 0.1 degree,
	}else//negative value part
	{
		ret = -1 * ret * 625 /1000;
	}

	return ret;
}

static int get_time_to_empty(struct ricoh618_battery_info *info)
{
	int ret = 0;

	ret = get_check_fuel_gauge_reg(info, TT_EMPTY_H_REG, TT_EMPTY_L_REG, 0xffff);
	if(ret<0){
		dev_err(info->dev, "Error in reading the fuel gauge control register\n");
		return ret;
	}

	ret = ret * 60;									//conversion unit// 1unit is 1miniute and return nnit should be 1 second

	return ret;
}

static int get_time_to_full(struct ricoh618_battery_info *info)
{
	int ret = 0;

	ret = get_check_fuel_gauge_reg(info, TT_FULL_H_REG, TT_FULL_L_REG, 0xffff);
	if(ret<0){
		dev_err(info->dev, "Error in reading the fuel gauge control register\n");
		return ret;
	}

	ret = ret * 60;

	return  ret;
}

//battery voltage is get from Fuel gauge
static int measure_vbatt_FG(struct ricoh618_battery_info *info, int *data)
{
	int ret = 0;

//	mutex_lock(&info->lock);

	ret =  get_check_fuel_gauge_reg(info, VOLTAGE_1_REG, VOLTAGE_2_REG, 0x0fff);
	if(ret<0){
		dev_err(info->dev, "Error in reading the fuel gauge control register\n");
		mutex_unlock(&info->lock);
		return ret;
	}

	*data = ret;
	*data = *data * 5000 / 4095;		//conversion unit 1 Unit is 1.22mv (5000/4095 mv)
	*data = *data * 1000;			//return unit should be 1uV

//	mutex_unlock(&info->lock);

	return ret;
}
static int measure_Ibatt_FG(struct ricoh618_battery_info *info, int *data)
{
	int ret = 0;

	ret =  get_check_fuel_gauge_reg(info, CC_AVERAGE1_REG,
						 CC_AVERAGE0_REG, 0x3fff);
	if (ret < 0) {
		dev_err(info->dev, "Error in reading the fuel gauge control register\n");
		return ret;
	}

	*data = (ret > 0x1fff) ? (ret - 0x4000) : ret;
	return ret;
}

static int get_OCV_init_Data(struct ricoh618_battery_info *info, int index)
{
	int ret = 0;
	ret =  (battery_init_para[index*2]<<8) | (battery_init_para[index*2+1]);
	return ret;
}

static int get_OCV_voltage(struct ricoh618_battery_info *info, int index)
{
	int ret = 0;
	ret =  get_OCV_init_Data(info, index);
	/* conversion unit 1 Unit is 1.22mv (5000/4095 mv) */
	ret = ret * 50000 / 4095;
	/* return unit should be 1uV */
	ret = ret * 100;
	return ret;
}


#else
//battery voltage is get from ADC
static int measure_vbatt_ADC(struct ricoh618_battery_info *info, int *data)
{
	int	i;
	uint8_t data_l =0, data_h = 0;
	int ret;

//	mutex_lock(&info->lock);

	ret = ricoh618_set_bits(info->dev->parent, INTEN_REG, 0x08); 		//ADC interrupt enable
	if(ret <0){
		dev_err(info->dev, "Error in setting the control register bit\n");
		goto err;
	}

	ret = ricoh618_set_bits(info->dev->parent, EN_ADCIR3_REG, 0x01); 	//enable interrupt request of single mode
	if(ret <0){
		dev_err(info->dev, "Error in setting the control register bit\n");
		goto err;
	}

	ret = ricoh618_write(info->dev->parent, ADCCNT3_REG, 0x10);		//single request
	if(ret <0){
		dev_err(info->dev, "Error in writing the control register\n");
		goto err;
	}

	for(i = 0; i < 5; i++){
		msleep(1);
		dev_info(info->dev, "ADC conversion times: %d\n", i);
		ret = ricoh618_read(info->dev->parent, EN_ADCIR3_REG, &data_h);	//read completed flag of  ADC
		if(ret <0){
			dev_err(info->dev, "Error in reading the control register\n");
			goto err;
		}

		if(data_h && 0x01){
			goto	done;
		}
	}

	dev_err(info->dev, "ADC conversion too long!\n");
	goto err;

done:
	ret = ricoh618_read(info->dev->parent, VBATDATAH_REG, &data_h);
	if(ret <0){
		dev_err(info->dev, "Error in reading the control register\n");
		goto err;
	}

	ret = ricoh618_read(info->dev->parent, VBATDATAL_REG, &data_l);
	if(ret <0){
		dev_err(info->dev, "Error in reading the control register\n");
		goto err;
	}

	*data = ((data_h & 0xff) << 4) | (data_l & 0x0f);
	*data = *data * 5000 / 4095;								//conversion unit 1 Unit is 1.22mv (5000/4095 mv)
	*data = *data * 1000;									//return unit should be 1uV

//	mutex_unlock(&info->lock);

	return 0;

err:
//	mutex_unlock(&info->lock);
	return -1;
}

#endif

static void ricoh618_external_power_changed(struct power_supply *psy)
{
	struct ricoh618_battery_info *info;

	info = container_of(psy, struct ricoh618_battery_info, battery);
	queue_delayed_work(info->monitor_wqueue,
			   &info->changed_work, HZ / 2);
//	printk("---> %s\n", __func__);
	return;
}


static int ricoh618_batt_get_prop(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	struct ricoh618_battery_info *info = dev_get_drvdata(psy->dev->parent);
	int data = 0;
	int ret = 0;
	uint8_t status;

	mutex_lock(&info->lock);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		ret = ricoh618_read(info->dev->parent, CHGSTATE_REG, &status);
		if (ret < 0) {
			dev_err(info->dev, "Error in reading the control register\n");
			mutex_unlock(&info->lock);
			return ret;
		}
		if (psy->type == POWER_SUPPLY_TYPE_MAINS)
			val->intval = (status & 0x40 ? 1 : 0);
		else if (psy->type == POWER_SUPPLY_TYPE_USB)
			val->intval = (status & 0x80 ? 1 : 0);
		break;

	 //this setting is same as battery driver of 584
	case POWER_SUPPLY_PROP_STATUS:
		ret = get_power_supply_status(info);
		val->intval = ret;
		info->status = ret;
		dev_info(info->dev, "Power Supply Status is %d\n", info->status);
		break;

	 //this setting is same as battery driver of 584
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = info->present;
		break;

	//current voltage is get from fuel gauge
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		/* return real vbatt Voltage */
#ifdef	FUEL_GAGE_FUNCTION_ENABLE
		ret = measure_vbatt_FG(info, &data);
#else
		ret = measure_vbatt_ADC(info, &data);
#endif
		val->intval = data;
		info->cur_voltage = data / 1000;		//convert unit uV -> mV
		dev_info(info->dev, "battery voltage is %d mV\n", info->cur_voltage);

		break;

#ifdef	FUEL_GAGE_FUNCTION_ENABLE
	//current battery capacity is get from fuel gauge
	case POWER_SUPPLY_PROP_CAPACITY:
		ret = calc_capacity(info);
		val->intval = ret;
		info->capacity = ret;
		dev_info(info->dev, "battery capacity is %d%%\n", info->capacity);
		break;

	//current temperature of buttery
	case POWER_SUPPLY_PROP_TEMP:
		ret = get_buttery_temp(info);
		val->intval = ret;
		info->battery_temp = ret/10;
		dev_info(info->dev, "battery temperature is %d degree \n",  info->battery_temp );
		break;

	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW:
		ret = get_time_to_empty(info);
		val->intval = ret;
		info->time_to_empty = ret/60;
		dev_info(info->dev, "time of empty buttery is %d minutes \n", info->time_to_empty );
		break;

	 case POWER_SUPPLY_PROP_TIME_TO_FULL_NOW:
		ret = get_time_to_full(info);
		val->intval = ret;
		info->time_to_full = ret/60;
		dev_info(info->dev, "time of full buttery is %d minutes \n", info->time_to_full );
		break;
#endif
	 case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		ret = 0;
		break;

	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = POWER_SUPPLY_HEALTH_GOOD;
		ret = 0;
		break;

	default:
		return -ENODEV;
	}

	mutex_unlock(&info->lock);

	return ret;
}

/*static int ricoh618_charger_get_prop(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	struct ricoh618_battery_info *info = dev_get_drvdata(psy->dev->parent);
	uint8_t status, supply_state;
	int ret = 0;

	ret = ricoh618_read(info->dev->parent, CHGSTATE_REG, &status);
	if (ret<0){
		dev_err(info->dev, "Error in reading the control register\n");
		return ret;
	}
	supply_state = (status & 0xC0) >> 5;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if (psy->type == POWER_SUPPLY_TYPE_MAINS) {
			val->intval = supply_state & (1 << SUPPLY_STATE_ADP);
		} else if (psy->type == POWER_SUPPLY_TYPE_USB) {
			val->intval = supply_state & (1 << SUPPLY_STATE_USB);
		} else
			val->intval = 0;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}*/

static irqreturn_t ricoh618_ac_irq(int irq, void *dev_id)
{
	struct ricoh618_battery_info *info = dev_id;

	power_supply_changed(&info->ac);

	printk("====> %s\n", __func__);
	return IRQ_HANDLED;
}

static irqreturn_t ricoh618_usb_irq(int irq, void *dev_id)
{
	struct ricoh618_battery_info *info = dev_id;

	dev_dbg(info->dev,"----> %s \n", __func__);

	power_supply_changed(&info->usb);

	printk("====> %s\n", __func__);
	return IRQ_HANDLED;
}

static enum power_supply_property ricoh618_batt_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,

#ifdef	FUEL_GAGE_FUNCTION_ENABLE
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW,
	POWER_SUPPLY_PROP_TIME_TO_FULL_NOW,
#endif
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_HEALTH,
};

static enum power_supply_property ricoh618_charger_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static char *supply_list[] = {
	"battery",
};

static __devinit int ricoh618_battery_probe(struct platform_device *pdev)
{
	struct ricoh618 *iodev = dev_get_drvdata(pdev->dev.parent);
	struct pmu_platform_data *pmu_pdata = dev_get_platdata(iodev->dev);
	struct ricoh618_battery_info *info;
	struct ricoh618_battery_platform_data *pdata;

	int ret = 0;

	printk("-------->  %s: \n",__func__);

	info = kzalloc(sizeof(struct ricoh618_battery_info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;
	info->soca = kzalloc(sizeof(struct ricoh618_soca_info), GFP_KERNEL);
		if (!info->soca)
			return -ENOMEM;


	info->dev = &pdev->dev;
	info->status = POWER_SUPPLY_STATUS_CHARGING;
	pdata = (struct ricoh618_battery_platform_data *)pmu_pdata->bat_private;
	info->monitor_time = pdata->monitor_time * HZ;
	info->alarm_vol_mv = pdata->alarm_vol_mv;
	info->adc_vdd_mv = ADC_VDD_MV;	//2800;
	info->min_voltage = MIN_VOLTAGE;	//3100;
	info->max_voltage = MAX_VOLTAGE;	//4200;
	info->delay = 500;
	info->entry_factory_mode = false;


	mutex_init(&info->lock);
	platform_set_drvdata(pdev, info);
	if (!((&pdev->dev)->p)) {
		printk("-----> platform_set_drvdata error  \n");
		goto out;
	}

	info->battery.name = "battery";
	info->battery.type = POWER_SUPPLY_TYPE_BATTERY;
	info->battery.properties = ricoh618_batt_props;
	info->battery.num_properties = ARRAY_SIZE(ricoh618_batt_props);
	info->battery.get_property = ricoh618_batt_get_prop;
	info->battery.set_property = NULL;
	info->battery.external_power_changed = ricoh618_external_power_changed;

	/* Disable Charger/ADC interrupt */
	ret = ricoh618_clr_bits(info->dev->parent, RICOH618_INTC_INTEN,
							 CHG_INT | ADC_INT);
	if (ret)
		goto out;

	ret = ricoh618_init_battery(info);
	if (ret){
		goto out;
	}

#ifdef ENABLE_FACTORY_MODE
	info->factory_mode_wqueue
		= create_singlethread_workqueue("ricoh618_factory_mode");
	INIT_DELAYED_WORK_DEFERRABLE(&info->factory_mode_work,
					 check_charging_state_work);

	ret = ricoh618_factory_mode(info);
	if (ret)
		goto out;

#endif


	ret = power_supply_register(&pdev->dev, &info->battery);
	if (ret){
		info->battery.dev->parent = &pdev->dev;
	}

#define DEF_CHARGER(PSY, NAME, TYPE)					\
	info->PSY.name = NAME;						\
	info->PSY.type = TYPE;						\
	info->PSY.supplied_to = supply_list;				\
	info->PSY.num_supplicants = ARRAY_SIZE(supply_list);		\
	info->PSY.properties = ricoh618_charger_props;			\
	info->PSY.num_properties = ARRAY_SIZE(ricoh618_charger_props);	\
	info->PSY.get_property = ricoh618_batt_get_prop
//	info->PSY.get_property = ricoh618_charger_get_prop

	DEF_CHARGER(usb, "usb", POWER_SUPPLY_TYPE_USB);
	DEF_CHARGER(ac, "ac", POWER_SUPPLY_TYPE_MAINS);
#undef DEF_POWER

	info->irq_usb = RICOH618_IRQ_FVUSBDETSINT + IRQ_RESERVED_BASE;
	info->irq_ac = RICOH618_IRQ_FVADPDETSINT + IRQ_RESERVED_BASE;

	info->workqueue = create_singlethread_workqueue("ricoh618_charger_in");
	INIT_WORK(&info->irq_work, charger_irq_work);

	ret = request_threaded_irq(RICOH618_IRQ_FONCHGINT + IRQ_RESERVED_BASE,
					NULL, charger_in_isr,IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
						"ricoh618_charger_in", info);
	if (ret < 0) {
		dev_err(&pdev->dev, "Can't get CHG_INT IRQ for chrager: %d\n",
									ret);
		goto out;
	}
	ret = request_threaded_irq(RICOH618_IRQ_FCHGCMPINT + IRQ_RESERVED_BASE,
						NULL, charger_complete_isr,
					IRQF_TRIGGER_FALLING | IRQF_ONESHOT, "ricoh618_charger_comp",
								info);
	if (ret < 0) {
		dev_err(&pdev->dev, "Can't get CHG_COMP IRQ for chrager: %d\n",
									 ret);
		goto out;
	}

	ret = request_threaded_irq(info->irq_ac, NULL, ricoh618_ac_irq,
			  IRQF_TRIGGER_FALLING | IRQF_DISABLED, "ricoh618_ac_irq", info);
	if (ret) {
		dev_err(info->dev, "request ricoh618_ac_irq fail with error num = %d \n", ret);
		goto out;
	}
	ret = request_threaded_irq(info->irq_usb, NULL, ricoh618_usb_irq,
			  IRQF_TRIGGER_FALLING | IRQF_DISABLED, "ricoh618_usb_irq", info);
	if (ret) {
		dev_err(info->dev, "request ricoh618_usb_irq fail with error num = %d \n", ret);
		goto out;
	}


	info->monitor_wqueue = create_singlethread_workqueue("ricoh618_battery_monitor");
	INIT_DELAYED_WORK_DEFERRABLE(&info->monitor_work, ricoh618_battery_work);
	INIT_DELAYED_WORK(&info->changed_work, ricoh618_changed_work);

	INIT_DELAYED_WORK_DEFERRABLE(&info->displayed_work,
					 ricoh618_displayed_work);
	INIT_DELAYED_WORK_DEFERRABLE(&info->charge_stable_work,
					 ricoh618_stable_charge_countdown_work);


	power_supply_register(&pdev->dev, &info->usb);
	power_supply_register(&pdev->dev, &info->ac);

	queue_delayed_work(info->monitor_wqueue, &info->monitor_work,
			   info->monitor_time);

#ifdef ENABLE_LOW_BATTERY_DETECTION
	ret = request_threaded_irq(RICOH618_IRQ_VSYSLIR + IRQ_RESERVED_BASE,
					NULL, adc_vsysl_isr, IRQF_ONESHOT,
						"ricoh618_adc_vsysl", info);
	if (ret < 0) {
		dev_err(&pdev->dev,
			"Can't get ADC_VSYSL IRQ for chrager: %d\n", ret);
		goto out;
	}
	INIT_DELAYED_WORK_DEFERRABLE(&info->low_battery_work,
					 low_battery_irq_work);
#endif

	/* Charger init and IRQ setting */
	ret = ricoh618_init_charger(info);
	if (ret)
		goto out;

#ifdef	FUEL_GAGE_FUNCTION_ENABLE
	ret = ricoh618_init_fgsoca(info);
#endif

	/* Enable Charger interrupt */
	ricoh618_set_bits(info->dev->parent, RICOH618_INTC_INTEN, CHG_INT);

	return 0;

out:
	kfree(info);
	return ret;
}

static int __devexit ricoh618_battery_remove(struct platform_device *pdev)
{
	struct ricoh618_battery_info *info = platform_get_drvdata(pdev);

	cancel_delayed_work(&info->monitor_work);
	cancel_delayed_work(&info->changed_work);
	flush_workqueue(info->monitor_wqueue);
	power_supply_unregister(&info->battery);
	kfree(info);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

#ifdef CONFIG_PM
static int ricoh618_battery_suspend(struct device *dev)
{
	struct ricoh618_battery_info *info = dev_get_drvdata(dev);

	cancel_delayed_work_sync(&info->monitor_work);

	return 0;
}

static int ricoh618_battery_resume(struct device *dev)
{
	struct ricoh618_battery_info *info = dev_get_drvdata(dev);

	queue_delayed_work(info->monitor_wqueue, &info->monitor_work,
			   info->monitor_time);
	return 0;
}

static struct dev_pm_ops ricoh618_battery_pm_ops = {
	.suspend	= ricoh618_battery_suspend,
	.resume		= ricoh618_battery_resume,
};
#endif

static struct platform_driver ricoh618_battery_driver = {
	.driver	= {
				.name	= "ricoh618-battery",
				.owner	= THIS_MODULE,
#ifdef CONFIG_PM
				.pm	= &ricoh618_battery_pm_ops,
#endif
	},
	.probe	= ricoh618_battery_probe,
	.remove	= __devexit_p(ricoh618_battery_remove),
};

static int __init ricoh618_battery_init(void)
{
	return platform_driver_register(&ricoh618_battery_driver);
}
module_init(ricoh618_battery_init);

static void __exit ricoh618_battery_exit(void)
{
	platform_driver_unregister(&ricoh618_battery_driver);
}
module_exit(ricoh618_battery_exit);

MODULE_DESCRIPTION("RICOH618 Battery driver");
MODULE_ALIAS("platform:ricoh618-battery");
MODULE_LICENSE("GPL");
