/*
 * [board]-pmu.c - This file defines PMU board information.
 *
 * Copyright (C) 2012 Ingenic Semiconductor Co., Ltd.
 * Author: Large Dipper <ykli@ingenic.com>.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/platform_device.h>
#include <linux/regulator/machine.h>
#include <linux/mfd/pmu-common.h>
#include <linux/i2c.h>
#include <gpio.h>

/**
 * Core voltage Regulator.
 * Couldn't be modified.
 * Voltage was inited at bootloader.
 */
CORE_REGULATOR_DEF(
	zpad80,	1000000,	1400000);

/**
 * I/O Regulator.
 * It's the parent regulator of most of devices regulator on board.
 * Voltage was inited at bootloader.
 */
IO_REGULATOR_DEF(
	zpad80_vccio,
	"Vcc-IO",	3300000,	1);

/**
 * USB VBUS Regulators.
 * Switch of USB VBUS. It may be a actual or virtual regulator.
 */
VBUS_REGULATOR_DEF(
	zpad80,		"OTG-Vbus");

/**
 * Exclusive Regulators.
 * They are only used by one device each other.
 */
EXCLUSIVE_REGULATOR_DEF(
	zpad80_vwifi,
	"Wi-Fi",
	"vwifi",	NULL,	NULL,	3300000,  0);

EXCLUSIVE_REGULATOR_DEF(
	zpad80_vtsc,
	"Touch Screen",
	"vtsc",		NULL,  NULL,	3300000,  0);

EXCLUSIVE_REGULATOR_DEF(
	zpad80_vlcd,
	"V-vlcd",
	"vlcd",	NULL,  NULL,	3300000,  0);

/**
 * Fixed voltage Regulators.
 * GPIO silulator regulators. Everyone is an independent device.
 */

/* FIXME! when board fixed, remove it */
FIXED_REGULATOR_DEF(
	zpad80_vbus,
	"OTG-Vbus",	5000000,       -1,
	HIGH_ENABLE,	UN_AT_BOOT,	0,
	NULL,	"vdrvvbus",	NULL);

FIXED_REGULATOR_DEF(
	zpad80_vcim,
	"Camera",	2800000,	GPIO_PB(27),
	HIGH_ENABLE,	UN_AT_BOOT,	0,
	NULL,		"vcim",		"jz-cim");

FIXED_REGULATOR_DEF(
	zpad80_vbklight,
	"vbklight",		3300000,	GPIO_PF(10),
	HIGH_ENABLE,	EN_AT_BOOT,	0,
	NULL,		"vbklight",		NULL);

FIXED_REGULATOR_DEF(
	zpad80_vlcd_en,
	"vlcd_en",		3300000,	GPIO_PB(23),
	HIGH_ENABLE,	EN_AT_BOOT,	0,
	NULL,		"vlcd_en",		NULL);

FIXED_REGULATOR_DEF(
	zpad80_vlcd_vcom,
	"vlcd_vcom",        3300000,    GPIO_PF(11),
	HIGH_ENABLE,	EN_AT_BOOT,	0,
	NULL,       "vlcd_vcom",        NULL);

FIXED_REGULATOR_DEF(
	zpad80_vpower_en,
	"vpower_en",		3300000,	GPIO_PF(9),
	HIGH_ENABLE,	EN_AT_BOOT,	0,
	NULL,		"vpower_en",		NULL);


static struct platform_device *fixed_regulator_devices[] __initdata = {
	&zpad80_vbus_regulator_device,
	&zpad80_vcim_regulator_device,
	&zpad80_vbklight_regulator_device,
	&zpad80_vlcd_en_regulator_device,
	&zpad80_vlcd_vcom_regulator_device,
	&zpad80_vpower_en_regulator_device,
};

/*
 * Regulators definitions used in PMU.
 *
 * If +5V is supplied by PMU, please define "VBUS" here with init_data NULL,
 * otherwise it should be supplied by a exclusive DC-DC, and you should define
 * it as a fixed regulator.
 */
static struct regulator_info zpad80_pmu_regulators[] = {
	{"OUT1", &zpad80_vcore_init_data},
	{"OUT3", &zpad80_vccio_init_data},
	{"OUT6", &zpad80_vwifi_init_data},
	{"OUT7", &zpad80_vtsc_init_data},
	{"OUT8", &zpad80_vlcd_init_data},
	{"VBUS", &zpad80_vbus_init_data},
};

static struct charger_board_info charger_board_info = {
	.gpio	= GPIO_PB(2),
	.enable_level	= LOW_ENABLE,
};

static struct pmu_platform_data zpad80_pmu_pdata = {
	.gpio = GPIO_PA(28),
	.num_regulators = ARRAY_SIZE(zpad80_pmu_regulators),
	.regulators = zpad80_pmu_regulators,
	.charger_board_info = &charger_board_info,
};

#define PMU_I2C_BUSNUM 0

struct i2c_board_info zpad80_pmu_board_info = {
	I2C_BOARD_INFO("act8600", 0x5a),
	.platform_data = &zpad80_pmu_pdata,
};

static int __init zpad80_pmu_dev_init(void)
{
	struct i2c_adapter *adap;
	struct i2c_client *client;
	int busnum = PMU_I2C_BUSNUM;
	int i;

	adap = i2c_get_adapter(busnum);
	if (!adap) {
		pr_err("failed to get adapter i2c%d\n", busnum);
		return -1;
	}

	client = i2c_new_device(adap, &zpad80_pmu_board_info);
	if (!client) {
		pr_err("failed to register pmu to i2c%d\n", busnum);
		return -1;
	}

	i2c_put_adapter(adap);

	for (i = 0; i < ARRAY_SIZE(fixed_regulator_devices); i++)
		fixed_regulator_devices[i]->id = i;

	return platform_add_devices(fixed_regulator_devices,
				    ARRAY_SIZE(fixed_regulator_devices));
}

subsys_initcall_sync(zpad80_pmu_dev_init);
