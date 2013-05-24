/*
 * include/linux/power/ricoh61x_battery_init.h
 *
 * Battery initial parameter for RICOH RN5T618/619 power management chip.
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
#ifndef __LINUX_POWER_RICOH61X_BATTERY_INIT_H
#define __LINUX_POWER_RICOH61X_BATTERY_INIT_H


uint8_t battery_init_para[32] = {
		0x08, 0xa3, 0x0a, 0xf9, 0x0b, 0x4b, 0x0b, 0x74, 0x0b, 0x94, 0x0b, 0xb5, 0x0b, 0xe6, 0x0c, 0x30,
		0x0c, 0x92, 0x0c, 0xf4, 0x0d, 0x6f, 0x08, 0xca, 0x00, 0x36, 0x0f, 0xc8, 0x05, 0x2c, 0x22, 0x56
};

#endif
