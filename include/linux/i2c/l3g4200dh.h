/*
 * L3G4200Dh gyroscope driver
 *
 * Copyright (C) 2011 TI India
 * Author: Pankaj Bharadiya <pankaj.bharadiya@ti.com>
 *
 *  Based on the code by kalle.viironen@digia.com
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __L3G4200DH__
#define __L3G4200DH__

/* board config */
struct l3g4200dh_platform_data {
	int (*setup_resources)(void *);
	void (*release_resources)(void *);
	struct device *dev;
	int use_regulators;
};

#define L3G4200DH_IRQ_GPIO      55

#endif
