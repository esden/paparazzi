/*
 * Copyright (C) Piotr Esden-Tempski
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/light/light_neo.h"
 * @author Piotr Esden-Tempski
 * NeoPixel (WS2812) based aircraft lighting.
 */

#ifndef LIGHT_NEO_H
#define LIGHT_NEO_H

void light_neo_init(void);
void light_neo_periodic(void);

#endif

