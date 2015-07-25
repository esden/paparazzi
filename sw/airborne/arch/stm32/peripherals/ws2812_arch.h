/*
 * Copyright (C) 2015 Piotr Esden-Tempski <piotr@esden.net>
 *
 * This file is part of paparazzi.
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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

 /**
  * @file arch/stm32/peripherals/ws2812_arch.h
  *
  * Driver for the WS2812 RGB led strings with built in driver.
  */

#ifndef WS2812_ARCH_H
#define WS2812_ARCH_H

void ws2812_arch_init(struct ws2812_status *ws2812_status);
void ws2812_arch_send(void);
bool ws2812_arch_running(void);

#endif /* WS2812_ARCH_H */