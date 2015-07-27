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
  * @file peripherals/ws2812.h
  *
  * Driver for the WS2812 RGB led strings with built in driver.
  */

#ifndef WS2812_H
#define WS2812_H

typedef union {
	struct __attribute__ ((__packed__)) {
		uint8_t _unused;
		uint8_t b;
		uint8_t r;
		uint8_t g;
	} colors;
	uint32_t grbu;
} ws2812_led_t;

enum ws2812_stage {
	ws2812_idle,
	ws2812_sending,
	ws2812_done,
	ws2812_reset
};

/* XXX: This should probably not be here. */
#define WS2812_BIT_BUFFER_SIZE (24*6)

struct ws2812_status {
	ws2812_led_t *leds;
	int led_count;
	int leds_sent;
	volatile enum ws2812_stage stage;
	uint16_t bit_buffer[WS2812_BIT_BUFFER_SIZE];
};

void ws2812_init(void);
void ws2812_send(ws2812_led_t *leds, int led_count);
bool ws2812_is_sending(void);

void ws2812_init_bit_buffer(void);
void ws2812_fill_low_bit_buffer(void);
void ws2812_fill_high_bit_buffer(void);
void ws2812_fill_bit_buffer(bool low_high);
void ws2812_clear_bit_buffer(void);

/* Hardware dependent implementation declarations. */
#include "peripherals/ws2812_arch.h"

#endif /* WS2812_H */