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
  * @file peripherals/ws2812.c
  *
  * Driver for the WS2812 RGB led strings with built in driver.
  */

#include <stdint.h>
#include <stdbool.h>

#include "peripherals/ws2812.h"

/* Private declarations. */
struct ws2812_status ws2812_status;

/* Private function declarations. */

/* API functions */
void ws2812_init(void)
{
	ws2812_status.stage = ws2812_idle;

	ws2812_arch_init(&ws2812_status);
}

void ws2812_send(ws2812_led_t *leds, int led_count)
{
	ws2812_status.leds = leds;
	ws2812_status.led_count = led_count;
	ws2812_status.leds_sent = 0;

	ws2812_init_bit_buffer();
	ws2812_arch_send();
}

bool ws2812_is_sending(void)
{
	return (ws2812_status.stage != ws2812_idle);
}

/* Private functions */

void ws2812_init_bit_buffer(void)
{
	ws2812_fill_low_bit_buffer();
	ws2812_fill_high_bit_buffer();
}

void ws2812_fill_low_bit_buffer(void)
{
	ws2812_fill_bit_buffer(false);
}

void ws2812_fill_high_bit_buffer(void)
{
	ws2812_fill_bit_buffer(true);
}

void ws2812_fill_bit_buffer(bool low_high)
{
	int offset = 0;
	int bitcount = WS2812_BIT_BUFFER_SIZE / 2;
	int led = ws2812_status.leds_sent;
	int i;

	ws2812_status.stage = ws2812_sending;

	if(low_high) {
		offset = bitcount;
	}

	/*
	 * 60 = 1
	 * 29 = 0
	 */
	for(i = 0; i < bitcount; i++) {
		if (i < ((ws2812_status.led_count - ws2812_status.leds_sent) * 24)) {
			if (((ws2812_status.leds[ws2812_status.leds_sent + (i/24)].grbu >> (31 - (i % 24)))
				 & 0x00000001) != 0) {
				ws2812_status.bit_buffer[offset + i] = 60;
			} else {
				ws2812_status.bit_buffer[offset + i] = 29;
			}
			led = ws2812_status.leds_sent + ((i + 0) / 24);
		} else {
			ws2812_status.stage = ws2812_done;
			break;
		}
	}

	for(; i < bitcount; i++) {
			ws2812_status.bit_buffer[offset + i] = 0;
	}

	ws2812_status.leds_sent = led + 1;
}

void ws2812_clear_bit_buffer(void) {
	for(int i = 0; i < WS2812_BIT_BUFFER_SIZE; i++) {
		ws2812_status.bit_buffer[i] = 0;
	}
}