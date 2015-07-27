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
 * @file "modules/light/light_neo.c"
 * @author Piotr Esden-Tempski
 * NeoPixel (WS2812) based aircraft lighting.
 */

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "modules/light/light_neo.h"
#include "peripherals/ws2812.h"
#include "state.h"

#ifndef LED_COUNT
#define LED_COUNT (12)
#endif

/*
 * 0: disabled
 * 1: strobe
 */
#ifndef LIGHT_NEO_STROBE
#define LIGHT_NEO_STROBE 0
#endif

/*
 * 0: disabled
 * 1: red dot north
 */
#ifndef LIGHT_NEO_HEADING
#define LIGHT_NEO_HEADING 1
#endif

#ifndef LIGHT_NEO_HEADING_INVERT
#define LIGHT_NEO_HEADING_INVERT -1
#endif

/*
 * 0: disabled
 * 1: blue -> x, green -> y
 */
#ifndef LIGHT_NEO_GRADIENT
#define LIGHT_NEO_GRADIENT 1
#endif

struct light_neo_status {
  ws2812_led_t leds[LED_COUNT];

  uint32_t timer;
} light_neo_status;

/* Private function declarations. */
void light_neo_set_dot(void);
void light_neo_set_gradient(void);

/* Implementation. */
void light_neo_init(void)
{
  int i;

  for (i = 0; i < LED_COUNT; i++) {
    light_neo_status.leds[i].grbu = 0;
  }

  light_neo_status.timer = 0;

  ws2812_init();
}

void light_neo_periodic(void)
{
	int i;

	light_neo_status.timer++;

	/* Reset LEDS. */
	for (i = 0; i < LED_COUNT; i++) {
		light_neo_status.leds[i].grbu = 0x00000000;
	}

	switch (LIGHT_NEO_GRADIENT) {
	case 0:
		break;
	case 1:
		light_neo_set_gradient();
		break;
	}

	switch (LIGHT_NEO_HEADING) {
	case 0:
		break;
	case 1: /* Red Dot */
		light_neo_set_dot();
		break;
	default:
		break;
	}

	switch (LIGHT_NEO_STROBE) {
	case 0:
		break;
	case 1:
		if ((light_neo_status.timer % 55) >= 50) {
			for (i = 0; i < LED_COUNT; i++) {
				light_neo_status.leds[i].grbu = 0xFFFFFFFF;
			}
		}
		break;
	default:
		break;
	}

	if (!ws2812_is_sending()) {
		ws2812_send(light_neo_status.leds, LED_COUNT);
	}
}

void light_neo_set_dot(void)
{
	int i;
	float value;

	/* Get heading in radians. */
	float heading = stateGetNedToBodyEulers_f()->psi * LIGHT_NEO_HEADING_INVERT;

	/* Set the red component of the using a cos gradient. */
	for (i = 0; i < LED_COUNT; i++) {
		value = (cosf(heading + (i * ((M_PI * 2) / LED_COUNT))) * 1024) - 767;
		if (value > 0) {
			light_neo_status.leds[i].colors.r += value;
		}
	}

}

void light_neo_set_gradient(void)
{
	int i;
	struct EnuCoor_f *pos = stateGetPositionEnu_f();

	if ((pos->x) < -5.0 || (pos->x > 5.0)) {
		if ((light_neo_status.timer % 55) >= 45) {
			for (i = 0; i < LED_COUNT; i++) {
				light_neo_status.leds[i].colors.g = 0xFF;
			}
		}
	} else {
		for (i = 0; i < LED_COUNT; i++) {
			light_neo_status.leds[i].colors.g = (pos->x + 5.0) * 25.5;
		}
	}

	if ((pos->y) < -5.0 || (pos->y > 5.0)) {
		if ((light_neo_status.timer % 55) >= 45) {
			for (i = 0; i < LED_COUNT; i++) {
				light_neo_status.leds[i].colors.b = 0xFF;
			}
		}
	} else {
		for (i = 0; i < LED_COUNT; i++) {
			light_neo_status.leds[i].colors.b = (pos->y + 5.0) * 25.5;
		}
	}
}