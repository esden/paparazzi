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


#include BOARD_CONFIG
#include "mcu.h"
#include "mcu_periph/sys_time.h"
#include "subsystems/datalink/downlink.h"
#include "peripherals/ws2812.h"
#include "led.h"

#define LED_COUNT (12)

struct ws2812_led_status {
  ws2812_led_t leds[LED_COUNT];

  uint32_t timer;
} ws2812_led_status;

static inline void main_init(void);
static inline void main_periodic_task(void);
static inline void main_event_task(void);
static inline void ws2812_led_init(void);
static inline void ws2812_led_inc(void);
static inline void ws2812_led_set_wheel(ws2812_led_t *led, int angle);

/* The angle from 0deg to 360deg represented by a value from 0 to 1536. */
void ws2812_led_set_wheel(ws2812_led_t *led, int angle)
{
  int segment_angle;

  if (angle < 0) {
    led->colors.r = 0;
    led->colors.g = 0;
    led->colors.b = 0;
    return;
  }

  if (angle > 1535) {
    led->colors.r = 255;
    led->colors.g = 255;
    led->colors.b = 255;
    return;
  }

  if(angle > (255 * 0) && angle <= (255 * 1)) {
    segment_angle = angle - (255 * 0);
    led->colors.r = 255;
    led->colors.g = segment_angle;
    led->colors.b = 0;
  } else if(angle > (255 * 1) && angle <= (255 * 2)) {
    segment_angle = angle - (255 * 1);
    led->colors.r = 255 - segment_angle;
    led->colors.g = 255;
    led->colors.b = 0;
  } else if(angle > (255 * 2) && angle <= (255 * 3)) {
    segment_angle = angle - (255 * 2);
    led->colors.r = 0;
    led->colors.g = 255;
    led->colors.b = segment_angle;
  } else if(angle > (255 * 3) && angle <= (255 * 4)) {
    segment_angle = angle - (255 * 3);
    led->colors.r = 0;
    led->colors.g = 255 - segment_angle;
    led->colors.b = 255;
  } else if(angle > (255 * 4) && angle <= (255 * 5)) {
    segment_angle = angle - (255 * 4);
    led->colors.r = segment_angle;
    led->colors.g = 0;
    led->colors.b = 255;
  } else if(angle > (255 * 5) && angle <= (255 * 6)) {
    segment_angle = angle - (255 * 5);
    led->colors.r = 255;
    led->colors.g = 0;
    led->colors.b = 255 - segment_angle;
  }
}

void ws2812_led_init(void)
{
  int i;

  for (i = 0; i < LED_COUNT; i++) {
    ws2812_led_status.leds[i].grbu = 0;
  }

  ws2812_led_status.timer = 0;

  for (i = 0; i < LED_COUNT; i++) {
    ws2812_led_set_wheel(&ws2812_led_status.leds[i], i * 127);
  }
}

void ws2812_led_inc(void)
{
  int i;
  ws2812_led_status.timer = (ws2812_led_status.timer + 10) % 1535;

  for (i = 0; i < LED_COUNT; i++) {
    ws2812_led_set_wheel(&ws2812_led_status.leds[i], (ws2812_led_status.timer + (i * 127)) % 1535);
  }
}

int main(void)
{
  main_init();

  while (1) {
    if (sys_time_check_and_ack_timer(0)) {
      main_periodic_task();
    }
    main_event_task();
  }

  return 0;
}

static inline void main_init(void)
{
  mcu_init();
  sys_time_register_timer((1. / 50), NULL);

  //ms2100_init(&ms2100, &(MS2100_SPI_DEV), MS2100_SLAVE_IDX);
  ws2812_led_init();
  ws2812_init();
  downlink_init();
  mcu_int_enable();
}

static inline void main_periodic_task(void)
{
  RunOnceEvery(10, {
    uint16_t foo = sys_time.nb_sec;
    DOWNLINK_SEND_TAKEOFF(DefaultChannel, DefaultDevice, &foo);
    LED_TOGGLE(2);
    LED_PERIODIC();
  });

  if (!ws2812_is_sending()) {
    ws2812_led_inc();
    ws2812_send(ws2812_led_status.leds, LED_COUNT);
  }

  //ms2100_periodic(&ms2100);

}

static inline void main_event_task(void)
{
  mcu_event();


#if 0
  if (!ws2812_is_sending()) {
    ws2812_led_inc();
    ws2812_send(ws2812_led_status.leds, LED_COUNT);
  ms2100_event(&ms2100);
  if (ms2100.status == MS2100_DATA_AVAILABLE) {
    RunOnceEvery(10, {
      int32_t mag_x = ms2100.data.vect.x;
      int32_t mag_y = ms2100.data.vect.y;
      int32_t mag_z = ms2100.data.vect.z;
      DOWNLINK_SEND_IMU_MAG_RAW(DefaultChannel, DefaultDevice,
      &mag_x, &mag_y, &mag_z);
    });
    ms2100.status = MS2100_IDLE;
  }
#endif
}

