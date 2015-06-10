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
  * @file arch/stm32/peripherals/ws2812_arch.c
  *
  * Driver for the WS2812 RGB led strings with built in driver.
  */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/cm3/nvic.h>

#include "peripherals/ws2812.h"

#include "peripherals/ws2812_arch.h"

#ifndef STM32F4
#error "WS2812 arch currently only implemented for STM32F4"
#endif

/* Private variable definitions. */
struct ws2812_status *ws2812_arch_status;

/* Private function declarations. */
void ws2812_arch_gpio_init(void);
void ws2812_arch_tim_init(void);
void ws2812_arch_dma_init(void);
void ws2812_arch_dma_start(void);
void ws2812_arch_dma_stop(void);

/* API function definitions. */
void ws2812_arch_init(struct ws2812_status *ws2812_status)
{

	ws2812_arch_status = ws2812_status;

	ws2812_arch_gpio_init();
	ws2812_arch_tim_init();
	ws2812_arch_dma_init();
}

void ws2812_arch_send(void)
{
	ws2812_arch_dma_start();
}

/* Private function definitions */
void ws2812_arch_gpio_init(void)
{
	rcc_periph_clock_enable(RCC_GPIOB);
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO6);
    /*gpio_mode_setup(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO13 | GPIO14);*/
    gpio_set_af(GPIOB, GPIO_AF2, GPIO6);
    /* Not sure why this does not really do the job if we have a pullup to 5V */
    gpio_set_output_options(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_2MHZ, GPIO6);
}

void ws2812_arch_tim_init(void)
{
	rcc_periph_clock_enable(RCC_TIM4);
    timer_reset(TIM4);
    timer_set_mode(TIM4, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    timer_set_prescaler(TIM4, 0);
    timer_continuous_mode(TIM4);
    timer_set_period(TIM4, 104); /* 168000000 / 2 / 800000 (800khz pwm) */
    timer_disable_oc_output(TIM4, TIM_OC1);
    timer_disable_oc_clear(TIM4, TIM_OC1);
    timer_enable_oc_preload(TIM4, TIM_OC1);
    timer_set_oc_slow_mode(TIM4, TIM_OC1);
    timer_set_oc_mode(TIM4, TIM_OC1, TIM_OCM_PWM1);
    timer_set_oc_polarity_high(TIM4, TIM_OC1);
    timer_set_oc_value(TIM4, TIM_OC1, 0);
    timer_enable_oc_output(TIM4, TIM_OC1);
    timer_enable_preload(TIM4);

    timer_enable_irq(TIM4, TIM_DIER_UDE);

    timer_enable_counter(TIM4);
}

void ws2812_arch_dma_init(void)
{
	rcc_periph_clock_enable(RCC_DMA1);
	nvic_enable_irq(NVIC_DMA1_STREAM6_IRQ);
	dma_stream_reset(DMA1, DMA_STREAM6);
    dma_set_priority(DMA1, DMA_STREAM6, DMA_SxCR_PL_VERY_HIGH);
    dma_set_memory_size(DMA1, DMA_STREAM6, DMA_SxCR_MSIZE_16BIT);
    dma_set_peripheral_size(DMA1, DMA_STREAM6, DMA_SxCR_PSIZE_16BIT);
    dma_enable_circular_mode(DMA1, DMA_STREAM6);
    dma_enable_memory_increment_mode(DMA1, DMA_STREAM6);
    dma_set_transfer_mode(DMA1, DMA_STREAM6, DMA_SxCR_DIR_MEM_TO_PERIPHERAL);
    dma_set_peripheral_address(DMA1, DMA_STREAM6, (uint32_t)&TIM4_CCR1);
    dma_set_memory_address(DMA1, DMA_STREAM6, (uint32_t)(&ws2812_arch_status->bit_buffer[0]));
    dma_set_number_of_data(DMA1, DMA_STREAM6, WS2812_BIT_BUFFER_SIZE);
    dma_enable_half_transfer_interrupt(DMA1, DMA_STREAM6);
    dma_enable_transfer_complete_interrupt(DMA1, DMA_STREAM6);
    dma_channel_select(DMA1, DMA_STREAM6, DMA_SxCR_CHSEL_2);
    nvic_clear_pending_irq(NVIC_DMA1_STREAM6_IRQ);
    nvic_set_priority(NVIC_DMA1_STREAM6_IRQ, 0);
    nvic_enable_irq(NVIC_DMA1_STREAM6_IRQ);
}

void ws2812_arch_dma_start(void)
{
	dma_enable_stream(DMA1, DMA_STREAM6);
}

void ws2812_arch_dma_stop(void)
{
	dma_disable_stream(DMA1, DMA_STREAM6);
}

/* Interrupt handlers. */
void dma1_stream6_isr(void)
{
    if (dma_get_interrupt_flag(DMA1, DMA_STREAM6, DMA_HTIF) != 0) {
        dma_clear_interrupt_flags(DMA1, DMA_STREAM6, DMA_HTIF);

        //gpio_toggle(GPIOD, GPIO13);

        if(ws2812_arch_status->stage != ws2812_idle){
        	if(ws2812_arch_status->stage == ws2812_done) {
        		ws2812_fill_low_bit_buffer();
        		ws2812_arch_status->stage = ws2812_idle;
        	} else {
        		ws2812_fill_low_bit_buffer();
        	}
        } else {
        	ws2812_clear_bit_buffer();
        	ws2812_arch_dma_stop();
        	timer_set_oc_value(TIM4, TIM_OC1, 0);
        }

    }
    if (dma_get_interrupt_flag(DMA1, DMA_STREAM6, DMA_TCIF) != 0) {
        dma_clear_interrupt_flags(DMA1, DMA_STREAM6, DMA_TCIF);

        //gpio_toggle(GPIOD, GPIO14);

        if(ws2812_arch_status->stage != ws2812_idle){
            if(ws2812_arch_status->stage == ws2812_done) {
        		ws2812_fill_high_bit_buffer();
        	    ws2812_arch_status->stage = ws2812_idle;
            } else {
        	    ws2812_fill_high_bit_buffer();
            }
        } else {
        	ws2812_clear_bit_buffer();
        	ws2812_arch_dma_stop();
        	timer_set_oc_value(TIM4, TIM_OC1, 0);
        }

    }
}