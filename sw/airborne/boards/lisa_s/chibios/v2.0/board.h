/*
    ChibiOS - Copyright (C) 2006..2015 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#pragma once

/*
 * Board identifier.
 */
#define BOARD_LISA_S_2_0
#define BOARD_NAME                  "Lisa/S V2.0 Autopilot"

/*
 * Board oscillators-related settings.
 */
#if !defined(STM32_LSECLK)
#define STM32_LSECLK                32768U
#endif

#if !defined(STM32_HSECLK)
#define STM32_HSECLK                24000000U
#endif

/*
 * Board voltages.
 * Required for performance limits calculation.
 */
#define STM32_VDD                   300U

/*
 * MCU type as defined in the ST header.
 */
#define STM32F745xx

/*
 * IO pins assignments.
 */
#define	UART4_TX                       0U
#define	UART4_RX                       1U
#define	VBAT_MEAS                      2U
#define	ANALOG_IN0                     3U
#define	NC0                            4U
#define	SPI1_SCK                       5U
#define	MPULS5_TIM3_CH1                6U
#define	MPULS6_TIM3_CH2                7U
#define	SERVO4_TIM1_CH1                8U
#define	SERVO2_TIM1_CH2                9U
#define	SERVO3_TIM1_CH3                10U
#define	SERVO1_TIM1_CH4                11U
#define	USB_DP                         12U
#define	SWDIO                          13U
#define	SWCLK                          14U
#define	SPI1_CS                        15U

#define	MPULS3_TIM3_CH3                0U
#define	MPULS4_TIM3_CH4                1U
#define	QSPI_CLK                       2U
#define	SWO                            3U
#define	SPI1_MISO                      4U
#define	SPI1_MOSI                      5U
#define	USART1_TX                      6U
#define	USART1_RX                      7U
#define	I2C1_SCL                       8U
#define	I2C1_SDA                       9U
#define	MPULS1_TIM2_CH3                10U
#define	MPULS2_TIM2_CH4                11U
#define	SPI2_CS                        12U
#define	NC1                            13U
#define	PB14                           14U
#define	PB15                           15U

#define	PC00                           0U
#define	SPI2_MOSI                      1U
#define	SPI2_MISO                      2U
#define	NC2                            3U
#define	NC3                            4U
#define	ANALOG_IN1                     5U
#define	USART6_TX                      6U
#define	USART6_RX                      7U
#define	SDMMC1_D0                      8U
#define	SDMMC1_D1                      9U
#define	SDMMC1_D2                      10U
#define	SDMMC1_D3                      11U
#define	SDMMC1_CK                      12U
#define	LEDB                           13U
#define	LEDG                           14U
#define	LEDR                           15U

#define	NC4                            0U
#define	NC5                            1U
#define	SDMMC1_CMD                     2U
#define	SPI2_SCK                       3U
#define	NC6                            4U
#define	USART2_TX                      5U
#define	USART2_RX                      6U
#define	NC7                            7U
#define	USART3_TX                      8U
#define	USART3_RX                      9U
#define	NC8                            10U
#define	NC9                            11U
#define	NC10                           12U
#define	QSPI_BK1_IO3                   13U
#define	SERVO6_TIM4_CH3                14U
#define	SERVO5_TIM4_CH4                15U

#define	UART8_RX                       0U
#define	UART8_TX                       1U
#define	QSPI_BK1_IO2                   2U
#define	NC11                           3U
#define	IMU_MPU_CS                     4U
#define	IMU_SPI_MISO                   5U
#define	IMU_SPI_MOSI                   6U
#define	GPS_RX                         7U
#define	GPS_TX                         8U
#define	IMU_MPU_INT                    9U
#define	NC12                           10U
#define	NC13                           11U
#define	IMU_SPI_SCK                    12U
#define	IMU_BARO_CS                    13U
#define	NC14                           14U
#define	NC15                           15U

#define	PF00                           0U
#define	PF01                           1U
#define	PF02                           2U
#define	PF03                           3U
#define	PF04                           4U
#define	PF05                           5U
#define	PF06                           6U
#define	PF07                           7U
#define	PF08                           8U
#define	PF09                           9U
#define	PF10                           10U
#define	PF11                           11U
#define	PF12                           12U
#define	PF13                           13U
#define	PF14                           14U
#define	PF15                           15U

#define	PG00                           0U
#define	PG01                           1U
#define	PG02                           2U
#define	PG03                           3U
#define	PG04                           4U
#define	PG05                           5U
#define	PG06                           6U
#define	PG07                           7U
#define	PG08                           8U
#define	PG09                           9U
#define	PG10                           10U
#define	PG11                           11U
#define	PG12                           12U
#define	PG13                           13U
#define	PG14                           14U
#define	PG15                           15U

#define	OSC_IN                         0U
#define	OSC_OUT                        1U
#define	PH02                           2U
#define	PH03                           3U
#define	PH04                           4U
#define	PH05                           5U
#define	PH06                           6U
#define	PH07                           7U
#define	PH08                           8U
#define	PH09                           9U
#define	PH10                           10U
#define	PH11                           11U
#define	PH12                           12U
#define	PH13                           13U
#define	PH14                           14U
#define	PH15                           15U

#define	PI00                           0U
#define	PI01                           1U
#define	PI02                           2U
#define	PI03                           3U
#define	PI04                           4U
#define	PI05                           5U
#define	PI06                           6U
#define	PI07                           7U
#define	PI08                           8U
#define	PI09                           9U
#define	PI10                           10U
#define	PI11                           11U
#define	PI12                           12U
#define	PI13                           13U
#define	PI14                           14U
#define	PI15                           15U

#define	PJ00                           0U
#define	PJ01                           1U
#define	PJ02                           2U
#define	PJ03                           3U
#define	PJ04                           4U
#define	PJ05                           5U
#define	PJ06                           6U
#define	PJ07                           7U
#define	PJ08                           8U
#define	PJ09                           9U
#define	PJ10                           10U
#define	PJ11                           11U
#define	PJ12                           12U
#define	PJ13                           13U
#define	PJ14                           14U
#define	PJ15                           15U

#define	PK00                           0U
#define	PK01                           1U
#define	PK02                           2U
#define	PK03                           3U
#define	PK04                           4U
#define	PK05                           5U
#define	PK06                           6U
#define	PK07                           7U
#define	PK08                           8U
#define	PK09                           9U
#define	PK10                           10U
#define	PK11                           11U
#define	PK12                           12U
#define	PK13                           13U
#define	PK14                           14U
#define	PK15                           15U

/*
 * IO lines assignments.
 */
#define	LINE_UART4_TX                  PAL_LINE(GPIOA, 0U)
#define	LINE_UART4_RX                  PAL_LINE(GPIOA, 1U)
#define	LINE_VBAT_MEAS                 PAL_LINE(GPIOA, 2U)
#define	LINE_ANALOG_IN0                PAL_LINE(GPIOA, 3U)
#define	LINE_NC0                       PAL_LINE(GPIOA, 4U)
#define	LINE_SPI1_SCK                  PAL_LINE(GPIOA, 5U)
#define	LINE_MPULS5_TIM3_CH1           PAL_LINE(GPIOA, 6U)
#define	LINE_MPULS6_TIM3_CH2           PAL_LINE(GPIOA, 7U)
#define	LINE_SERVO4_TIM1_CH1           PAL_LINE(GPIOA, 8U)
#define	LINE_SERVO2_TIM1_CH2           PAL_LINE(GPIOA, 9U)
#define	LINE_SERVO3_TIM1_CH3           PAL_LINE(GPIOA, 10U)
#define	LINE_SERVO1_TIM1_CH4           PAL_LINE(GPIOA, 11U)
#define	LINE_USB_DP                    PAL_LINE(GPIOA, 12U)
#define	LINE_SWDIO                     PAL_LINE(GPIOA, 13U)
#define	LINE_SWCLK                     PAL_LINE(GPIOA, 14U)
#define	LINE_SPI1_CS                   PAL_LINE(GPIOA, 15U)

#define	LINE_MPULS3_TIM3_CH3           PAL_LINE(GPIOB, 0U)
#define	LINE_MPULS4_TIM3_CH4           PAL_LINE(GPIOB, 1U)
#define	LINE_QSPI_CLK                  PAL_LINE(GPIOB, 2U)
#define	LINE_SWO                       PAL_LINE(GPIOB, 3U)
#define	LINE_SPI1_MISO                 PAL_LINE(GPIOB, 4U)
#define	LINE_SPI1_MOSI                 PAL_LINE(GPIOB, 5U)
#define	LINE_USART1_TX                 PAL_LINE(GPIOB, 6U)
#define	LINE_USART1_RX                 PAL_LINE(GPIOB, 7U)
#define	LINE_I2C1_SCL                  PAL_LINE(GPIOB, 8U)
#define	LINE_I2C1_SDA                  PAL_LINE(GPIOB, 9U)
#define	LINE_MPULS1_TIM2_CH3           PAL_LINE(GPIOB, 10U)
#define	LINE_MPULS2_TIM2_CH4           PAL_LINE(GPIOB, 11U)
#define	LINE_SPI2_CS                   PAL_LINE(GPIOB, 12U)
#define	LINE_NC1                       PAL_LINE(GPIOB, 13U)

#define	LINE_SPI2_MOSI                 PAL_LINE(GPIOC, 1U)
#define	LINE_SPI2_MISO                 PAL_LINE(GPIOC, 2U)
#define	LINE_NC2                       PAL_LINE(GPIOC, 3U)
#define	LINE_NC3                       PAL_LINE(GPIOC, 4U)
#define	LINE_ANALOG_IN1                PAL_LINE(GPIOC, 5U)
#define	LINE_USART6_TX                 PAL_LINE(GPIOC, 6U)
#define	LINE_USART6_RX                 PAL_LINE(GPIOC, 7U)
#define	LINE_SDMMC1_D0                 PAL_LINE(GPIOC, 8U)
#define	LINE_SDMMC1_D1                 PAL_LINE(GPIOC, 9U)
#define	LINE_SDMMC1_D2                 PAL_LINE(GPIOC, 10U)
#define	LINE_SDMMC1_D3                 PAL_LINE(GPIOC, 11U)
#define	LINE_SDMMC1_CK                 PAL_LINE(GPIOC, 12U)
#define	LINE_LEDB                      PAL_LINE(GPIOC, 13U)
#define	LINE_LEDG                      PAL_LINE(GPIOC, 14U)
#define	LINE_LEDR                      PAL_LINE(GPIOC, 15U)

#define	LINE_NC4                       PAL_LINE(GPIOD, 0U)
#define	LINE_NC5                       PAL_LINE(GPIOD, 1U)
#define	LINE_SDMMC1_CMD                PAL_LINE(GPIOD, 2U)
#define	LINE_SPI2_SCK                  PAL_LINE(GPIOD, 3U)
#define	LINE_NC6                       PAL_LINE(GPIOD, 4U)
#define	LINE_USART2_TX                 PAL_LINE(GPIOD, 5U)
#define	LINE_USART2_RX                 PAL_LINE(GPIOD, 6U)
#define	LINE_NC7                       PAL_LINE(GPIOD, 7U)
#define	LINE_USART3_TX                 PAL_LINE(GPIOD, 8U)
#define	LINE_USART3_RX                 PAL_LINE(GPIOD, 9U)
#define	LINE_NC8                       PAL_LINE(GPIOD, 10U)
#define	LINE_NC9                       PAL_LINE(GPIOD, 11U)
#define	LINE_NC10                      PAL_LINE(GPIOD, 12U)
#define	LINE_QSPI_BK1_IO3              PAL_LINE(GPIOD, 13U)
#define	LINE_SERVO6_TIM4_CH3           PAL_LINE(GPIOD, 14U)
#define	LINE_SERVO5_TIM4_CH4           PAL_LINE(GPIOD, 15U)

#define	LINE_UART8_RX                  PAL_LINE(GPIOE, 0U)
#define	LINE_UART8_TX                  PAL_LINE(GPIOE, 1U)
#define	LINE_QSPI_BK1_IO2              PAL_LINE(GPIOE, 2U)
#define	LINE_NC11                      PAL_LINE(GPIOE, 3U)
#define	LINE_IMU_MPU_CS                PAL_LINE(GPIOE, 4U)
#define	LINE_IMU_SPI_MISO              PAL_LINE(GPIOE, 5U)
#define	LINE_IMU_SPI_MOSI              PAL_LINE(GPIOE, 6U)
#define	LINE_GPS_RX                    PAL_LINE(GPIOE, 7U)
#define	LINE_GPS_TX                    PAL_LINE(GPIOE, 8U)
#define	LINE_IMU_MPU_INT               PAL_LINE(GPIOE, 9U)
#define	LINE_NC12                      PAL_LINE(GPIOE, 10U)
#define	LINE_NC13                      PAL_LINE(GPIOE, 11U)
#define	LINE_IMU_SPI_SCK               PAL_LINE(GPIOE, 12U)
#define	LINE_IMU_BARO_CS               PAL_LINE(GPIOE, 13U)
#define	LINE_NC14                      PAL_LINE(GPIOE, 14U)
#define	LINE_NC15                      PAL_LINE(GPIOE, 15U)

#define	LINE_OSC_IN                    PAL_LINE(GPIOH, 0U)
#define	LINE_OSC_OUT                   PAL_LINE(GPIOH, 1U)


/*
 * I/O ports initial setup, this configuration is established soon after reset
 * in the initialization code.
 * Please refer to the STM32 Reference Manual for details.
 */
#define PIN_MODE_INPUT(n)           (0U << ((n) * 2U))
#define PIN_MODE_OUTPUT(n)          (1U << ((n) * 2U))
#define PIN_MODE_ALTERNATE(n)       (2U << ((n) * 2U))
#define PIN_MODE_ANALOG(n)          (3U << ((n) * 2U))
#define PIN_ODR_LEVEL_LOW(n)        (0U << (n))
#define PIN_ODR_LEVEL_HIGH(n)       (1U << (n))
#define PIN_OTYPE_PUSHPULL(n)       (0U << (n))
#define PIN_OTYPE_OPENDRAIN(n)      (1U << (n))
#define PIN_OSPEED_SPEED_VERYLOW(n) (0U << ((n) * 2U))
#define PIN_OSPEED_SPEED_LOW(n)     (1U << ((n) * 2U))
#define PIN_OSPEED_SPEED_MEDIUM(n)  (2U << ((n) * 2U))
#define PIN_OSPEED_SPEED_HIGH(n)    (3U << ((n) * 2U))
#define PIN_PUPDR_FLOATING(n)       (0U << ((n) * 2U))
#define PIN_PUPDR_PULLUP(n)         (1U << ((n) * 2U))
#define PIN_PUPDR_PULLDOWN(n)       (2U << ((n) * 2U))
#define PIN_AFIO_AF(n, v)           ((v) << (((n) % 8U) * 4U))

#define VAL_GPIOA_MODER                 (PIN_MODE_ALTERNATE(UART4_TX) | \
					 PIN_MODE_ALTERNATE(UART4_RX) | \
					 PIN_MODE_ANALOG(VBAT_MEAS) | \
					 PIN_MODE_ANALOG(ANALOG_IN0) | \
					 PIN_MODE_INPUT(NC0) | \
					 PIN_MODE_ALTERNATE(SPI1_SCK) | \
					 PIN_MODE_INPUT(MPULS5_TIM3_CH1) | \
					 PIN_MODE_INPUT(MPULS6_TIM3_CH2) | \
					 PIN_MODE_INPUT(SERVO4_TIM1_CH1) | \
					 PIN_MODE_INPUT(SERVO2_TIM1_CH2) | \
					 PIN_MODE_INPUT(SERVO3_TIM1_CH3) | \
					 PIN_MODE_INPUT(SERVO1_TIM1_CH4) | \
					 PIN_MODE_INPUT(USB_DP) | \
					 PIN_MODE_ALTERNATE(SWDIO) | \
					 PIN_MODE_ALTERNATE(SWCLK) | \
					 PIN_MODE_OUTPUT(SPI1_CS))

#define VAL_GPIOA_OTYPER                (PIN_OTYPE_PUSHPULL(UART4_TX) | \
					 PIN_OTYPE_PUSHPULL(UART4_RX) | \
					 PIN_OTYPE_PUSHPULL(VBAT_MEAS) | \
					 PIN_OTYPE_PUSHPULL(ANALOG_IN0) | \
					 PIN_OTYPE_OPENDRAIN(NC0) | \
					 PIN_OTYPE_PUSHPULL(SPI1_SCK) | \
					 PIN_OTYPE_OPENDRAIN(MPULS5_TIM3_CH1) | \
					 PIN_OTYPE_OPENDRAIN(MPULS6_TIM3_CH2) | \
					 PIN_OTYPE_OPENDRAIN(SERVO4_TIM1_CH1) | \
					 PIN_OTYPE_OPENDRAIN(SERVO2_TIM1_CH2) | \
					 PIN_OTYPE_OPENDRAIN(SERVO3_TIM1_CH3) | \
					 PIN_OTYPE_OPENDRAIN(SERVO1_TIM1_CH4) | \
					 PIN_OTYPE_OPENDRAIN(USB_DP) | \
					 PIN_OTYPE_PUSHPULL(SWDIO) | \
					 PIN_OTYPE_PUSHPULL(SWCLK) | \
					 PIN_OTYPE_PUSHPULL(SPI1_CS))

#define VAL_GPIOA_OSPEEDR               (PIN_OSPEED_SPEED_HIGH(UART4_TX) | \
					 PIN_OSPEED_SPEED_HIGH(UART4_RX) | \
					 PIN_OSPEED_SPEED_VERYLOW(VBAT_MEAS) | \
					 PIN_OSPEED_SPEED_VERYLOW(ANALOG_IN0) | \
					 PIN_OSPEED_SPEED_VERYLOW(NC0) | \
					 PIN_OSPEED_SPEED_HIGH(SPI1_SCK) | \
					 PIN_OSPEED_SPEED_VERYLOW(MPULS5_TIM3_CH1) | \
					 PIN_OSPEED_SPEED_VERYLOW(MPULS6_TIM3_CH2) | \
					 PIN_OSPEED_SPEED_VERYLOW(SERVO4_TIM1_CH1) | \
					 PIN_OSPEED_SPEED_VERYLOW(SERVO2_TIM1_CH2) | \
					 PIN_OSPEED_SPEED_VERYLOW(SERVO3_TIM1_CH3) | \
					 PIN_OSPEED_SPEED_VERYLOW(SERVO1_TIM1_CH4) | \
					 PIN_OSPEED_SPEED_VERYLOW(USB_DP) | \
					 PIN_OSPEED_SPEED_HIGH(SWDIO) | \
					 PIN_OSPEED_SPEED_HIGH(SWCLK) | \
					 PIN_OSPEED_SPEED_HIGH(SPI1_CS))

#define VAL_GPIOA_PUPDR                 (PIN_PUPDR_FLOATING(UART4_TX) | \
					 PIN_PUPDR_FLOATING(UART4_RX) | \
					 PIN_PUPDR_FLOATING(VBAT_MEAS) | \
					 PIN_PUPDR_FLOATING(ANALOG_IN0) | \
					 PIN_PUPDR_PULLDOWN(NC0) | \
					 PIN_PUPDR_FLOATING(SPI1_SCK) | \
					 PIN_PUPDR_PULLDOWN(MPULS5_TIM3_CH1) | \
					 PIN_PUPDR_PULLDOWN(MPULS6_TIM3_CH2) | \
					 PIN_PUPDR_PULLDOWN(SERVO4_TIM1_CH1) | \
					 PIN_PUPDR_PULLDOWN(SERVO2_TIM1_CH2) | \
					 PIN_PUPDR_PULLDOWN(SERVO3_TIM1_CH3) | \
					 PIN_PUPDR_PULLDOWN(SERVO1_TIM1_CH4) | \
					 PIN_PUPDR_PULLDOWN(USB_DP) | \
					 PIN_PUPDR_FLOATING(SWDIO) | \
					 PIN_PUPDR_FLOATING(SWCLK) | \
					 PIN_PUPDR_FLOATING(SPI1_CS))

#define VAL_GPIOA_ODR                   (PIN_ODR_LEVEL_HIGH(UART4_TX) | \
					 PIN_ODR_LEVEL_HIGH(UART4_RX) | \
					 PIN_ODR_LEVEL_LOW(VBAT_MEAS) | \
					 PIN_ODR_LEVEL_LOW(ANALOG_IN0) | \
					 PIN_ODR_LEVEL_HIGH(NC0) | \
					 PIN_ODR_LEVEL_HIGH(SPI1_SCK) | \
					 PIN_ODR_LEVEL_HIGH(MPULS5_TIM3_CH1) | \
					 PIN_ODR_LEVEL_HIGH(MPULS6_TIM3_CH2) | \
					 PIN_ODR_LEVEL_HIGH(SERVO4_TIM1_CH1) | \
					 PIN_ODR_LEVEL_HIGH(SERVO2_TIM1_CH2) | \
					 PIN_ODR_LEVEL_HIGH(SERVO3_TIM1_CH3) | \
					 PIN_ODR_LEVEL_HIGH(SERVO1_TIM1_CH4) | \
					 PIN_ODR_LEVEL_HIGH(USB_DP) | \
					 PIN_ODR_LEVEL_HIGH(SWDIO) | \
					 PIN_ODR_LEVEL_HIGH(SWCLK) | \
					 PIN_ODR_LEVEL_HIGH(SPI1_CS))

#define VAL_GPIOA_AFRL			(PIN_AFIO_AF(UART4_TX, 8) | \
					 PIN_AFIO_AF(UART4_RX, 8) | \
					 PIN_AFIO_AF(VBAT_MEAS, 0) | \
					 PIN_AFIO_AF(ANALOG_IN0, 0) | \
					 PIN_AFIO_AF(NC0, 0) | \
					 PIN_AFIO_AF(SPI1_SCK, 5) | \
					 PIN_AFIO_AF(MPULS5_TIM3_CH1, 0) | \
					 PIN_AFIO_AF(MPULS6_TIM3_CH2, 0))

#define VAL_GPIOA_AFRH			(PIN_AFIO_AF(SERVO4_TIM1_CH1, 0) | \
					 PIN_AFIO_AF(SERVO2_TIM1_CH2, 0) | \
					 PIN_AFIO_AF(SERVO3_TIM1_CH3, 0) | \
					 PIN_AFIO_AF(SERVO1_TIM1_CH4, 0) | \
					 PIN_AFIO_AF(USB_DP, 0) | \
					 PIN_AFIO_AF(SWDIO, 0) | \
					 PIN_AFIO_AF(SWCLK, 0) | \
					 PIN_AFIO_AF(SPI1_CS, 0))

#define VAL_GPIOB_MODER                 (PIN_MODE_INPUT(MPULS3_TIM3_CH3) | \
					 PIN_MODE_INPUT(MPULS4_TIM3_CH4) | \
					 PIN_MODE_INPUT(QSPI_CLK) | \
					 PIN_MODE_ALTERNATE(SWO) | \
					 PIN_MODE_ALTERNATE(SPI1_MISO) | \
					 PIN_MODE_ALTERNATE(SPI1_MOSI) | \
					 PIN_MODE_ALTERNATE(USART1_TX) | \
					 PIN_MODE_ALTERNATE(USART1_RX) | \
					 PIN_MODE_ALTERNATE(I2C1_SCL) | \
					 PIN_MODE_ALTERNATE(I2C1_SDA) | \
					 PIN_MODE_INPUT(MPULS1_TIM2_CH3) | \
					 PIN_MODE_INPUT(MPULS2_TIM2_CH4) | \
					 PIN_MODE_OUTPUT(SPI2_CS) | \
					 PIN_MODE_INPUT(NC1) | \
					 PIN_MODE_INPUT(PB14) | \
					 PIN_MODE_INPUT(PB15))

#define VAL_GPIOB_OTYPER                (PIN_OTYPE_OPENDRAIN(MPULS3_TIM3_CH3) | \
					 PIN_OTYPE_OPENDRAIN(MPULS4_TIM3_CH4) | \
					 PIN_OTYPE_OPENDRAIN(QSPI_CLK) | \
					 PIN_OTYPE_PUSHPULL(SWO) | \
					 PIN_OTYPE_PUSHPULL(SPI1_MISO) | \
					 PIN_OTYPE_PUSHPULL(SPI1_MOSI) | \
					 PIN_OTYPE_PUSHPULL(USART1_TX) | \
					 PIN_OTYPE_PUSHPULL(USART1_RX) | \
					 PIN_OTYPE_OPENDRAIN(I2C1_SCL) | \
					 PIN_OTYPE_OPENDRAIN(I2C1_SDA) | \
					 PIN_OTYPE_OPENDRAIN(MPULS1_TIM2_CH3) | \
					 PIN_OTYPE_OPENDRAIN(MPULS2_TIM2_CH4) | \
					 PIN_OTYPE_PUSHPULL(SPI2_CS) | \
					 PIN_OTYPE_OPENDRAIN(NC1) | \
					 PIN_OTYPE_PUSHPULL(PB14) | \
					 PIN_OTYPE_PUSHPULL(PB15))

#define VAL_GPIOB_OSPEEDR               (PIN_OSPEED_SPEED_VERYLOW(MPULS3_TIM3_CH3) | \
					 PIN_OSPEED_SPEED_VERYLOW(MPULS4_TIM3_CH4) | \
					 PIN_OSPEED_SPEED_VERYLOW(QSPI_CLK) | \
					 PIN_OSPEED_SPEED_HIGH(SWO) | \
					 PIN_OSPEED_SPEED_HIGH(SPI1_MISO) | \
					 PIN_OSPEED_SPEED_HIGH(SPI1_MOSI) | \
					 PIN_OSPEED_SPEED_HIGH(USART1_TX) | \
					 PIN_OSPEED_SPEED_HIGH(USART1_RX) | \
					 PIN_OSPEED_SPEED_HIGH(I2C1_SCL) | \
					 PIN_OSPEED_SPEED_HIGH(I2C1_SDA) | \
					 PIN_OSPEED_SPEED_VERYLOW(MPULS1_TIM2_CH3) | \
					 PIN_OSPEED_SPEED_VERYLOW(MPULS2_TIM2_CH4) | \
					 PIN_OSPEED_SPEED_HIGH(SPI2_CS) | \
					 PIN_OSPEED_SPEED_VERYLOW(NC1) | \
					 PIN_OSPEED_SPEED_VERYLOW(PB14) | \
					 PIN_OSPEED_SPEED_VERYLOW(PB15))

#define VAL_GPIOB_PUPDR                 (PIN_PUPDR_PULLDOWN(MPULS3_TIM3_CH3) | \
					 PIN_PUPDR_PULLDOWN(MPULS4_TIM3_CH4) | \
					 PIN_PUPDR_PULLDOWN(QSPI_CLK) | \
					 PIN_PUPDR_FLOATING(SWO) | \
					 PIN_PUPDR_FLOATING(SPI1_MISO) | \
					 PIN_PUPDR_FLOATING(SPI1_MOSI) | \
					 PIN_PUPDR_FLOATING(USART1_TX) | \
					 PIN_PUPDR_FLOATING(USART1_RX) | \
					 PIN_PUPDR_PULLUP(I2C1_SCL) | \
					 PIN_PUPDR_PULLUP(I2C1_SDA) | \
					 PIN_PUPDR_PULLDOWN(MPULS1_TIM2_CH3) | \
					 PIN_PUPDR_PULLDOWN(MPULS2_TIM2_CH4) | \
					 PIN_PUPDR_FLOATING(SPI2_CS) | \
					 PIN_PUPDR_PULLDOWN(NC1) | \
					 PIN_PUPDR_PULLDOWN(PB14) | \
					 PIN_PUPDR_PULLDOWN(PB15))

#define VAL_GPIOB_ODR                   (PIN_ODR_LEVEL_HIGH(MPULS3_TIM3_CH3) | \
					 PIN_ODR_LEVEL_HIGH(MPULS4_TIM3_CH4) | \
					 PIN_ODR_LEVEL_HIGH(QSPI_CLK) | \
					 PIN_ODR_LEVEL_HIGH(SWO) | \
					 PIN_ODR_LEVEL_HIGH(SPI1_MISO) | \
					 PIN_ODR_LEVEL_HIGH(SPI1_MOSI) | \
					 PIN_ODR_LEVEL_HIGH(USART1_TX) | \
					 PIN_ODR_LEVEL_HIGH(USART1_RX) | \
					 PIN_ODR_LEVEL_HIGH(I2C1_SCL) | \
					 PIN_ODR_LEVEL_HIGH(I2C1_SDA) | \
					 PIN_ODR_LEVEL_HIGH(MPULS1_TIM2_CH3) | \
					 PIN_ODR_LEVEL_HIGH(MPULS2_TIM2_CH4) | \
					 PIN_ODR_LEVEL_HIGH(SPI2_CS) | \
					 PIN_ODR_LEVEL_HIGH(NC1) | \
					 PIN_ODR_LEVEL_LOW(PB14) | \
					 PIN_ODR_LEVEL_LOW(PB15))

#define VAL_GPIOB_AFRL			(PIN_AFIO_AF(MPULS3_TIM3_CH3, 0) | \
					 PIN_AFIO_AF(MPULS4_TIM3_CH4, 0) | \
					 PIN_AFIO_AF(QSPI_CLK, 0) | \
					 PIN_AFIO_AF(SWO, 0) | \
					 PIN_AFIO_AF(SPI1_MISO, 5) | \
					 PIN_AFIO_AF(SPI1_MOSI, 5) | \
					 PIN_AFIO_AF(USART1_TX, 7) | \
					 PIN_AFIO_AF(USART1_RX, 7))

#define VAL_GPIOB_AFRH			(PIN_AFIO_AF(I2C1_SCL, 4) | \
					 PIN_AFIO_AF(I2C1_SDA, 4) | \
					 PIN_AFIO_AF(MPULS1_TIM2_CH3, 0) | \
					 PIN_AFIO_AF(MPULS2_TIM2_CH4, 0) | \
					 PIN_AFIO_AF(SPI2_CS, 0) | \
					 PIN_AFIO_AF(NC1, 0) | \
					 PIN_AFIO_AF(PB14, 0) | \
					 PIN_AFIO_AF(PB15, 0))

#define VAL_GPIOC_MODER                 (PIN_MODE_INPUT(PC00) | \
					 PIN_MODE_ALTERNATE(SPI2_MOSI) | \
					 PIN_MODE_ALTERNATE(SPI2_MISO) | \
					 PIN_MODE_INPUT(NC2) | \
					 PIN_MODE_INPUT(NC3) | \
					 PIN_MODE_ANALOG(ANALOG_IN1) | \
					 PIN_MODE_INPUT(USART6_TX) | \
					 PIN_MODE_INPUT(USART6_RX) | \
					 PIN_MODE_ALTERNATE(SDMMC1_D0) | \
					 PIN_MODE_ALTERNATE(SDMMC1_D1) | \
					 PIN_MODE_ALTERNATE(SDMMC1_D2) | \
					 PIN_MODE_ALTERNATE(SDMMC1_D3) | \
					 PIN_MODE_ALTERNATE(SDMMC1_CK) | \
					 PIN_MODE_OUTPUT(LEDB) | \
					 PIN_MODE_OUTPUT(LEDG) | \
					 PIN_MODE_OUTPUT(LEDR))

#define VAL_GPIOC_OTYPER                (PIN_OTYPE_PUSHPULL(PC00) | \
					 PIN_OTYPE_PUSHPULL(SPI2_MOSI) | \
					 PIN_OTYPE_PUSHPULL(SPI2_MISO) | \
					 PIN_OTYPE_OPENDRAIN(NC2) | \
					 PIN_OTYPE_OPENDRAIN(NC3) | \
					 PIN_OTYPE_PUSHPULL(ANALOG_IN1) | \
					 PIN_OTYPE_OPENDRAIN(USART6_TX) | \
					 PIN_OTYPE_OPENDRAIN(USART6_RX) | \
					 PIN_OTYPE_PUSHPULL(SDMMC1_D0) | \
					 PIN_OTYPE_PUSHPULL(SDMMC1_D1) | \
					 PIN_OTYPE_PUSHPULL(SDMMC1_D2) | \
					 PIN_OTYPE_PUSHPULL(SDMMC1_D3) | \
					 PIN_OTYPE_PUSHPULL(SDMMC1_CK) | \
					 PIN_OTYPE_PUSHPULL(LEDB) | \
					 PIN_OTYPE_PUSHPULL(LEDG) | \
					 PIN_OTYPE_PUSHPULL(LEDR))

#define VAL_GPIOC_OSPEEDR               (PIN_OSPEED_SPEED_VERYLOW(PC00) | \
					 PIN_OSPEED_SPEED_HIGH(SPI2_MOSI) | \
					 PIN_OSPEED_SPEED_HIGH(SPI2_MISO) | \
					 PIN_OSPEED_SPEED_VERYLOW(NC2) | \
					 PIN_OSPEED_SPEED_VERYLOW(NC3) | \
					 PIN_OSPEED_SPEED_VERYLOW(ANALOG_IN1) | \
					 PIN_OSPEED_SPEED_VERYLOW(USART6_TX) | \
					 PIN_OSPEED_SPEED_VERYLOW(USART6_RX) | \
					 PIN_OSPEED_SPEED_HIGH(SDMMC1_D0) | \
					 PIN_OSPEED_SPEED_HIGH(SDMMC1_D1) | \
					 PIN_OSPEED_SPEED_HIGH(SDMMC1_D2) | \
					 PIN_OSPEED_SPEED_HIGH(SDMMC1_D3) | \
					 PIN_OSPEED_SPEED_HIGH(SDMMC1_CK) | \
					 PIN_OSPEED_SPEED_VERYLOW(LEDB) | \
					 PIN_OSPEED_SPEED_VERYLOW(LEDG) | \
					 PIN_OSPEED_SPEED_VERYLOW(LEDR))

#define VAL_GPIOC_PUPDR                 (PIN_PUPDR_PULLDOWN(PC00) | \
					 PIN_PUPDR_FLOATING(SPI2_MOSI) | \
					 PIN_PUPDR_FLOATING(SPI2_MISO) | \
					 PIN_PUPDR_PULLDOWN(NC2) | \
					 PIN_PUPDR_PULLDOWN(NC3) | \
					 PIN_PUPDR_FLOATING(ANALOG_IN1) | \
					 PIN_PUPDR_PULLDOWN(USART6_TX) | \
					 PIN_PUPDR_PULLDOWN(USART6_RX) | \
					 PIN_PUPDR_PULLUP(SDMMC1_D0) | \
					 PIN_PUPDR_PULLUP(SDMMC1_D1) | \
					 PIN_PUPDR_PULLUP(SDMMC1_D2) | \
					 PIN_PUPDR_PULLUP(SDMMC1_D3) | \
					 PIN_PUPDR_PULLUP(SDMMC1_CK) | \
					 PIN_PUPDR_FLOATING(LEDB) | \
					 PIN_PUPDR_FLOATING(LEDG) | \
					 PIN_PUPDR_FLOATING(LEDR))

#define VAL_GPIOC_ODR                   (PIN_ODR_LEVEL_LOW(PC00) | \
					 PIN_ODR_LEVEL_HIGH(SPI2_MOSI) | \
					 PIN_ODR_LEVEL_HIGH(SPI2_MISO) | \
					 PIN_ODR_LEVEL_HIGH(NC2) | \
					 PIN_ODR_LEVEL_HIGH(NC3) | \
					 PIN_ODR_LEVEL_LOW(ANALOG_IN1) | \
					 PIN_ODR_LEVEL_HIGH(USART6_TX) | \
					 PIN_ODR_LEVEL_HIGH(USART6_RX) | \
					 PIN_ODR_LEVEL_HIGH(SDMMC1_D0) | \
					 PIN_ODR_LEVEL_HIGH(SDMMC1_D1) | \
					 PIN_ODR_LEVEL_HIGH(SDMMC1_D2) | \
					 PIN_ODR_LEVEL_HIGH(SDMMC1_D3) | \
					 PIN_ODR_LEVEL_HIGH(SDMMC1_CK) | \
					 PIN_ODR_LEVEL_LOW(LEDB) | \
					 PIN_ODR_LEVEL_LOW(LEDG) | \
					 PIN_ODR_LEVEL_LOW(LEDR))

#define VAL_GPIOC_AFRL			(PIN_AFIO_AF(PC00, 0) | \
					 PIN_AFIO_AF(SPI2_MOSI, 5) | \
					 PIN_AFIO_AF(SPI2_MISO, 5) | \
					 PIN_AFIO_AF(NC2, 0) | \
					 PIN_AFIO_AF(NC3, 0) | \
					 PIN_AFIO_AF(ANALOG_IN1, 0) | \
					 PIN_AFIO_AF(USART6_TX, 0) | \
					 PIN_AFIO_AF(USART6_RX, 0))

#define VAL_GPIOC_AFRH			(PIN_AFIO_AF(SDMMC1_D0, 12) | \
					 PIN_AFIO_AF(SDMMC1_D1, 12) | \
					 PIN_AFIO_AF(SDMMC1_D2, 12) | \
					 PIN_AFIO_AF(SDMMC1_D3, 12) | \
					 PIN_AFIO_AF(SDMMC1_CK, 12) | \
					 PIN_AFIO_AF(LEDB, 0) | \
					 PIN_AFIO_AF(LEDG, 0) | \
					 PIN_AFIO_AF(LEDR, 0))

#define VAL_GPIOD_MODER                 (PIN_MODE_INPUT(NC4) | \
					 PIN_MODE_INPUT(NC5) | \
					 PIN_MODE_ALTERNATE(SDMMC1_CMD) | \
					 PIN_MODE_ALTERNATE(SPI2_SCK) | \
					 PIN_MODE_INPUT(NC6) | \
					 PIN_MODE_INPUT(USART2_TX) | \
					 PIN_MODE_INPUT(USART2_RX) | \
					 PIN_MODE_INPUT(NC7) | \
					 PIN_MODE_INPUT(USART3_TX) | \
					 PIN_MODE_INPUT(USART3_RX) | \
					 PIN_MODE_INPUT(NC8) | \
					 PIN_MODE_INPUT(NC9) | \
					 PIN_MODE_INPUT(NC10) | \
					 PIN_MODE_INPUT(QSPI_BK1_IO3) | \
					 PIN_MODE_INPUT(SERVO6_TIM4_CH3) | \
					 PIN_MODE_INPUT(SERVO5_TIM4_CH4))

#define VAL_GPIOD_OTYPER                (PIN_OTYPE_OPENDRAIN(NC4) | \
					 PIN_OTYPE_OPENDRAIN(NC5) | \
					 PIN_OTYPE_PUSHPULL(SDMMC1_CMD) | \
					 PIN_OTYPE_PUSHPULL(SPI2_SCK) | \
					 PIN_OTYPE_OPENDRAIN(NC6) | \
					 PIN_OTYPE_OPENDRAIN(USART2_TX) | \
					 PIN_OTYPE_OPENDRAIN(USART2_RX) | \
					 PIN_OTYPE_OPENDRAIN(NC7) | \
					 PIN_OTYPE_OPENDRAIN(USART3_TX) | \
					 PIN_OTYPE_OPENDRAIN(USART3_RX) | \
					 PIN_OTYPE_OPENDRAIN(NC8) | \
					 PIN_OTYPE_OPENDRAIN(NC9) | \
					 PIN_OTYPE_OPENDRAIN(NC10) | \
					 PIN_OTYPE_OPENDRAIN(QSPI_BK1_IO3) | \
					 PIN_OTYPE_OPENDRAIN(SERVO6_TIM4_CH3) | \
					 PIN_OTYPE_OPENDRAIN(SERVO5_TIM4_CH4))

#define VAL_GPIOD_OSPEEDR               (PIN_OSPEED_SPEED_VERYLOW(NC4) | \
					 PIN_OSPEED_SPEED_VERYLOW(NC5) | \
					 PIN_OSPEED_SPEED_HIGH(SDMMC1_CMD) | \
					 PIN_OSPEED_SPEED_HIGH(SPI2_SCK) | \
					 PIN_OSPEED_SPEED_VERYLOW(NC6) | \
					 PIN_OSPEED_SPEED_VERYLOW(USART2_TX) | \
					 PIN_OSPEED_SPEED_VERYLOW(USART2_RX) | \
					 PIN_OSPEED_SPEED_VERYLOW(NC7) | \
					 PIN_OSPEED_SPEED_VERYLOW(USART3_TX) | \
					 PIN_OSPEED_SPEED_VERYLOW(USART3_RX) | \
					 PIN_OSPEED_SPEED_VERYLOW(NC8) | \
					 PIN_OSPEED_SPEED_VERYLOW(NC9) | \
					 PIN_OSPEED_SPEED_VERYLOW(NC10) | \
					 PIN_OSPEED_SPEED_VERYLOW(QSPI_BK1_IO3) | \
					 PIN_OSPEED_SPEED_VERYLOW(SERVO6_TIM4_CH3) | \
					 PIN_OSPEED_SPEED_VERYLOW(SERVO5_TIM4_CH4))

#define VAL_GPIOD_PUPDR                 (PIN_PUPDR_PULLDOWN(NC4) | \
					 PIN_PUPDR_PULLDOWN(NC5) | \
					 PIN_PUPDR_PULLUP(SDMMC1_CMD) | \
					 PIN_PUPDR_FLOATING(SPI2_SCK) | \
					 PIN_PUPDR_PULLDOWN(NC6) | \
					 PIN_PUPDR_PULLDOWN(USART2_TX) | \
					 PIN_PUPDR_PULLDOWN(USART2_RX) | \
					 PIN_PUPDR_PULLDOWN(NC7) | \
					 PIN_PUPDR_PULLDOWN(USART3_TX) | \
					 PIN_PUPDR_PULLDOWN(USART3_RX) | \
					 PIN_PUPDR_PULLDOWN(NC8) | \
					 PIN_PUPDR_PULLDOWN(NC9) | \
					 PIN_PUPDR_PULLDOWN(NC10) | \
					 PIN_PUPDR_PULLDOWN(QSPI_BK1_IO3) | \
					 PIN_PUPDR_PULLDOWN(SERVO6_TIM4_CH3) | \
					 PIN_PUPDR_PULLDOWN(SERVO5_TIM4_CH4))

#define VAL_GPIOD_ODR                   (PIN_ODR_LEVEL_HIGH(NC4) | \
					 PIN_ODR_LEVEL_HIGH(NC5) | \
					 PIN_ODR_LEVEL_HIGH(SDMMC1_CMD) | \
					 PIN_ODR_LEVEL_HIGH(SPI2_SCK) | \
					 PIN_ODR_LEVEL_HIGH(NC6) | \
					 PIN_ODR_LEVEL_HIGH(USART2_TX) | \
					 PIN_ODR_LEVEL_HIGH(USART2_RX) | \
					 PIN_ODR_LEVEL_HIGH(NC7) | \
					 PIN_ODR_LEVEL_HIGH(USART3_TX) | \
					 PIN_ODR_LEVEL_HIGH(USART3_RX) | \
					 PIN_ODR_LEVEL_HIGH(NC8) | \
					 PIN_ODR_LEVEL_HIGH(NC9) | \
					 PIN_ODR_LEVEL_HIGH(NC10) | \
					 PIN_ODR_LEVEL_HIGH(QSPI_BK1_IO3) | \
					 PIN_ODR_LEVEL_HIGH(SERVO6_TIM4_CH3) | \
					 PIN_ODR_LEVEL_HIGH(SERVO5_TIM4_CH4))

#define VAL_GPIOD_AFRL			(PIN_AFIO_AF(NC4, 0) | \
					 PIN_AFIO_AF(NC5, 0) | \
					 PIN_AFIO_AF(SDMMC1_CMD, 12) | \
					 PIN_AFIO_AF(SPI2_SCK, 5) | \
					 PIN_AFIO_AF(NC6, 0) | \
					 PIN_AFIO_AF(USART2_TX, 0) | \
					 PIN_AFIO_AF(USART2_RX, 0) | \
					 PIN_AFIO_AF(NC7, 0))

#define VAL_GPIOD_AFRH			(PIN_AFIO_AF(USART3_TX, 0) | \
					 PIN_AFIO_AF(USART3_RX, 0) | \
					 PIN_AFIO_AF(NC8, 0) | \
					 PIN_AFIO_AF(NC9, 0) | \
					 PIN_AFIO_AF(NC10, 0) | \
					 PIN_AFIO_AF(QSPI_BK1_IO3, 0) | \
					 PIN_AFIO_AF(SERVO6_TIM4_CH3, 0) | \
					 PIN_AFIO_AF(SERVO5_TIM4_CH4, 0))

#define VAL_GPIOE_MODER                 (PIN_MODE_ALTERNATE(UART8_RX) | \
					 PIN_MODE_ALTERNATE(UART8_TX) | \
					 PIN_MODE_INPUT(QSPI_BK1_IO2) | \
					 PIN_MODE_INPUT(NC11) | \
					 PIN_MODE_OUTPUT(IMU_MPU_CS) | \
					 PIN_MODE_ALTERNATE(IMU_SPI_MISO) | \
					 PIN_MODE_ALTERNATE(IMU_SPI_MOSI) | \
					 PIN_MODE_ALTERNATE(GPS_RX) | \
					 PIN_MODE_ALTERNATE(GPS_TX) | \
					 PIN_MODE_INPUT(IMU_MPU_INT) | \
					 PIN_MODE_INPUT(NC12) | \
					 PIN_MODE_INPUT(NC13) | \
					 PIN_MODE_ALTERNATE(IMU_SPI_SCK) | \
					 PIN_MODE_OUTPUT(IMU_BARO_CS) | \
					 PIN_MODE_INPUT(NC14) | \
					 PIN_MODE_INPUT(NC15))

#define VAL_GPIOE_OTYPER                (PIN_OTYPE_PUSHPULL(UART8_RX) | \
					 PIN_OTYPE_PUSHPULL(UART8_TX) | \
					 PIN_OTYPE_OPENDRAIN(QSPI_BK1_IO2) | \
					 PIN_OTYPE_OPENDRAIN(NC11) | \
					 PIN_OTYPE_PUSHPULL(IMU_MPU_CS) | \
					 PIN_OTYPE_PUSHPULL(IMU_SPI_MISO) | \
					 PIN_OTYPE_PUSHPULL(IMU_SPI_MOSI) | \
					 PIN_OTYPE_PUSHPULL(GPS_RX) | \
					 PIN_OTYPE_PUSHPULL(GPS_TX) | \
					 PIN_OTYPE_OPENDRAIN(IMU_MPU_INT) | \
					 PIN_OTYPE_OPENDRAIN(NC12) | \
					 PIN_OTYPE_OPENDRAIN(NC13) | \
					 PIN_OTYPE_PUSHPULL(IMU_SPI_SCK) | \
					 PIN_OTYPE_PUSHPULL(IMU_BARO_CS) | \
					 PIN_OTYPE_OPENDRAIN(NC14) | \
					 PIN_OTYPE_OPENDRAIN(NC15))

#define VAL_GPIOE_OSPEEDR               (PIN_OSPEED_SPEED_HIGH(UART8_RX) | \
					 PIN_OSPEED_SPEED_HIGH(UART8_TX) | \
					 PIN_OSPEED_SPEED_VERYLOW(QSPI_BK1_IO2) | \
					 PIN_OSPEED_SPEED_VERYLOW(NC11) | \
					 PIN_OSPEED_SPEED_HIGH(IMU_MPU_CS) | \
					 PIN_OSPEED_SPEED_HIGH(IMU_SPI_MISO) | \
					 PIN_OSPEED_SPEED_HIGH(IMU_SPI_MOSI) | \
					 PIN_OSPEED_SPEED_HIGH(GPS_RX) | \
					 PIN_OSPEED_SPEED_HIGH(GPS_TX) | \
					 PIN_OSPEED_SPEED_VERYLOW(IMU_MPU_INT) | \
					 PIN_OSPEED_SPEED_VERYLOW(NC12) | \
					 PIN_OSPEED_SPEED_VERYLOW(NC13) | \
					 PIN_OSPEED_SPEED_HIGH(IMU_SPI_SCK) | \
					 PIN_OSPEED_SPEED_HIGH(IMU_BARO_CS) | \
					 PIN_OSPEED_SPEED_VERYLOW(NC14) | \
					 PIN_OSPEED_SPEED_VERYLOW(NC15))

#define VAL_GPIOE_PUPDR                 (PIN_PUPDR_FLOATING(UART8_RX) | \
					 PIN_PUPDR_FLOATING(UART8_TX) | \
					 PIN_PUPDR_PULLDOWN(QSPI_BK1_IO2) | \
					 PIN_PUPDR_PULLDOWN(NC11) | \
					 PIN_PUPDR_FLOATING(IMU_MPU_CS) | \
					 PIN_PUPDR_FLOATING(IMU_SPI_MISO) | \
					 PIN_PUPDR_FLOATING(IMU_SPI_MOSI) | \
					 PIN_PUPDR_FLOATING(GPS_RX) | \
					 PIN_PUPDR_FLOATING(GPS_TX) | \
					 PIN_PUPDR_FLOATING(IMU_MPU_INT) | \
					 PIN_PUPDR_PULLDOWN(NC12) | \
					 PIN_PUPDR_PULLDOWN(NC13) | \
					 PIN_PUPDR_FLOATING(IMU_SPI_SCK) | \
					 PIN_PUPDR_FLOATING(IMU_BARO_CS) | \
					 PIN_PUPDR_PULLDOWN(NC14) | \
					 PIN_PUPDR_PULLDOWN(NC15))

#define VAL_GPIOE_ODR                   (PIN_ODR_LEVEL_HIGH(UART8_RX) | \
					 PIN_ODR_LEVEL_HIGH(UART8_TX) | \
					 PIN_ODR_LEVEL_HIGH(QSPI_BK1_IO2) | \
					 PIN_ODR_LEVEL_HIGH(NC11) | \
					 PIN_ODR_LEVEL_HIGH(IMU_MPU_CS) | \
					 PIN_ODR_LEVEL_HIGH(IMU_SPI_MISO) | \
					 PIN_ODR_LEVEL_HIGH(IMU_SPI_MOSI) | \
					 PIN_ODR_LEVEL_HIGH(GPS_RX) | \
					 PIN_ODR_LEVEL_HIGH(GPS_TX) | \
					 PIN_ODR_LEVEL_LOW(IMU_MPU_INT) | \
					 PIN_ODR_LEVEL_HIGH(NC12) | \
					 PIN_ODR_LEVEL_HIGH(NC13) | \
					 PIN_ODR_LEVEL_HIGH(IMU_SPI_SCK) | \
					 PIN_ODR_LEVEL_HIGH(IMU_BARO_CS) | \
					 PIN_ODR_LEVEL_HIGH(NC14) | \
					 PIN_ODR_LEVEL_HIGH(NC15))

#define VAL_GPIOE_AFRL			(PIN_AFIO_AF(UART8_RX, 8) | \
					 PIN_AFIO_AF(UART8_TX, 8) | \
					 PIN_AFIO_AF(QSPI_BK1_IO2, 0) | \
					 PIN_AFIO_AF(NC11, 0) | \
					 PIN_AFIO_AF(IMU_MPU_CS, 0) | \
					 PIN_AFIO_AF(IMU_SPI_MISO, 5) | \
					 PIN_AFIO_AF(IMU_SPI_MOSI, 5) | \
					 PIN_AFIO_AF(GPS_RX, 8))

#define VAL_GPIOE_AFRH			(PIN_AFIO_AF(GPS_TX, 8) | \
					 PIN_AFIO_AF(IMU_MPU_INT, 0) | \
					 PIN_AFIO_AF(NC12, 0) | \
					 PIN_AFIO_AF(NC13, 0) | \
					 PIN_AFIO_AF(IMU_SPI_SCK, 5) | \
					 PIN_AFIO_AF(IMU_BARO_CS, 0) | \
					 PIN_AFIO_AF(NC14, 0) | \
					 PIN_AFIO_AF(NC15, 0))

#define VAL_GPIOF_MODER                 (PIN_MODE_INPUT(PF00) | \
					 PIN_MODE_INPUT(PF01) | \
					 PIN_MODE_INPUT(PF02) | \
					 PIN_MODE_INPUT(PF03) | \
					 PIN_MODE_INPUT(PF04) | \
					 PIN_MODE_INPUT(PF05) | \
					 PIN_MODE_INPUT(PF06) | \
					 PIN_MODE_INPUT(PF07) | \
					 PIN_MODE_INPUT(PF08) | \
					 PIN_MODE_INPUT(PF09) | \
					 PIN_MODE_INPUT(PF10) | \
					 PIN_MODE_INPUT(PF11) | \
					 PIN_MODE_INPUT(PF12) | \
					 PIN_MODE_INPUT(PF13) | \
					 PIN_MODE_INPUT(PF14) | \
					 PIN_MODE_INPUT(PF15))

#define VAL_GPIOF_OTYPER                (PIN_OTYPE_PUSHPULL(PF00) | \
					 PIN_OTYPE_PUSHPULL(PF01) | \
					 PIN_OTYPE_PUSHPULL(PF02) | \
					 PIN_OTYPE_PUSHPULL(PF03) | \
					 PIN_OTYPE_PUSHPULL(PF04) | \
					 PIN_OTYPE_PUSHPULL(PF05) | \
					 PIN_OTYPE_PUSHPULL(PF06) | \
					 PIN_OTYPE_PUSHPULL(PF07) | \
					 PIN_OTYPE_PUSHPULL(PF08) | \
					 PIN_OTYPE_PUSHPULL(PF09) | \
					 PIN_OTYPE_PUSHPULL(PF10) | \
					 PIN_OTYPE_PUSHPULL(PF11) | \
					 PIN_OTYPE_PUSHPULL(PF12) | \
					 PIN_OTYPE_PUSHPULL(PF13) | \
					 PIN_OTYPE_PUSHPULL(PF14) | \
					 PIN_OTYPE_PUSHPULL(PF15))

#define VAL_GPIOF_OSPEEDR               (PIN_OSPEED_SPEED_VERYLOW(PF00) | \
					 PIN_OSPEED_SPEED_VERYLOW(PF01) | \
					 PIN_OSPEED_SPEED_VERYLOW(PF02) | \
					 PIN_OSPEED_SPEED_VERYLOW(PF03) | \
					 PIN_OSPEED_SPEED_VERYLOW(PF04) | \
					 PIN_OSPEED_SPEED_VERYLOW(PF05) | \
					 PIN_OSPEED_SPEED_VERYLOW(PF06) | \
					 PIN_OSPEED_SPEED_VERYLOW(PF07) | \
					 PIN_OSPEED_SPEED_VERYLOW(PF08) | \
					 PIN_OSPEED_SPEED_VERYLOW(PF09) | \
					 PIN_OSPEED_SPEED_VERYLOW(PF10) | \
					 PIN_OSPEED_SPEED_VERYLOW(PF11) | \
					 PIN_OSPEED_SPEED_VERYLOW(PF12) | \
					 PIN_OSPEED_SPEED_VERYLOW(PF13) | \
					 PIN_OSPEED_SPEED_VERYLOW(PF14) | \
					 PIN_OSPEED_SPEED_VERYLOW(PF15))

#define VAL_GPIOF_PUPDR                 (PIN_PUPDR_PULLDOWN(PF00) | \
					 PIN_PUPDR_PULLDOWN(PF01) | \
					 PIN_PUPDR_PULLDOWN(PF02) | \
					 PIN_PUPDR_PULLDOWN(PF03) | \
					 PIN_PUPDR_PULLDOWN(PF04) | \
					 PIN_PUPDR_PULLDOWN(PF05) | \
					 PIN_PUPDR_PULLDOWN(PF06) | \
					 PIN_PUPDR_PULLDOWN(PF07) | \
					 PIN_PUPDR_PULLDOWN(PF08) | \
					 PIN_PUPDR_PULLDOWN(PF09) | \
					 PIN_PUPDR_PULLDOWN(PF10) | \
					 PIN_PUPDR_PULLDOWN(PF11) | \
					 PIN_PUPDR_PULLDOWN(PF12) | \
					 PIN_PUPDR_PULLDOWN(PF13) | \
					 PIN_PUPDR_PULLDOWN(PF14) | \
					 PIN_PUPDR_PULLDOWN(PF15))

#define VAL_GPIOF_ODR                   (PIN_ODR_LEVEL_LOW(PF00) | \
					 PIN_ODR_LEVEL_LOW(PF01) | \
					 PIN_ODR_LEVEL_LOW(PF02) | \
					 PIN_ODR_LEVEL_LOW(PF03) | \
					 PIN_ODR_LEVEL_LOW(PF04) | \
					 PIN_ODR_LEVEL_LOW(PF05) | \
					 PIN_ODR_LEVEL_LOW(PF06) | \
					 PIN_ODR_LEVEL_LOW(PF07) | \
					 PIN_ODR_LEVEL_LOW(PF08) | \
					 PIN_ODR_LEVEL_LOW(PF09) | \
					 PIN_ODR_LEVEL_LOW(PF10) | \
					 PIN_ODR_LEVEL_LOW(PF11) | \
					 PIN_ODR_LEVEL_LOW(PF12) | \
					 PIN_ODR_LEVEL_LOW(PF13) | \
					 PIN_ODR_LEVEL_LOW(PF14) | \
					 PIN_ODR_LEVEL_LOW(PF15))

#define VAL_GPIOF_AFRL			(PIN_AFIO_AF(PF00, 0) | \
					 PIN_AFIO_AF(PF01, 0) | \
					 PIN_AFIO_AF(PF02, 0) | \
					 PIN_AFIO_AF(PF03, 0) | \
					 PIN_AFIO_AF(PF04, 0) | \
					 PIN_AFIO_AF(PF05, 0) | \
					 PIN_AFIO_AF(PF06, 0) | \
					 PIN_AFIO_AF(PF07, 0))

#define VAL_GPIOF_AFRH			(PIN_AFIO_AF(PF08, 0) | \
					 PIN_AFIO_AF(PF09, 0) | \
					 PIN_AFIO_AF(PF10, 0) | \
					 PIN_AFIO_AF(PF11, 0) | \
					 PIN_AFIO_AF(PF12, 0) | \
					 PIN_AFIO_AF(PF13, 0) | \
					 PIN_AFIO_AF(PF14, 0) | \
					 PIN_AFIO_AF(PF15, 0))

#define VAL_GPIOG_MODER                 (PIN_MODE_INPUT(PG00) | \
					 PIN_MODE_INPUT(PG01) | \
					 PIN_MODE_INPUT(PG02) | \
					 PIN_MODE_INPUT(PG03) | \
					 PIN_MODE_INPUT(PG04) | \
					 PIN_MODE_INPUT(PG05) | \
					 PIN_MODE_INPUT(PG06) | \
					 PIN_MODE_INPUT(PG07) | \
					 PIN_MODE_INPUT(PG08) | \
					 PIN_MODE_INPUT(PG09) | \
					 PIN_MODE_INPUT(PG10) | \
					 PIN_MODE_INPUT(PG11) | \
					 PIN_MODE_INPUT(PG12) | \
					 PIN_MODE_INPUT(PG13) | \
					 PIN_MODE_INPUT(PG14) | \
					 PIN_MODE_INPUT(PG15))

#define VAL_GPIOG_OTYPER                (PIN_OTYPE_PUSHPULL(PG00) | \
					 PIN_OTYPE_PUSHPULL(PG01) | \
					 PIN_OTYPE_PUSHPULL(PG02) | \
					 PIN_OTYPE_PUSHPULL(PG03) | \
					 PIN_OTYPE_PUSHPULL(PG04) | \
					 PIN_OTYPE_PUSHPULL(PG05) | \
					 PIN_OTYPE_PUSHPULL(PG06) | \
					 PIN_OTYPE_PUSHPULL(PG07) | \
					 PIN_OTYPE_PUSHPULL(PG08) | \
					 PIN_OTYPE_PUSHPULL(PG09) | \
					 PIN_OTYPE_PUSHPULL(PG10) | \
					 PIN_OTYPE_PUSHPULL(PG11) | \
					 PIN_OTYPE_PUSHPULL(PG12) | \
					 PIN_OTYPE_PUSHPULL(PG13) | \
					 PIN_OTYPE_PUSHPULL(PG14) | \
					 PIN_OTYPE_PUSHPULL(PG15))

#define VAL_GPIOG_OSPEEDR               (PIN_OSPEED_SPEED_VERYLOW(PG00) | \
					 PIN_OSPEED_SPEED_VERYLOW(PG01) | \
					 PIN_OSPEED_SPEED_VERYLOW(PG02) | \
					 PIN_OSPEED_SPEED_VERYLOW(PG03) | \
					 PIN_OSPEED_SPEED_VERYLOW(PG04) | \
					 PIN_OSPEED_SPEED_VERYLOW(PG05) | \
					 PIN_OSPEED_SPEED_VERYLOW(PG06) | \
					 PIN_OSPEED_SPEED_VERYLOW(PG07) | \
					 PIN_OSPEED_SPEED_VERYLOW(PG08) | \
					 PIN_OSPEED_SPEED_VERYLOW(PG09) | \
					 PIN_OSPEED_SPEED_VERYLOW(PG10) | \
					 PIN_OSPEED_SPEED_VERYLOW(PG11) | \
					 PIN_OSPEED_SPEED_VERYLOW(PG12) | \
					 PIN_OSPEED_SPEED_VERYLOW(PG13) | \
					 PIN_OSPEED_SPEED_VERYLOW(PG14) | \
					 PIN_OSPEED_SPEED_VERYLOW(PG15))

#define VAL_GPIOG_PUPDR                 (PIN_PUPDR_PULLDOWN(PG00) | \
					 PIN_PUPDR_PULLDOWN(PG01) | \
					 PIN_PUPDR_PULLDOWN(PG02) | \
					 PIN_PUPDR_PULLDOWN(PG03) | \
					 PIN_PUPDR_PULLDOWN(PG04) | \
					 PIN_PUPDR_PULLDOWN(PG05) | \
					 PIN_PUPDR_PULLDOWN(PG06) | \
					 PIN_PUPDR_PULLDOWN(PG07) | \
					 PIN_PUPDR_PULLDOWN(PG08) | \
					 PIN_PUPDR_PULLDOWN(PG09) | \
					 PIN_PUPDR_PULLDOWN(PG10) | \
					 PIN_PUPDR_PULLDOWN(PG11) | \
					 PIN_PUPDR_PULLDOWN(PG12) | \
					 PIN_PUPDR_PULLDOWN(PG13) | \
					 PIN_PUPDR_PULLDOWN(PG14) | \
					 PIN_PUPDR_PULLDOWN(PG15))

#define VAL_GPIOG_ODR                   (PIN_ODR_LEVEL_LOW(PG00) | \
					 PIN_ODR_LEVEL_LOW(PG01) | \
					 PIN_ODR_LEVEL_LOW(PG02) | \
					 PIN_ODR_LEVEL_LOW(PG03) | \
					 PIN_ODR_LEVEL_LOW(PG04) | \
					 PIN_ODR_LEVEL_LOW(PG05) | \
					 PIN_ODR_LEVEL_LOW(PG06) | \
					 PIN_ODR_LEVEL_LOW(PG07) | \
					 PIN_ODR_LEVEL_LOW(PG08) | \
					 PIN_ODR_LEVEL_LOW(PG09) | \
					 PIN_ODR_LEVEL_LOW(PG10) | \
					 PIN_ODR_LEVEL_LOW(PG11) | \
					 PIN_ODR_LEVEL_LOW(PG12) | \
					 PIN_ODR_LEVEL_LOW(PG13) | \
					 PIN_ODR_LEVEL_LOW(PG14) | \
					 PIN_ODR_LEVEL_LOW(PG15))

#define VAL_GPIOG_AFRL			(PIN_AFIO_AF(PG00, 0) | \
					 PIN_AFIO_AF(PG01, 0) | \
					 PIN_AFIO_AF(PG02, 0) | \
					 PIN_AFIO_AF(PG03, 0) | \
					 PIN_AFIO_AF(PG04, 0) | \
					 PIN_AFIO_AF(PG05, 0) | \
					 PIN_AFIO_AF(PG06, 0) | \
					 PIN_AFIO_AF(PG07, 0))

#define VAL_GPIOG_AFRH			(PIN_AFIO_AF(PG08, 0) | \
					 PIN_AFIO_AF(PG09, 0) | \
					 PIN_AFIO_AF(PG10, 0) | \
					 PIN_AFIO_AF(PG11, 0) | \
					 PIN_AFIO_AF(PG12, 0) | \
					 PIN_AFIO_AF(PG13, 0) | \
					 PIN_AFIO_AF(PG14, 0) | \
					 PIN_AFIO_AF(PG15, 0))

#define VAL_GPIOH_MODER                 (PIN_MODE_ALTERNATE(OSC_IN) | \
					 PIN_MODE_ALTERNATE(OSC_OUT) | \
					 PIN_MODE_INPUT(PH02) | \
					 PIN_MODE_INPUT(PH03) | \
					 PIN_MODE_INPUT(PH04) | \
					 PIN_MODE_INPUT(PH05) | \
					 PIN_MODE_INPUT(PH06) | \
					 PIN_MODE_INPUT(PH07) | \
					 PIN_MODE_INPUT(PH08) | \
					 PIN_MODE_INPUT(PH09) | \
					 PIN_MODE_INPUT(PH10) | \
					 PIN_MODE_INPUT(PH11) | \
					 PIN_MODE_INPUT(PH12) | \
					 PIN_MODE_INPUT(PH13) | \
					 PIN_MODE_INPUT(PH14) | \
					 PIN_MODE_INPUT(PH15))

#define VAL_GPIOH_OTYPER                (PIN_OTYPE_PUSHPULL(OSC_IN) | \
					 PIN_OTYPE_PUSHPULL(OSC_OUT) | \
					 PIN_OTYPE_PUSHPULL(PH02) | \
					 PIN_OTYPE_PUSHPULL(PH03) | \
					 PIN_OTYPE_PUSHPULL(PH04) | \
					 PIN_OTYPE_PUSHPULL(PH05) | \
					 PIN_OTYPE_PUSHPULL(PH06) | \
					 PIN_OTYPE_PUSHPULL(PH07) | \
					 PIN_OTYPE_PUSHPULL(PH08) | \
					 PIN_OTYPE_PUSHPULL(PH09) | \
					 PIN_OTYPE_PUSHPULL(PH10) | \
					 PIN_OTYPE_PUSHPULL(PH11) | \
					 PIN_OTYPE_PUSHPULL(PH12) | \
					 PIN_OTYPE_PUSHPULL(PH13) | \
					 PIN_OTYPE_PUSHPULL(PH14) | \
					 PIN_OTYPE_PUSHPULL(PH15))

#define VAL_GPIOH_OSPEEDR               (PIN_OSPEED_SPEED_HIGH(OSC_IN) | \
					 PIN_OSPEED_SPEED_HIGH(OSC_OUT) | \
					 PIN_OSPEED_SPEED_VERYLOW(PH02) | \
					 PIN_OSPEED_SPEED_VERYLOW(PH03) | \
					 PIN_OSPEED_SPEED_VERYLOW(PH04) | \
					 PIN_OSPEED_SPEED_VERYLOW(PH05) | \
					 PIN_OSPEED_SPEED_VERYLOW(PH06) | \
					 PIN_OSPEED_SPEED_VERYLOW(PH07) | \
					 PIN_OSPEED_SPEED_VERYLOW(PH08) | \
					 PIN_OSPEED_SPEED_VERYLOW(PH09) | \
					 PIN_OSPEED_SPEED_VERYLOW(PH10) | \
					 PIN_OSPEED_SPEED_VERYLOW(PH11) | \
					 PIN_OSPEED_SPEED_VERYLOW(PH12) | \
					 PIN_OSPEED_SPEED_VERYLOW(PH13) | \
					 PIN_OSPEED_SPEED_VERYLOW(PH14) | \
					 PIN_OSPEED_SPEED_VERYLOW(PH15))

#define VAL_GPIOH_PUPDR                 (PIN_PUPDR_FLOATING(OSC_IN) | \
					 PIN_PUPDR_FLOATING(OSC_OUT) | \
					 PIN_PUPDR_PULLDOWN(PH02) | \
					 PIN_PUPDR_PULLDOWN(PH03) | \
					 PIN_PUPDR_PULLDOWN(PH04) | \
					 PIN_PUPDR_PULLDOWN(PH05) | \
					 PIN_PUPDR_PULLDOWN(PH06) | \
					 PIN_PUPDR_PULLDOWN(PH07) | \
					 PIN_PUPDR_PULLDOWN(PH08) | \
					 PIN_PUPDR_PULLDOWN(PH09) | \
					 PIN_PUPDR_PULLDOWN(PH10) | \
					 PIN_PUPDR_PULLDOWN(PH11) | \
					 PIN_PUPDR_PULLDOWN(PH12) | \
					 PIN_PUPDR_PULLDOWN(PH13) | \
					 PIN_PUPDR_PULLDOWN(PH14) | \
					 PIN_PUPDR_PULLDOWN(PH15))

#define VAL_GPIOH_ODR                   (PIN_ODR_LEVEL_HIGH(OSC_IN) | \
					 PIN_ODR_LEVEL_HIGH(OSC_OUT) | \
					 PIN_ODR_LEVEL_LOW(PH02) | \
					 PIN_ODR_LEVEL_LOW(PH03) | \
					 PIN_ODR_LEVEL_LOW(PH04) | \
					 PIN_ODR_LEVEL_LOW(PH05) | \
					 PIN_ODR_LEVEL_LOW(PH06) | \
					 PIN_ODR_LEVEL_LOW(PH07) | \
					 PIN_ODR_LEVEL_LOW(PH08) | \
					 PIN_ODR_LEVEL_LOW(PH09) | \
					 PIN_ODR_LEVEL_LOW(PH10) | \
					 PIN_ODR_LEVEL_LOW(PH11) | \
					 PIN_ODR_LEVEL_LOW(PH12) | \
					 PIN_ODR_LEVEL_LOW(PH13) | \
					 PIN_ODR_LEVEL_LOW(PH14) | \
					 PIN_ODR_LEVEL_LOW(PH15))

#define VAL_GPIOH_AFRL			(PIN_AFIO_AF(OSC_IN, 0) | \
					 PIN_AFIO_AF(OSC_OUT, 0) | \
					 PIN_AFIO_AF(PH02, 0) | \
					 PIN_AFIO_AF(PH03, 0) | \
					 PIN_AFIO_AF(PH04, 0) | \
					 PIN_AFIO_AF(PH05, 0) | \
					 PIN_AFIO_AF(PH06, 0) | \
					 PIN_AFIO_AF(PH07, 0))

#define VAL_GPIOH_AFRH			(PIN_AFIO_AF(PH08, 0) | \
					 PIN_AFIO_AF(PH09, 0) | \
					 PIN_AFIO_AF(PH10, 0) | \
					 PIN_AFIO_AF(PH11, 0) | \
					 PIN_AFIO_AF(PH12, 0) | \
					 PIN_AFIO_AF(PH13, 0) | \
					 PIN_AFIO_AF(PH14, 0) | \
					 PIN_AFIO_AF(PH15, 0))

#define VAL_GPIOI_MODER                 (PIN_MODE_INPUT(PI00) | \
					 PIN_MODE_INPUT(PI01) | \
					 PIN_MODE_INPUT(PI02) | \
					 PIN_MODE_INPUT(PI03) | \
					 PIN_MODE_INPUT(PI04) | \
					 PIN_MODE_INPUT(PI05) | \
					 PIN_MODE_INPUT(PI06) | \
					 PIN_MODE_INPUT(PI07) | \
					 PIN_MODE_INPUT(PI08) | \
					 PIN_MODE_INPUT(PI09) | \
					 PIN_MODE_INPUT(PI10) | \
					 PIN_MODE_INPUT(PI11) | \
					 PIN_MODE_INPUT(PI12) | \
					 PIN_MODE_INPUT(PI13) | \
					 PIN_MODE_INPUT(PI14) | \
					 PIN_MODE_INPUT(PI15))

#define VAL_GPIOI_OTYPER                (PIN_OTYPE_PUSHPULL(PI00) | \
					 PIN_OTYPE_PUSHPULL(PI01) | \
					 PIN_OTYPE_PUSHPULL(PI02) | \
					 PIN_OTYPE_PUSHPULL(PI03) | \
					 PIN_OTYPE_PUSHPULL(PI04) | \
					 PIN_OTYPE_PUSHPULL(PI05) | \
					 PIN_OTYPE_PUSHPULL(PI06) | \
					 PIN_OTYPE_PUSHPULL(PI07) | \
					 PIN_OTYPE_PUSHPULL(PI08) | \
					 PIN_OTYPE_PUSHPULL(PI09) | \
					 PIN_OTYPE_PUSHPULL(PI10) | \
					 PIN_OTYPE_PUSHPULL(PI11) | \
					 PIN_OTYPE_PUSHPULL(PI12) | \
					 PIN_OTYPE_PUSHPULL(PI13) | \
					 PIN_OTYPE_PUSHPULL(PI14) | \
					 PIN_OTYPE_PUSHPULL(PI15))

#define VAL_GPIOI_OSPEEDR               (PIN_OSPEED_SPEED_VERYLOW(PI00) | \
					 PIN_OSPEED_SPEED_VERYLOW(PI01) | \
					 PIN_OSPEED_SPEED_VERYLOW(PI02) | \
					 PIN_OSPEED_SPEED_VERYLOW(PI03) | \
					 PIN_OSPEED_SPEED_VERYLOW(PI04) | \
					 PIN_OSPEED_SPEED_VERYLOW(PI05) | \
					 PIN_OSPEED_SPEED_VERYLOW(PI06) | \
					 PIN_OSPEED_SPEED_VERYLOW(PI07) | \
					 PIN_OSPEED_SPEED_VERYLOW(PI08) | \
					 PIN_OSPEED_SPEED_VERYLOW(PI09) | \
					 PIN_OSPEED_SPEED_VERYLOW(PI10) | \
					 PIN_OSPEED_SPEED_VERYLOW(PI11) | \
					 PIN_OSPEED_SPEED_VERYLOW(PI12) | \
					 PIN_OSPEED_SPEED_VERYLOW(PI13) | \
					 PIN_OSPEED_SPEED_VERYLOW(PI14) | \
					 PIN_OSPEED_SPEED_VERYLOW(PI15))

#define VAL_GPIOI_PUPDR                 (PIN_PUPDR_PULLDOWN(PI00) | \
					 PIN_PUPDR_PULLDOWN(PI01) | \
					 PIN_PUPDR_PULLDOWN(PI02) | \
					 PIN_PUPDR_PULLDOWN(PI03) | \
					 PIN_PUPDR_PULLDOWN(PI04) | \
					 PIN_PUPDR_PULLDOWN(PI05) | \
					 PIN_PUPDR_PULLDOWN(PI06) | \
					 PIN_PUPDR_PULLDOWN(PI07) | \
					 PIN_PUPDR_PULLDOWN(PI08) | \
					 PIN_PUPDR_PULLDOWN(PI09) | \
					 PIN_PUPDR_PULLDOWN(PI10) | \
					 PIN_PUPDR_PULLDOWN(PI11) | \
					 PIN_PUPDR_PULLDOWN(PI12) | \
					 PIN_PUPDR_PULLDOWN(PI13) | \
					 PIN_PUPDR_PULLDOWN(PI14) | \
					 PIN_PUPDR_PULLDOWN(PI15))

#define VAL_GPIOI_ODR                   (PIN_ODR_LEVEL_LOW(PI00) | \
					 PIN_ODR_LEVEL_LOW(PI01) | \
					 PIN_ODR_LEVEL_LOW(PI02) | \
					 PIN_ODR_LEVEL_LOW(PI03) | \
					 PIN_ODR_LEVEL_LOW(PI04) | \
					 PIN_ODR_LEVEL_LOW(PI05) | \
					 PIN_ODR_LEVEL_LOW(PI06) | \
					 PIN_ODR_LEVEL_LOW(PI07) | \
					 PIN_ODR_LEVEL_LOW(PI08) | \
					 PIN_ODR_LEVEL_LOW(PI09) | \
					 PIN_ODR_LEVEL_LOW(PI10) | \
					 PIN_ODR_LEVEL_LOW(PI11) | \
					 PIN_ODR_LEVEL_LOW(PI12) | \
					 PIN_ODR_LEVEL_LOW(PI13) | \
					 PIN_ODR_LEVEL_LOW(PI14) | \
					 PIN_ODR_LEVEL_LOW(PI15))

#define VAL_GPIOI_AFRL			(PIN_AFIO_AF(PI00, 0) | \
					 PIN_AFIO_AF(PI01, 0) | \
					 PIN_AFIO_AF(PI02, 0) | \
					 PIN_AFIO_AF(PI03, 0) | \
					 PIN_AFIO_AF(PI04, 0) | \
					 PIN_AFIO_AF(PI05, 0) | \
					 PIN_AFIO_AF(PI06, 0) | \
					 PIN_AFIO_AF(PI07, 0))

#define VAL_GPIOI_AFRH			(PIN_AFIO_AF(PI08, 0) | \
					 PIN_AFIO_AF(PI09, 0) | \
					 PIN_AFIO_AF(PI10, 0) | \
					 PIN_AFIO_AF(PI11, 0) | \
					 PIN_AFIO_AF(PI12, 0) | \
					 PIN_AFIO_AF(PI13, 0) | \
					 PIN_AFIO_AF(PI14, 0) | \
					 PIN_AFIO_AF(PI15, 0))

#define VAL_GPIOJ_MODER                 (PIN_MODE_INPUT(PJ00) | \
					 PIN_MODE_INPUT(PJ01) | \
					 PIN_MODE_INPUT(PJ02) | \
					 PIN_MODE_INPUT(PJ03) | \
					 PIN_MODE_INPUT(PJ04) | \
					 PIN_MODE_INPUT(PJ05) | \
					 PIN_MODE_INPUT(PJ06) | \
					 PIN_MODE_INPUT(PJ07) | \
					 PIN_MODE_INPUT(PJ08) | \
					 PIN_MODE_INPUT(PJ09) | \
					 PIN_MODE_INPUT(PJ10) | \
					 PIN_MODE_INPUT(PJ11) | \
					 PIN_MODE_INPUT(PJ12) | \
					 PIN_MODE_INPUT(PJ13) | \
					 PIN_MODE_INPUT(PJ14) | \
					 PIN_MODE_INPUT(PJ15))

#define VAL_GPIOJ_OTYPER                (PIN_OTYPE_PUSHPULL(PJ00) | \
					 PIN_OTYPE_PUSHPULL(PJ01) | \
					 PIN_OTYPE_PUSHPULL(PJ02) | \
					 PIN_OTYPE_PUSHPULL(PJ03) | \
					 PIN_OTYPE_PUSHPULL(PJ04) | \
					 PIN_OTYPE_PUSHPULL(PJ05) | \
					 PIN_OTYPE_PUSHPULL(PJ06) | \
					 PIN_OTYPE_PUSHPULL(PJ07) | \
					 PIN_OTYPE_PUSHPULL(PJ08) | \
					 PIN_OTYPE_PUSHPULL(PJ09) | \
					 PIN_OTYPE_PUSHPULL(PJ10) | \
					 PIN_OTYPE_PUSHPULL(PJ11) | \
					 PIN_OTYPE_PUSHPULL(PJ12) | \
					 PIN_OTYPE_PUSHPULL(PJ13) | \
					 PIN_OTYPE_PUSHPULL(PJ14) | \
					 PIN_OTYPE_PUSHPULL(PJ15))

#define VAL_GPIOJ_OSPEEDR               (PIN_OSPEED_SPEED_VERYLOW(PJ00) | \
					 PIN_OSPEED_SPEED_VERYLOW(PJ01) | \
					 PIN_OSPEED_SPEED_VERYLOW(PJ02) | \
					 PIN_OSPEED_SPEED_VERYLOW(PJ03) | \
					 PIN_OSPEED_SPEED_VERYLOW(PJ04) | \
					 PIN_OSPEED_SPEED_VERYLOW(PJ05) | \
					 PIN_OSPEED_SPEED_VERYLOW(PJ06) | \
					 PIN_OSPEED_SPEED_VERYLOW(PJ07) | \
					 PIN_OSPEED_SPEED_VERYLOW(PJ08) | \
					 PIN_OSPEED_SPEED_VERYLOW(PJ09) | \
					 PIN_OSPEED_SPEED_VERYLOW(PJ10) | \
					 PIN_OSPEED_SPEED_VERYLOW(PJ11) | \
					 PIN_OSPEED_SPEED_VERYLOW(PJ12) | \
					 PIN_OSPEED_SPEED_VERYLOW(PJ13) | \
					 PIN_OSPEED_SPEED_VERYLOW(PJ14) | \
					 PIN_OSPEED_SPEED_VERYLOW(PJ15))

#define VAL_GPIOJ_PUPDR                 (PIN_PUPDR_PULLDOWN(PJ00) | \
					 PIN_PUPDR_PULLDOWN(PJ01) | \
					 PIN_PUPDR_PULLDOWN(PJ02) | \
					 PIN_PUPDR_PULLDOWN(PJ03) | \
					 PIN_PUPDR_PULLDOWN(PJ04) | \
					 PIN_PUPDR_PULLDOWN(PJ05) | \
					 PIN_PUPDR_PULLDOWN(PJ06) | \
					 PIN_PUPDR_PULLDOWN(PJ07) | \
					 PIN_PUPDR_PULLDOWN(PJ08) | \
					 PIN_PUPDR_PULLDOWN(PJ09) | \
					 PIN_PUPDR_PULLDOWN(PJ10) | \
					 PIN_PUPDR_PULLDOWN(PJ11) | \
					 PIN_PUPDR_PULLDOWN(PJ12) | \
					 PIN_PUPDR_PULLDOWN(PJ13) | \
					 PIN_PUPDR_PULLDOWN(PJ14) | \
					 PIN_PUPDR_PULLDOWN(PJ15))

#define VAL_GPIOJ_ODR                   (PIN_ODR_LEVEL_LOW(PJ00) | \
					 PIN_ODR_LEVEL_LOW(PJ01) | \
					 PIN_ODR_LEVEL_LOW(PJ02) | \
					 PIN_ODR_LEVEL_LOW(PJ03) | \
					 PIN_ODR_LEVEL_LOW(PJ04) | \
					 PIN_ODR_LEVEL_LOW(PJ05) | \
					 PIN_ODR_LEVEL_LOW(PJ06) | \
					 PIN_ODR_LEVEL_LOW(PJ07) | \
					 PIN_ODR_LEVEL_LOW(PJ08) | \
					 PIN_ODR_LEVEL_LOW(PJ09) | \
					 PIN_ODR_LEVEL_LOW(PJ10) | \
					 PIN_ODR_LEVEL_LOW(PJ11) | \
					 PIN_ODR_LEVEL_LOW(PJ12) | \
					 PIN_ODR_LEVEL_LOW(PJ13) | \
					 PIN_ODR_LEVEL_LOW(PJ14) | \
					 PIN_ODR_LEVEL_LOW(PJ15))

#define VAL_GPIOJ_AFRL			(PIN_AFIO_AF(PJ00, 0) | \
					 PIN_AFIO_AF(PJ01, 0) | \
					 PIN_AFIO_AF(PJ02, 0) | \
					 PIN_AFIO_AF(PJ03, 0) | \
					 PIN_AFIO_AF(PJ04, 0) | \
					 PIN_AFIO_AF(PJ05, 0) | \
					 PIN_AFIO_AF(PJ06, 0) | \
					 PIN_AFIO_AF(PJ07, 0))

#define VAL_GPIOJ_AFRH			(PIN_AFIO_AF(PJ08, 0) | \
					 PIN_AFIO_AF(PJ09, 0) | \
					 PIN_AFIO_AF(PJ10, 0) | \
					 PIN_AFIO_AF(PJ11, 0) | \
					 PIN_AFIO_AF(PJ12, 0) | \
					 PIN_AFIO_AF(PJ13, 0) | \
					 PIN_AFIO_AF(PJ14, 0) | \
					 PIN_AFIO_AF(PJ15, 0))

#define VAL_GPIOK_MODER                 (PIN_MODE_INPUT(PK00) | \
					 PIN_MODE_INPUT(PK01) | \
					 PIN_MODE_INPUT(PK02) | \
					 PIN_MODE_INPUT(PK03) | \
					 PIN_MODE_INPUT(PK04) | \
					 PIN_MODE_INPUT(PK05) | \
					 PIN_MODE_INPUT(PK06) | \
					 PIN_MODE_INPUT(PK07) | \
					 PIN_MODE_INPUT(PK08) | \
					 PIN_MODE_INPUT(PK09) | \
					 PIN_MODE_INPUT(PK10) | \
					 PIN_MODE_INPUT(PK11) | \
					 PIN_MODE_INPUT(PK12) | \
					 PIN_MODE_INPUT(PK13) | \
					 PIN_MODE_INPUT(PK14) | \
					 PIN_MODE_INPUT(PK15))

#define VAL_GPIOK_OTYPER                (PIN_OTYPE_PUSHPULL(PK00) | \
					 PIN_OTYPE_PUSHPULL(PK01) | \
					 PIN_OTYPE_PUSHPULL(PK02) | \
					 PIN_OTYPE_PUSHPULL(PK03) | \
					 PIN_OTYPE_PUSHPULL(PK04) | \
					 PIN_OTYPE_PUSHPULL(PK05) | \
					 PIN_OTYPE_PUSHPULL(PK06) | \
					 PIN_OTYPE_PUSHPULL(PK07) | \
					 PIN_OTYPE_PUSHPULL(PK08) | \
					 PIN_OTYPE_PUSHPULL(PK09) | \
					 PIN_OTYPE_PUSHPULL(PK10) | \
					 PIN_OTYPE_PUSHPULL(PK11) | \
					 PIN_OTYPE_PUSHPULL(PK12) | \
					 PIN_OTYPE_PUSHPULL(PK13) | \
					 PIN_OTYPE_PUSHPULL(PK14) | \
					 PIN_OTYPE_PUSHPULL(PK15))

#define VAL_GPIOK_OSPEEDR               (PIN_OSPEED_SPEED_VERYLOW(PK00) | \
					 PIN_OSPEED_SPEED_VERYLOW(PK01) | \
					 PIN_OSPEED_SPEED_VERYLOW(PK02) | \
					 PIN_OSPEED_SPEED_VERYLOW(PK03) | \
					 PIN_OSPEED_SPEED_VERYLOW(PK04) | \
					 PIN_OSPEED_SPEED_VERYLOW(PK05) | \
					 PIN_OSPEED_SPEED_VERYLOW(PK06) | \
					 PIN_OSPEED_SPEED_VERYLOW(PK07) | \
					 PIN_OSPEED_SPEED_VERYLOW(PK08) | \
					 PIN_OSPEED_SPEED_VERYLOW(PK09) | \
					 PIN_OSPEED_SPEED_VERYLOW(PK10) | \
					 PIN_OSPEED_SPEED_VERYLOW(PK11) | \
					 PIN_OSPEED_SPEED_VERYLOW(PK12) | \
					 PIN_OSPEED_SPEED_VERYLOW(PK13) | \
					 PIN_OSPEED_SPEED_VERYLOW(PK14) | \
					 PIN_OSPEED_SPEED_VERYLOW(PK15))

#define VAL_GPIOK_PUPDR                 (PIN_PUPDR_PULLDOWN(PK00) | \
					 PIN_PUPDR_PULLDOWN(PK01) | \
					 PIN_PUPDR_PULLDOWN(PK02) | \
					 PIN_PUPDR_PULLDOWN(PK03) | \
					 PIN_PUPDR_PULLDOWN(PK04) | \
					 PIN_PUPDR_PULLDOWN(PK05) | \
					 PIN_PUPDR_PULLDOWN(PK06) | \
					 PIN_PUPDR_PULLDOWN(PK07) | \
					 PIN_PUPDR_PULLDOWN(PK08) | \
					 PIN_PUPDR_PULLDOWN(PK09) | \
					 PIN_PUPDR_PULLDOWN(PK10) | \
					 PIN_PUPDR_PULLDOWN(PK11) | \
					 PIN_PUPDR_PULLDOWN(PK12) | \
					 PIN_PUPDR_PULLDOWN(PK13) | \
					 PIN_PUPDR_PULLDOWN(PK14) | \
					 PIN_PUPDR_PULLDOWN(PK15))

#define VAL_GPIOK_ODR                   (PIN_ODR_LEVEL_LOW(PK00) | \
					 PIN_ODR_LEVEL_LOW(PK01) | \
					 PIN_ODR_LEVEL_LOW(PK02) | \
					 PIN_ODR_LEVEL_LOW(PK03) | \
					 PIN_ODR_LEVEL_LOW(PK04) | \
					 PIN_ODR_LEVEL_LOW(PK05) | \
					 PIN_ODR_LEVEL_LOW(PK06) | \
					 PIN_ODR_LEVEL_LOW(PK07) | \
					 PIN_ODR_LEVEL_LOW(PK08) | \
					 PIN_ODR_LEVEL_LOW(PK09) | \
					 PIN_ODR_LEVEL_LOW(PK10) | \
					 PIN_ODR_LEVEL_LOW(PK11) | \
					 PIN_ODR_LEVEL_LOW(PK12) | \
					 PIN_ODR_LEVEL_LOW(PK13) | \
					 PIN_ODR_LEVEL_LOW(PK14) | \
					 PIN_ODR_LEVEL_LOW(PK15))

#define VAL_GPIOK_AFRL			(PIN_AFIO_AF(PK00, 0) | \
					 PIN_AFIO_AF(PK01, 0) | \
					 PIN_AFIO_AF(PK02, 0) | \
					 PIN_AFIO_AF(PK03, 0) | \
					 PIN_AFIO_AF(PK04, 0) | \
					 PIN_AFIO_AF(PK05, 0) | \
					 PIN_AFIO_AF(PK06, 0) | \
					 PIN_AFIO_AF(PK07, 0))

#define VAL_GPIOK_AFRH			(PIN_AFIO_AF(PK08, 0) | \
					 PIN_AFIO_AF(PK09, 0) | \
					 PIN_AFIO_AF(PK10, 0) | \
					 PIN_AFIO_AF(PK11, 0) | \
					 PIN_AFIO_AF(PK12, 0) | \
					 PIN_AFIO_AF(PK13, 0) | \
					 PIN_AFIO_AF(PK14, 0) | \
					 PIN_AFIO_AF(PK15, 0))

#define AF_UART4_TX                      8U
#define AF_LINE_UART4_TX                 8U
#define AF_UART4_RX                      8U
#define AF_LINE_UART4_RX                 8U
#define AF_SPI1_SCK                      5U
#define AF_LINE_SPI1_SCK                 5U
#define AF_SWDIO                         0U
#define AF_LINE_SWDIO                    0U
#define AF_SWCLK                         0U
#define AF_LINE_SWCLK                    0U
#define AF_SWO                           0U
#define AF_LINE_SWO                      0U
#define AF_SPI1_MISO                     5U
#define AF_LINE_SPI1_MISO                5U
#define AF_SPI1_MOSI                     5U
#define AF_LINE_SPI1_MOSI                5U
#define AF_USART1_TX                     7U
#define AF_LINE_USART1_TX                7U
#define AF_USART1_RX                     7U
#define AF_LINE_USART1_RX                7U
#define AF_I2C1_SCL                      4U
#define AF_LINE_I2C1_SCL                 4U
#define AF_I2C1_SDA                      4U
#define AF_LINE_I2C1_SDA                 4U
#define AF_SPI2_MOSI                     5U
#define AF_LINE_SPI2_MOSI                5U
#define AF_SPI2_MISO                     5U
#define AF_LINE_SPI2_MISO                5U
#define AF_SDMMC1_D0                     12U
#define AF_LINE_SDMMC1_D0                12U
#define AF_SDMMC1_D1                     12U
#define AF_LINE_SDMMC1_D1                12U
#define AF_SDMMC1_D2                     12U
#define AF_LINE_SDMMC1_D2                12U
#define AF_SDMMC1_D3                     12U
#define AF_LINE_SDMMC1_D3                12U
#define AF_SDMMC1_CK                     12U
#define AF_LINE_SDMMC1_CK                12U
#define AF_SDMMC1_CMD                    12U
#define AF_LINE_SDMMC1_CMD               12U
#define AF_SPI2_SCK                      5U
#define AF_LINE_SPI2_SCK                 5U
#define AF_UART8_RX                      8U
#define AF_LINE_UART8_RX                 8U
#define AF_UART8_TX                      8U
#define AF_LINE_UART8_TX                 8U
#define AF_IMU_SPI_MISO                  5U
#define AF_LINE_IMU_SPI_MISO             5U
#define AF_IMU_SPI_MOSI                  5U
#define AF_LINE_IMU_SPI_MOSI             5U
#define AF_GPS_RX                        8U
#define AF_LINE_GPS_RX                   8U
#define AF_GPS_TX                        8U
#define AF_LINE_GPS_TX                   8U
#define AF_IMU_SPI_SCK                   5U
#define AF_LINE_IMU_SPI_SCK              5U
#define AF_OSC_IN                        0U
#define AF_LINE_OSC_IN                   0U
#define AF_OSC_OUT                       0U
#define AF_LINE_OSC_OUT                  0U


#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
  void boardInit(void);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

