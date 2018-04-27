#ifndef CONFIG_LISA_S_2_00_H
#define CONFIG_LISA_S_2_00_H

#define BOARD_LISA_S_2_0

/**
 * ChibiOS board file
 */
#include "boards/lisa_s/chibios/v2.0/board.h"

/**
 * PPRZ definitions
 */

/*
 * AHB_CLK
 */
#define AHB_CLK STM32_HCLK


/*
 * LEDs
 */
/* red, on PC13, 0 on LED_ON, 1 on LED_OFF */
#ifndef USE_LED_1
#define USE_LED_1 1
#endif
#define LED_1_GPIO GPIOC
#define LED_1_GPIO_PIN GPIO13
#define LED_1_GPIO_ON gpio_clear
#define LED_1_GPIO_OFF gpio_set

/* green, on PC14, 0 on LED_ON, 1 on LED_OFF */
#ifndef USE_LED_2
#define USE_LED_2 1
#endif
#define LED_2_GPIO GPIOC
#define LED_2_GPIO_PIN GPIO14
#define LED_2_GPIO_ON gpio_clear
#define LED_2_GPIO_OFF gpio_set

/* blue, on PC15, 0 on LED_ON, 1 on LED_OFF */
#ifndef USE_LED_3
#define USE_LED_3 1
#endif
#define LED_3_GPIO GPIOC
#define LED_3_GPIO_PIN GPIO15
#define LED_3_GPIO_ON gpio_clear
#define LED_3_GPIO_OFF gpio_set

/* yellow, on PD11, 1 on LED_ON, 0 on LED_OFF */
#ifndef USE_LED_4
#define USE_LED_4 0
#endif
#define LED_4_GPIO GPIOD
#define LED_4_GPIO_PIN GPIO11
#define LED_4_GPIO_ON gpio_set
#define LED_4_GPIO_OFF gpio_clear

/* AUX0, on PA5, 1 on LED_ON, 0 on LED_OFF */
#ifndef USE_LED_5
#define USE_LED_5 0
#endif
#define LED_5_GPIO GPIOA
#define LED_5_GPIO_PIN GPIO5
#define LED_5_GPIO_ON gpio_set
#define LED_5_GPIO_OFF gpio_clear

/* AUX1, on PA3, 1 on LED_ON, 0 on LED_OFF */
#ifndef USE_LED_6
#define USE_LED_6 0
#endif
#define LED_6_GPIO GPIOA
#define LED_6_GPIO_PIN GPIO3
#define LED_6_GPIO_ON gpio_set
#define LED_6_GPIO_OFF gpio_clear

/* AUX2, on PA2, 1 on LED_ON, 0 on LED_OFF */
#ifndef USE_LED_7
#define USE_LED_7 0
#endif
#define LED_7_GPIO GPIOA
#define LED_7_GPIO_PIN GPIO2
#define LED_7_GPIO_ON gpio_set
#define LED_7_GPIO_OFF gpio_clear

/* AUX3, on PA0, 1 on LED_ON, 0 on LED_OFF */
#ifndef USE_LED_8
#define USE_LED_8 0
#endif
#define LED_8_GPIO GPIOA
#define LED_8_GPIO_PIN GPIO0
#define LED_8_GPIO_ON gpio_set
#define LED_8_GPIO_OFF gpio_clear

/* AUX4, on PC3, 1 on LED_ON, 0 on LED_OFF */
#ifndef USE_LED_9
#define USE_LED_9 0
#endif
#define LED_9_GPIO GPIOC
#define LED_9_GPIO_PIN GPIO3
#define LED_9_GPIO_ON gpio_set
#define LED_9_GPIO_OFF gpio_clear

/* AUX5, on PC2, 1 on LED_ON, 0 on LED_OFF */
#ifndef USE_LED_10
#define USE_LED_10 0
#endif
#define LED_10_GPIO GPIOC
#define LED_10_GPIO_PIN GPIO2
#define LED_10_GPIO_ON gpio_set
#define LED_10_GPIO_OFF gpio_clear

/* AUX6, on PC6, 1 on LED_ON, 0 on LED_OFF */
#ifndef USE_LED_11
#define USE_LED_11 0
#endif
#define LED_11_GPIO GPIOC
#define LED_11_GPIO_PIN GPIO6
#define LED_11_GPIO_ON gpio_set
#define LED_11_GPIO_OFF gpio_clear

/* AUX7, on PC7, 1 on LED_ON, 0 on LED_OFF */
#ifndef USE_LED_12
#define USE_LED_12 0
#endif
#define LED_12_GPIO GPIOC
#define LED_12_GPIO_PIN GPIO7
#define LED_12_GPIO_ON gpio_set
#define LED_12_GPIO_OFF gpio_clear

/*
 * ADCs
 */

// Internal ADC for battery enabled by default
#ifndef USE_ADC_1
#define USE_ADC_1 1
#endif
#if USE_ADC_1
#define AD1_1_CHANNEL ADC_CHANNEL_IN2
#define ADC_1 AD1_1
#define ADC_1_GPIO_PORT GPIOA
#define ADC_1_GPIO_PIN GPIO2
#endif

// ANALOG_IN0
#if USE_ADC_2
#define AD1_2_CHANNEL ADC_CHANNEL_IN3
#define ADC_2 AD1_2
#define ADC_2_GPIO_PORT GPIOA
#define ADC_2_GPIO_PIN GPIO3
#endif

// ANALOG_IN1
#if USE_ADC_3
#define AD1_3_CHANNEL ADC_CHANNEL_IN15
#define ADC_3 AD1_3
#define ADC_3_GPIO_PORT GPIOC
#define ADC_3_GPIO_PIN GPIO5
#endif


/* allow to define ADC_CHANNEL_VSUPPLY in the airframe file*/
#ifndef ADC_CHANNEL_VSUPPLY
#define ADC_CHANNEL_VSUPPLY ADC_1
#endif

/*
 * R1 = 10k
 * R2 = 2k
 * adc * (3.3 / 2^12) * ((R1 + R2) / R1)
 */
#define VBAT_R1 10000.0f
#define VBAT_R2 2000.0f
#define DefaultVoltageOfAdc(adc) ((3.3f/4096.0f)*((VBAT_R1+VBAT_R2)/VBAT_R1)*adc)

//TODO configure DAC (ADC_1)

/*
 * PWM defines
 */
#ifndef USE_PWM0
#define USE_PWM0 1
#endif
#if USE_PWM0
#define PWM_SERVO_0 0
#define PWM_SERVO_0_GPIO GPIOA
#define PWM_SERVO_0_PIN GPIO11
#define PWM_SERVO_0_AF GPIO_AF2
#define PWM_SERVO_0_DRIVER PWMD1
#define PWM_SERVO_0_CHANNEL 3
#define PWM_SERVO_0_ACTIVE PWM_OUTPUT_ACTIVE_HIGH
#else
#define PWM_SERVO_0_ACTIVE PWM_OUTPUT_DISABLED
#endif

#ifndef USE_PWM1
#define USE_PWM1 1
#endif
#if USE_PWM1
#define PWM_SERVO_1 1
#define PWM_SERVO_1_GPIO GPIOA
#define PWM_SERVO_1_PIN GPIO9
#define PWM_SERVO_1_AF GPIO_AF2
#define PWM_SERVO_1_DRIVER PWMD1
#define PWM_SERVO_1_CHANNEL 1
#define PWM_SERVO_1_ACTIVE PWM_OUTPUT_ACTIVE_HIGH
#else
#define PWM_SERVO_1_ACTIVE PWM_OUTPUT_DISABLED
#endif

#ifndef USE_PWM2
#define USE_PWM2 1
#endif
#if USE_PWM2
#define PWM_SERVO_2 2
#define PWM_SERVO_2_GPIO GPIOA
#define PWM_SERVO_2_PIN GPIO10
#define PWM_SERVO_2_AF GPIO_AF2
#define PWM_SERVO_2_DRIVER PWMD1
#define PWM_SERVO_2_CHANNEL 2
#define PWM_SERVO_2_ACTIVE PWM_OUTPUT_ACTIVE_HIGH
#else
#define PWM_SERVO_2_ACTIVE PWM_OUTPUT_DISABLED
#endif

#ifndef USE_PWM3
#define USE_PWM3 1
#endif
#if USE_PWM3
#define PWM_SERVO_3 3
#define PWM_SERVO_3_GPIO GPIOA
#define PWM_SERVO_3_PIN GPIO8
#define PWM_SERVO_3_AF GPIO_AF2
#define PWM_SERVO_3_DRIVER PWMD1
#define PWM_SERVO_3_CHANNEL 0
#define PWM_SERVO_3_ACTIVE PWM_OUTPUT_ACTIVE_HIGH
#else
#define PWM_SERVO_3_ACTIVE PWM_OUTPUT_DISABLED
#endif

#ifndef USE_PWM4
#define USE_PWM4 1
#endif
#if USE_PWM4
#define PWM_SERVO_4 4
#define PWM_SERVO_4_GPIO GPIOD
#define PWM_SERVO_4_PIN GPIO14
#define PWM_SERVO_4_AF GPIO_AF2
#define PWM_SERVO_4_DRIVER PWMD4
#define PWM_SERVO_4_CHANNEL 2
#define PWM_SERVO_4_ACTIVE PWM_OUTPUT_ACTIVE_HIGH
#else
#define PWM_SERVO_4_ACTIVE PWM_OUTPUT_DISABLED
#endif

#ifndef USE_PWM5
#define USE_PWM5 1
#endif
#if USE_PWM5
#define PWM_SERVO_5 5
#define PWM_SERVO_5_GPIO GPIOD
#define PWM_SERVO_5_PIN GPIO15
#define PWM_SERVO_5_AF GPIO_AF2
#define PWM_SERVO_5_DRIVER PWMD4
#define PWM_SERVO_5_CHANNEL 3
#define PWM_SERVO_5_ACTIVE PWM_OUTPUT_ACTIVE_HIGH
#else
#define PWM_SERVO_5_ACTIVE PWM_OUTPUT_DISABLED
#endif


#ifdef STM32_PWM_USE_TIM1
#define PWM_CONF_TIM1 STM32_PWM_USE_TIM1
#else
#define PWM_CONF_TIM1 1
#endif
#define PWM_CONF1_DEF { \
  PWM_FREQUENCY, \
  PWM_FREQUENCY/TIM1_SERVO_HZ, \
  NULL, \
  { \
    { PWM_SERVO_3_ACTIVE, NULL }, \
    { PWM_SERVO_1_ACTIVE, NULL }, \
    { PWM_SERVO_2_ACTIVE, NULL }, \
    { PWM_SERVO_0_ACTIVE, NULL }, \
  }, \
  0, \
  0 \
}

#ifdef STM32_PWM_USE_TIM4
#define PWM_CONF_TIM4 STM32_PWM_USE_TIM4
#else
#define PWM_CONF_TIM4 1
#endif
#define PWM_CONF4_DEF { \
  PWM_FREQUENCY, \
  PWM_FREQUENCY/TIM4_SERVO_HZ, \
  NULL, \
  { \
    { PWM_OUTPUT_DISABLED, NULL }, \
    { PWM_OUTPUT_DISABLED, NULL }, \
    { PWM_SERVO_4_ACTIVE, NULL }, \
    { PWM_SERVO_5_ACTIVE, NULL }, \
  }, \
  0, \
  0 \
}

/**
 * UART2 
 *
 * chimera copypasta: (with optional flow control deactivated by default)
 */
#define UART2_GPIO_PORT_TX GPIOD
#define UART2_GPIO_TX GPIO5
#define UART2_GPIO_PORT_RX GPIOD
#define UART2_GPIO_RX GPIO6
#define UART2_GPIO_AF 7
#ifndef UART2_HW_FLOW_CONTROL
#define UART2_HW_FLOW_CONTROL FALSE
#endif

/**
 * UART8 (XBee slot), UART7 (GPS) and UART1 (Companion)
 * are configured as UART from ChibiOS board file
 */

/**
 * SBUS
 *
 * primary SBUS port is UART3
 * secondary port (in dual driver) is UART6
 */

// In case, do dynamic config of UARTs
#define USE_UART3_RX TRUE
#ifndef USE_UART3_TX // may be used in half duplex mode
#define USE_UART3_TX FALSE
#endif
#define UART3_GPIO_PORT_RX GPIOD
#define UART3_GPIO_RX GPIO9
#define UART3_GPIO_AF 7

#define USE_UART6_RX TRUE
#define USE_UART6_TX FALSE
#define UART6_GPIO_PORT_RX GPIOC
#define UART6_GPIO_RX GPIO6
#define UART6_GPIO_AF 8

/*
 * Spektrum
 *
 * Not supported yet in chibios arch
 * Only here for future reference
 *
 * primary Spektrum port is UART3
 * secondary port is UART6
 */
/* The line that is pulled low at power up to initiate the bind process */
/* These are not common between versions of lisa/mx and thus defined in the
 * version specific header files. */
#define SPEKTRUM_UART3_BANK UART3_GPIO_PORT_RX
#define SPEKTRUM_UART3_PIN UART3_GPIO_RX
#define SPEKTRUM_UART3_AF UART3_GPIO_AF
#define SPEKTRUM_UART3_DEV SD3

#define SPEKTRUM_UART6_BANK UART6_GPIO_PORT_RX
#define SPEKTRUM_UART6_PIN UART6_GPIO_RX
#define SPEKTRUM_UART6_AF UART6_GPIO_AF
#define SPEKTRUM_UART6_DEV SD6

/**
 * PPM radio defines
 *
 * available on RC1
 */
#define RC_PPM_TICKS_PER_USEC 6
#define PPM_TIMER_FREQUENCY 6000000
#define PPM_CHANNEL ICU_CHANNEL_2
#define PPM_TIMER ICUD5

/*
 * PWM input
 */
// PWM_INPUT 1 on PA0 (AUX3) (XXX: chimera copypasta)
/*
#define PWM_INPUT1_ICU            ICUD2
#define PWM_INPUT1_CHANNEL        ICU_CHANNEL_1
#define PWM_INPUT1_GPIO_PORT      GPIOA
#define PWM_INPUT1_GPIO_PIN       GPIO0
#define PWM_INPUT1_GPIO_AF        GPIO_AF1
*/

// PWM_INPUT 2 on PC7 (AUX7) (XXX: chimera copypasta)
/*
#define PWM_INPUT2_ICU            ICUD8
#define PWM_INPUT2_CHANNEL        ICU_CHANNEL_2
#define PWM_INPUT2_GPIO_PORT      GPIOC
#define PWM_INPUT2_GPIO_PIN       GPIO7
#define PWM_INPUT2_GPIO_AF        GPIO_AF3
*/

/**
 * I2C defines
 */
// Digital noise filter: 0 disabled, [0x1 - 0xF] enable up to n t_I2CCLK
#define STM32_CR1_DNF(n)          ((n & 0x0f) << 8)
// Timing register
#define I2C_FAST_400KHZ_DNF0_100NS_PCLK54MHZ_TIMINGR  (STM32_TIMINGR_PRESC(0U) | \
    STM32_TIMINGR_SCLDEL(10U) | STM32_TIMINGR_SDADEL(0U) | \
    STM32_TIMINGR_SCLH(34U)  | STM32_TIMINGR_SCLL(86U))

#define I2C1_CLOCK_SPEED 400000
#define I2C1_CFG_DEF { \
  .timingr = I2C_FAST_400KHZ_DNF0_100NS_PCLK54MHZ_TIMINGR, \
  .cr1 = STM32_CR1_DNF(0), \
  .cr2 = 0 \
}

#define I2C2_CLOCK_SPEED 400000
#define I2C2_CFG_DEF { \
  .timingr = I2C_FAST_400KHZ_DNF0_100NS_PCLK54MHZ_TIMINGR, \
  .cr1 = STM32_CR1_DNF(0), \
  .cr2 = 0 \
}

/**
 * SPI1 Config
 */
#define SPI1_GPIO_AF GPIO_AF5
#define SPI1_GPIO_PORT_MISO GPIOB
#define SPI1_GPIO_MISO GPIO4
#define SPI1_GPIO_PORT_MOSI GPIOB
#define SPI1_GPIO_MOSI GPIO5
#define SPI1_GPIO_PORT_SCK GPIOA
#define SPI1_GPIO_SCK GPIO5

/**
 * SPI2 Config
 */
#define SPI2_GPIO_AF GPIO_AF5
#define SPI2_GPIO_PORT_MISO GPIOC
#define SPI2_GPIO_MISO GPIO2
#define SPI2_GPIO_PORT_MOSI GPIOC
#define SPI2_GPIO_MOSI GPIO1
#define SPI2_GPIO_PORT_SCK GPIOD
#define SPI2_GPIO_SCK GPIO3

/* We do not have SPI3 available */

/**
 * SPI4 Config
 */
#define SPI4_GPIO_AF GPIO_AF5
#define SPI4_GPIO_PORT_MISO GPIOE
#define SPI4_GPIO_MISO GPIO5
#define SPI4_GPIO_PORT_MOSI GPIOE
#define SPI4_GPIO_MOSI GPIO6
#define SPI4_GPIO_PORT_SCK GPIOE
#define SPI4_GPIO_SCK GPIO12

// SLAVE0 on SPI1
#define SPI_SELECT_SLAVE0_PORT GPIOA
#define SPI_SELECT_SLAVE0_PIN GPIO15
// SLAVE1 on SPI2
#define SPI_SELECT_SLAVE1_PORT GPIOB
#define SPI_SELECT_SLAVE1_PIN GPIO12
// SLAVE2 on SPI4 (MPU9250 IMU CS)
#define SPI_SELECT_SLAVE2_PORT GPIOE
#define SPI_SELECT_SLAVE2_PIN GPIO4
// SLAVE3 on SPI4 (MS5611 Baro CS)
#define SPI_SELECT_SLAVE3_PORT GPIOE
#define SPI_SELECT_SLAVE3_PIN GPIO13

/**
 * Baro
 *
 * Apparently needed for backwards compatibility
 * with the ancient onboard baro boards
 */
#ifndef USE_BARO_BOARD
#define USE_BARO_BOARD 1
#endif

/**
 * SDIO
 */
#define SDIO_D0_PORT GPIOC
#define SDIO_D0_PIN GPIO8
#define SDIO_D1_PORT GPIOC
#define SDIO_D1_PIN GPIO9
#define SDIO_D2_PORT GPIOC
#define SDIO_D2_PIN GPIO10
#define SDIO_D3_PORT GPIOC
#define SDIO_D3_PIN GPIO11
#define SDIO_CK_PORT GPIOC
#define SDIO_CK_PIN GPIO12
#define SDIO_CMD_PORT GPIOD
#define SDIO_CMD_PIN GPIO2
#define SDIO_AF 12
// bat monitoring for file closing
#define SDLOG_BAT_ADC ADCD1
#define SDLOG_BAT_CHAN AD1_2_CHANNEL
// usb led status
#define SDLOG_USB_LED none
#define SDLOG_USB_VBUS_PORT GPIOA
#define SDLOG_USB_VBUS_PIN GPIO4


/*
 * Actuators for fixedwing
 */
 /* Default actuators driver */
 /* XXX: This is chimera copypasta might need channel reshuffeling... */
#define DEFAULT_ACTUATORS "subsystems/actuators/actuators_pwm.h"
#define ActuatorDefaultSet(_x,_y) ActuatorPwmSet(_x,_y)
#define ActuatorsDefaultInit() ActuatorsPwmInit()
#define ActuatorsDefaultCommit() ActuatorsPwmCommit()

#endif /* CONFIG_LISA_S_2_00_H */

