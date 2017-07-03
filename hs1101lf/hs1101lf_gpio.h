/*
 * hs1101lf_gpio.h
 *
 *  Created on: 03/lug/2017
 *      Author: daniele
 */

#ifndef HS1101LF_GPIO_H_
#define HS1101LF_GPIO_H_

/**
 * struct hs1101lf_gpio_platform_data - Platform-dependent data for hs1101lf-gpio
 * @pin: GPIO pin to use
 * @is_open_drain: GPIO pin is configured as open drain
 */
struct hs1101lf_gpio_platform_data {
  unsigned int pin;
  unsigned int is_open_drain :1;
};

#endif /* HS1101LF_GPIO_H_ */
