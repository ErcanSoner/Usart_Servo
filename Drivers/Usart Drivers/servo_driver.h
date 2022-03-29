/*
 * servo_driver.h
 *
 *  Created on: Nov 5, 2021
 *      Author: ercan
 */

#ifndef USART_DRIVERS_SERVO_DRIVER_H_
#define USART_DRIVERS_SERVO_DRIVER_H_

#include "stdint.h"

void servo_pin(void);
void servo_init(void);
void servo_enable(void);
void servo_disable(void);
void servo_set_duty_cycle();


#endif /* USART_DRIVERS_SERVO_DRIVER_H_ */
