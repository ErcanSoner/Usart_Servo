/*
 * servo_driver.c
 *
 *  Created on: Nov 5, 2021
 *      Author: ercan
 */

#include "stm32f0xx_hal.h"
#include "servo_driver.h"

void servo_pin(){

	__HAL_RCC_GPIOB_CLK_ENABLE();

	/////B5 TIM3 CH2
	GPIOB->MODER &=~(1<<10); //PORTB PIN5 AF SECILDI
	GPIOB->MODER |=(1<<11);
	GPIOB->OTYPER &=~(0<<5); //AF PP SECILDI
	GPIOB->OSPEEDR |=(1<<10);  //HIGH SPEED
	GPIOB->OSPEEDR |=(1<<11);
	GPIOB->PUPDR &=~(1<<10); //NO PP
	GPIOB->PUPDR &=~(1<<11);

	GPIOB->AFR[0] &=~(1<<23);  //AF1 SECILDI
	GPIOB->AFR[0] &=~(1<<22);
	GPIOB->AFR[0] &=~(1<<21);
	GPIOB->AFR[0] |=(1<<20);

	/////B4 TIM CH1
	GPIOB->MODER &=~(1<<8); //PORTB PIN4 AF SECILDI
	GPIOB->MODER |=(1<<9);
	GPIOB->OTYPER &=~(0<<4); //AF PP SECILDI
	GPIOB->OSPEEDR |=(1<<8);  //HIGH SPEED
	GPIOB->OSPEEDR |=(1<<9);
	GPIOB->PUPDR &=~(1<<8); //NO PP
	GPIOB->PUPDR &=~(1<<9);

	GPIOB->AFR[0] &=~(1<<19);  //AF1 SECILDI
	GPIOB->AFR[0] &=~(1<<18);
	GPIOB->AFR[0] &=~(1<<17);
	GPIOB->AFR[0] |=(1<<16);

	///B0 TIM3 CH3
	GPIOB->MODER &= ~(1 << 0); //PORTB PIN4 AF SECILDI
	GPIOB->MODER |= (1 << 1);
	GPIOB->OTYPER &= ~(0 << 0); //AF PP SECILDI
	GPIOB->OSPEEDR |= (1 << 0);  //HIGH SPEED
	GPIOB->OSPEEDR |= (1 << 1);
	GPIOB->PUPDR &= ~(1 << 0); //NO PP
	GPIOB->PUPDR &= ~(1 << 1);

	GPIOB->AFR[0] &= ~(1 << 3);  //AF1 SECILDI
	GPIOB->AFR[0] &= ~(1 << 2);
	GPIOB->AFR[0] &= ~(1 << 1);
	GPIOB->AFR[0] |= (1 << 0);

	///B1 TIM3 CH4
	GPIOB->MODER &= ~(1 << 2); //PORTB PIN4 AF SECILDI
	GPIOB->MODER |= (1 << 3);
	GPIOB->OTYPER &= ~(0 << 1); //AF PP SECILDI
	GPIOB->OSPEEDR |= (1 << 2);  //HIGH SPEED
	GPIOB->OSPEEDR |= (1 << 3);
	GPIOB->PUPDR &= ~(1 << 2); //NO PP
	GPIOB->PUPDR &= ~(1 << 3);

	GPIOB->AFR[0] &= ~(1 << 7);  //AF1 SECILDI
	GPIOB->AFR[0] &= ~(1 << 6);
	GPIOB->AFR[0] &= ~(1 << 5);
	GPIOB->AFR[0] |= (1 << 4);


}

void servo_init(){

	__HAL_RCC_TIM3_CLK_ENABLE();

	TIM3->PSC = 47; // Timer clock = 48 mhz / 24 = 2Mhz
	TIM3->ARR = 20000;  // Period ==> (2 Mhz / 100) = 20 Khz
	TIM3->CCR1 = 500;
	TIM3->CCR2 = 500;  // Duty Cycle
	TIM3->CCR3 = 500;
	TIM3->CCR4 = 500;

	// CH-1 PWM MODE
	TIM3->CCMR1 |= TIM_CCMR1_OC1M_2;
	TIM3->CCMR1 |= TIM_CCMR1_OC1M_1;
	TIM3->CCMR1 |= TIM_CCMR1_OC1PE;

	// CH-2 PWM MODE
	TIM3->CCMR1 |= TIM_CCMR1_OC2M_2;
	TIM3->CCMR1 |= TIM_CCMR1_OC2M_1;
	TIM3->CCMR1 |= TIM_CCMR1_OC2PE;

	// CH-3 PWM MODE
	TIM3->CCMR2 |= TIM_CCMR2_OC3M_2;
	TIM3->CCMR2 |= TIM_CCMR2_OC3M_1;
	TIM3->CCMR2 |= TIM_CCMR2_OC3PE;

	// CH-4 PWM MODE
	TIM3->CCMR2 |= TIM_CCMR2_OC4M_2;
	TIM3->CCMR2 |= TIM_CCMR2_OC4M_1;
	TIM3->CCMR2 |= TIM_CCMR2_OC4PE;

	TIM3->CCER |=(1<<0);
	TIM3->CCER |=(1<<4);
	TIM3->CCER |=(1<<8);
	TIM3->CCER |= TIM_CCER_CC4E;
}

void servo_enable(){

	// Enable Timer
	TIM3->CR1 &=~(1<<7);
	TIM3->CR1 |= TIM_CR1_CEN;
	TIM3->EGR |= TIM_EGR_UG;

}


void servo_disable(){

	// Disable Timer
	TIM3->CR1 &= ~(TIM_CR1_CEN);
}

void servo_set_duty_cycle(uint32_t duty)
{
		TIM3->CCR2 = duty;
}
