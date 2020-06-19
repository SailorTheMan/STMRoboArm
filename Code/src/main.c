#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <stdbool.h>

void GPIO_Init (void);
void RCC_Init(void);
void PWM_Init (void);
void moveServo(uint16_t ccr, uint16_t channel, uint16_t delay);
void vKeyBoardScan(void * arguments);

bool isGripped;

int main (void){
	
	RCC_Init();
	
	PWM_Init();
	
	GPIO_Init();
	
	xTaskCreate(vKeyBoardScan, "KeyboardScan", 32, NULL, 1, NULL);
	
	vTaskStartScheduler();
	
	while(1)
	{
		
	}
	
}

/***********************************************************************************************/

void RCC_Init(void){
	
	RCC -> CR |= ((uint32_t) RCC_CR_HSEON);                    // Enable HSE
	while (!(RCC -> CR & RCC_CR_HSERDY));                      // HSE Ready
	
	FLASH -> ACR = FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY;       // Flash Clock
	
	RCC -> CFGR |= RCC_CFGR_HPRE_DIV1;                         // AHB Prescaller /1
	RCC -> CFGR |= RCC_CFGR_PPRE1_DIV2;                        // APB1 Prescaller /2
	RCC -> CFGR |= RCC_CFGR_PPRE2_DIV1;                        // APB2 Prescaller /1
	
	RCC -> CFGR &= ~RCC_CFGR_PLLMULL;                          // Clear PLLMULL bits
	RCC -> CFGR &= ~RCC_CFGR_PLLSRC;                           // Clear PLLSRC bits 
	RCC -> CFGR &= ~RCC_CFGR_PLLXTPRE;                         // Clear PLLXTPRE bits
	
	RCC -> CFGR |= RCC_CFGR_PLLSRC_HSE;                        // HSE Source
	RCC -> CFGR |= RCC_CFGR_PLLXTPRE_HSE;                      // PLLXTRPE div /1
	RCC -> CFGR |= RCC_CFGR_PLLMULL9;                          // PLLMULL x9
	
	RCC -> CR |= RCC_CR_PLLON;                                 // Enable PLL
	while ((RCC -> CR & RCC_CR_PLLRDY) == 0);                  // PLL Ready
	
	RCC -> CFGR &= ~ RCC_CFGR_SW;                              // Clear SW bits
	RCC -> CFGR |= RCC_CFGR_SW_PLL;                            // SYSCLK = PLL
	while ((RCC -> CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_1);    // PLL is used
	
}

void PWM_Init (void){
	
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	
	GPIOA->CRL &= ~GPIO_CRL_CNF1;
	GPIOA->CRL |= GPIO_CRL_CNF1_1;
	
	GPIOA->CRL &= ~GPIO_CRL_MODE1;
	GPIOA->CRL |= GPIO_CRL_MODE1;
	
	GPIOA->CRL &= ~GPIO_CRL_CNF0;
	GPIOA->CRL |= GPIO_CRL_CNF0_1;
	
	GPIOA->CRL &= ~GPIO_CRL_MODE0;
	GPIOA->CRL |= GPIO_CRL_MODE0;
	
	GPIOA->CRL &= ~GPIO_CRL_CNF2;
	GPIOA->CRL |= GPIO_CRL_CNF2_1;
	
	GPIOA->CRL &= ~GPIO_CRL_MODE2;
	GPIOA->CRL |= GPIO_CRL_MODE2;
	
	TIM2->PSC = 72 - 1;
	TIM2->ARR = 20000;
	TIM2->CCR2 = 1500;
	TIM2->CCR1 = 2000;
	TIM2->CCR3 = 2000;
	
	TIM2->CCMR1 |= TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2;
	TIM2->CCER |= TIM_CCER_CC2E;
	TIM2->CCER &= ~TIM_CCER_CC2P;
	
	TIM2->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;
	TIM2->CCER |= TIM_CCER_CC1E;
	TIM2->CCER &= ~TIM_CCER_CC1P;
	
	TIM2->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2;
	TIM2->CCER |= TIM_CCER_CC3E;
	TIM2->CCER &= ~TIM_CCER_CC3P;
	
	TIM2->CR1 &= ~TIM_CR1_DIR;
	TIM2->CR1 |= TIM_CR1_CEN;
	
}

void GPIO_Init(void){
	
	RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
	
	GPIOB -> CRH &= ~GPIO_CRH_CNF8;
	
	GPIOB -> CRH |= GPIO_CRH_CNF8_1;
	GPIOB -> CRH &= ~GPIO_CRH_MODE8;
	GPIOB -> ODR &= ~GPIO_ODR_ODR8;
	
	GPIOB -> CRH &= ~GPIO_CRH_CNF9;
	
	GPIOB -> CRH |= GPIO_CRH_CNF9_1;
	GPIOB -> CRH &= ~GPIO_CRH_MODE9;
	GPIOB -> ODR &= ~GPIO_ODR_ODR9;
	
	GPIOB -> CRH &= ~GPIO_CRH_CNF12;
	
	GPIOB -> CRH |= GPIO_CRH_CNF12_1;
	GPIOB -> CRH &= ~GPIO_CRH_MODE12;
	GPIOB -> ODR &= ~GPIO_ODR_ODR12;
	
	GPIOB -> CRL &= ~GPIO_CRL_CNF5;
	
	GPIOB -> CRL |= GPIO_CRL_CNF5_1;
	GPIOB -> CRL &= ~GPIO_CRL_MODE5;
	GPIOB -> ODR &= ~GPIO_ODR_ODR5;
	
	GPIOB -> CRL &= ~GPIO_CRL_CNF6;
	
	GPIOB -> CRL |= GPIO_CRL_CNF6_1;
	GPIOB -> CRL &= ~GPIO_CRL_MODE6;
	GPIOB -> ODR &= ~GPIO_ODR_ODR6;
	
	GPIOB -> CRH &= ~GPIO_CRH_CNF12;
	
	GPIOB -> CRH &= ~GPIO_CRH_CNF12;
	GPIOB -> CRH |= GPIO_CRH_MODE12;
	
	GPIOB -> CRL &= ~GPIO_CRL_CNF7;
	
	GPIOB -> CRL &= ~GPIO_CRL_CNF7;
	GPIOB -> CRL |= GPIO_CRL_MODE7;
	
	GPIOC -> CRH &= ~GPIO_CRH_CNF13;
	GPIOC -> CRH |= GPIO_CRH_MODE13_0;
	GPIOC -> BSRR |= GPIO_BSRR_BS13;
	
	
}


void vKeyBoardScan(void *arguments){
	while (1)
	{
		uint16_t ccr2 = TIM2 -> CCR2;
		uint16_t ccr1 = TIM2 -> CCR1;
		uint16_t delta2 = 10;
		uint16_t delta1 = 10;
		
		GPIOB -> BSRR |= GPIO_BSRR_BS7;
		
		if (((GPIOB -> IDR & GPIO_IDR_IDR6) != 0) && (ccr2 <= 2500 - delta2)){
			ccr2 = ccr2 + delta2;
			moveServo(ccr2, 2, 20);
			continue;
		 }
		
		 else if (((GPIOB -> IDR & GPIO_IDR_IDR5) != 0) && (ccr2 >= 500 + delta2)){
			 ccr2 = ccr2 - delta2;
			 moveServo(ccr2, 2, 20);
			 continue;
		 }
		
		 else if (((GPIOB -> IDR & GPIO_IDR_IDR8) != 0) && (ccr1 <= 2500 - delta1)){
			ccr1 = ccr1 + delta1;
			moveServo(ccr1, 1, 20);
			 continue;
		 }
		
		 else if (((GPIOB -> IDR & GPIO_IDR_IDR9) != 0) && (ccr1 >= 1500 + delta1)){
			 ccr1 = ccr1 - delta1;
			 moveServo(ccr1, 1, 20);
			 continue;
		 }
		 GPIOB -> BSRR |= GPIO_BSRR_BR7;
		
		 GPIOB -> BSRR |= GPIO_BSRR_BS12;
		
		 if (((GPIOB -> IDR & GPIO_IDR_IDR6) != 0) && (!isGripped)){
			isGripped = true;
			moveServo(2300, 3, 500);
		 }
		 
		else if (((GPIOB -> IDR & GPIO_IDR_IDR6) != 0) && (isGripped)){
			isGripped = false;
			moveServo(2000, 3, 500);
		 }
		 GPIOB -> BSRR |= GPIO_BSRR_BR12;
		 
	}
	
}

void moveServo(uint16_t ccr, uint16_t channel, uint16_t delay){
	
	if (channel == 1){
		TIM2->CCR1 = ccr;
	}
	
	else if (channel == 2){
		TIM2->CCR2 = ccr;
	}
	
	else if (channel == 3){
		TIM2->CCR3 = ccr;
	}
	
	vTaskDelay(delay);

}
