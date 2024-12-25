#include "stm32f4xx.h"

#define GPIOAEN  (1U<<0)		//led
#define GPIOCEN		(1U<<2)//for button

#define PIN5		(1U<<5)				//FOR LED ON PORT A
#define PIN13		(1U<<13)//FOR BUTTON ON PORT C


#define led5		PIN5
#define button 		PIN13

int main(void )

{
	/* enable the clock access to gpioa and gpioc port*/
	RCC -> AHB1ENR|=GPIOAEN|GPIOCEN;

	GPIOA->MODER|=(1U<<10);		// LED IS ON
	GPIOA->MODER&=~(1U<<11);		// LED BIT FOR 0 SET

	GPIOC->MODER &=~(1U<<26);			// 26 for 0 and 27 for 0
	GPIOC->MODER &=~(1U<<27);//input set for gpioc port


	while(1)
	{
		int i;
		if(GPIOC->IDR &(1U<<13))
		{
			GPIOA->ODR^=(1U<<5);
			for(i=0; i<2000; i++)
			{}
		}
		else
		{
			GPIOA->ODR&=~(1U<<5);
		}
	}

}
