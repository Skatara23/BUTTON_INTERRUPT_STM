#include "stm32f4xx.h"

void EXTI0_IRQHandler(void)
{
    if (EXTI->PR & EXTI_PR_PR0) // Check if interrupt flag is set for EXTI Line 0
    {
        // External interrupt code here
        // Your code to handle the interrupt

        EXTI->PR |= EXTI_PR_PR0; // Clear the interrupt flag for EXTI Line 0

        // Toggle LED on GPIOB Pin 5
        GPIOB->ODR ^= (1U << 5);

        // Start PWM on GPIOB Pin 10
        TIM2->CCER |= TIM_CCER_CC3E; // Enable the output
        TIM2->CR1 |= TIM_CR1_CEN;     // Start the timer

        for (int i = 0; i < 1000000; i++)
        {
        }

        // Stop PWM after a delay
        TIM2->CCER &= ~TIM_CCER_CC3E; // Disable the output
        TIM2->CR1 &= ~TIM_CR1_CEN;     // Stop the timer
    }
}

void EXTI0_Config(void)
{
    // Enable the GPIOA and GPIOB clocks
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;

    // Configure GPIOA Pin 0 as input
    GPIOA->MODER &= ~GPIO_MODER_MODER0_Msk;

    // Configure GPIOB Pin 5 as output (LED)
    GPIOB->MODER |= (1U << 10);
    GPIOB->MODER &= ~(1U << 11);

    // Enable SYSCFG clock
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    // Connect EXTI Line 0 to PA0 pin
    SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI0_Msk;

    // Configure EXTI Line 0
    EXTI->IMR |= EXTI_IMR_MR0;    // Enable interrupt for EXTI Line 0
    EXTI->EMR &= ~EXTI_EMR_MR0;   // Disable event for EXTI Line 0
    EXTI->RTSR |= EXTI_RTSR_TR0;  // Enable rising edge trigger for EXTI Line 0
    EXTI->FTSR &= ~EXTI_FTSR_TR0; // Disable falling edge trigger for EXTI Line 0

    // Enable EXTI Line 0 Interrupt
    NVIC_SetPriority(EXTI0_IRQn, 0x0F);
    NVIC_EnableIRQ(EXTI0_IRQn);
}

void PWM_Config(void)
{
    // Enable TIM2 clock
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    // Configure GPIOB Pin 10 in alternate function mode (AF1) for TIM2_CH3
    GPIOB->MODER |= (1U << 21);
    GPIOB->MODER |= (1U << 20);

    GPIOB->AFR[1] |= (1U << 8); // AF1 for Pin 10 (TIM2_CH3)

    // Configure TIM2 for PWM generation
    TIM2->PSC = 83; // Assuming a 1 MHz clock (you may need to adjust this based on your system clock)
    TIM2->ARR = 19; // Gives a frequency of 50 kHz (1 MHz / (83 + 1) / (19 + 1) = 50 kHz)

    // Configure PWM mode for TIM2_CH3
    TIM2->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2; // PWM Mode 1

    // Disable the output initially
    TIM2->CCER &= ~TIM_CCER_CC3E;

    // Start the timer
    TIM2->CR1 |= TIM_CR1_CEN;
}

int main(void)
{
    // Configure external interrupt
    EXTI0_Config();

    // Configure PWM
    PWM_Config();

    while (1)
    {
        // Your main program loop
    }
}
