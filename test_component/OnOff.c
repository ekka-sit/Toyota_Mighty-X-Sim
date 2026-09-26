/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Key insert / engine start simulation
 ******************************************************************************
 *
 * Button PA10 : insert / remove the key (toggle each press)
 * Potentiometer PA4 (ADC1 channel 4) : turn to half or more to start engine
 * LED PB6 :
 *   OFF        -> key not inserted
 *   BLINKING   -> key inserted, engine not started yet
 *   SOLID ON   -> engine started
 *
 * Rule: engine can only start while the key is inserted (that is the
 * same thing as "LED is currently blinking"). The LED follows the
 * pot LIVE: turn it to half or more -> solid ON, turn it back below
 * half -> goes back to blinking. Pressing the button removes the key
 * and turns the LED off completely.
 */

#include <stdint.h>
#define	STM32F411xE
#include "stm32f4xx.h"

#define ADC_MAX_VALUE           4095u
#define HALF_ADC_VALUE          2048u

#define BLINK_TOGGLE_COUNT      50u
#define LOOP_DELAY              30000u

void GPIO_Init(void);
void ADC_Init(void);
uint16_t ADC_Read(void);
void Delay(uint32_t count);

int main(void)
{
    uint8_t keyInserted;
    uint8_t engineStarted;
    uint8_t buttonLatched;
    uint8_t ledBlinkState;
    uint32_t blinkCounter;
    uint16_t adcValue;

    GPIO_Init();
    ADC_Init();

    keyInserted = 0u;
    engineStarted = 0u;
    buttonLatched = 0u;
    ledBlinkState = 0u;
    blinkCounter = 0u;

    while (1)
    {
        /* --- Insert / remove key (PA10), pressed = pin reads 0 --- */
        if ((GPIOA->IDR & GPIO_IDR_ID10) == 0u)
        {
            if (buttonLatched == 0u)
            {
                buttonLatched = 1u;

                if (keyInserted == 1u)
                {
                    /* Key was in, this press removes it. Engine stops too. */
                    keyInserted = 0u;
                    engineStarted = 0u;
                }
                else
                {
                    /* Key was out, this press inserts it. */
                    keyInserted = 1u;
                }
            }
            else
            {
                /* Still held, do nothing */
            }
        }
        else
        {
            buttonLatched = 0u;
        }

        /* --- Check potentiometer, engine follows it live --- */
        adcValue = ADC_Read();

        if (keyInserted == 1u)
        {
            if (adcValue >= HALF_ADC_VALUE)
            {
                engineStarted = 1u;
            }
            else
            {
                /* Turned back down: engine stops, go back to blinking */
                engineStarted = 0u;
            }
        }
        else
        {
            /* No key: nothing to start */
            engineStarted = 0u;
        }

        /* --- Update LED PB6 to match the current state --- */
        if (engineStarted == 1u)
        {
            /* Engine running: solid ON */
            GPIOB->ODR |= GPIO_ODR_OD6;
            blinkCounter = 0u;
        }
        else if (keyInserted == 1u)
        {
            /* Key in, engine not started yet: blink */
            blinkCounter = blinkCounter + 1u;

            if (blinkCounter >= BLINK_TOGGLE_COUNT)
            {
                blinkCounter = 0u;

                if (ledBlinkState == 0u)
                {
                    ledBlinkState = 1u;
                    GPIOB->ODR |= GPIO_ODR_OD6;
                }
                else
                {
                    ledBlinkState = 0u;
                    GPIOB->ODR &= ~(GPIO_ODR_OD6);
                }
            }
            else
            {
                /* Not time to toggle yet */
            }
        }
        else
        {
            /* No key: LED off */
            GPIOB->ODR &= ~(GPIO_ODR_OD6);
            ledBlinkState = 0u;
            blinkCounter = 0u;
        }

        Delay(LOOP_DELAY);
    }
}

void GPIO_Init(void)
{
    RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN);
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    /* PA10 = key button, input with pull-up */
    GPIOA->MODER &= ~(GPIO_MODER_MODER10);
    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD10);
    GPIOA->PUPDR |= (0b01 << GPIO_PUPDR_PUPD10_Pos);

    /* PB6 = LED, output */
    GPIOB->MODER &= ~(GPIO_MODER_MODER6);
    GPIOB->MODER |= (0b01 << GPIO_MODER_MODER6_Pos);

    /* PA4 = potentiometer, analog input (ADC1_IN4) */
    GPIOA->MODER &= ~(GPIO_MODER_MODER4);
    GPIOA->MODER |= (0b11 << GPIO_MODER_MODER4_Pos);
}

void ADC_Init(void)
{
    ADC1->SMPR2 |= ADC_SMPR2_SMP4;

    ADC1->SQR1 &= ~(ADC_SQR1_L);
    ADC1->SQR3 &= ~(ADC_SQR3_SQ1);
    ADC1->SQR3 |= (4 << ADC_SQR3_SQ1_Pos);

    ADC1->CR2 |= ADC_CR2_ADON;
}

uint16_t ADC_Read(void)
{
    ADC1->CR2 |= ADC_CR2_SWSTART;

    while ((ADC1->SR & ADC_SR_EOC) == 0u)
    {
        /* Wait for the conversion to finish */
    }

    return (uint16_t)ADC1->DR;
}

void Delay(uint32_t count)
{
    uint32_t i;

    for (i = 0u; i < count; i++)
    {
        /* Busy wait */
    }
}