/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Gear selector + throttle (RPM) + speed dashboard
 ******************************************************************************
 *
 * --- Gear selector (kept from the original Gear.c, unchanged logic) ---
 * Joystick: PC2 = VRx (ADC1_IN12), PC3 = VRy (ADC1_IN13)
 * 7-segment via BCD decoder: 2^0=PC7, 2^1=PA8, 2^2=PB10, 2^3=PA9
 * Digit 0 = Neutral, 1-5 = gears 1-5, 8 = Reverse
 *
 * --- Throttle potentiometer (NEW) ---
 * A 10k potentiometer, wired as a voltage divider:
 *   one outer pin  -> 3.3V
 *   other outer pin -> GND
 *   middle pin (wiper, the signal) -> PA0 (ADC1_IN0)
 * If the reading goes the wrong direction when you turn it, just swap
 * the two outer pins (3.3V and GND) - no code change needed for that.
 *
 * All 3 ADC channels (VRx, VRy, throttle pot) are read through ONE
 * interrupt handler in round-robin order: VRx -> VRy -> throttle ->
 * back to VRx. There is NO polling anywhere in this file - the main
 * loop only ever reads the latest values that the interrupt already
 * filled in.
 *
 * Throttle raw value (0-4095) is mapped to engine RPM between 800
 * (idle) and 4200 (redline) - this always updates, in every gear,
 * including neutral, since the engine can rev in neutral too.
 *
 * --- Speed (NEW) ---
 * Velocity (km/hr) = (RPM * 0.36) / (2.6526 * gear_ratio * final_drive)
 * In neutral, instead of snapping to 0, the speed coasts down a
 * little bit each loop until it reaches 0 (simple engine-braking
 * simulation).
 *
 * --- UART output (PA2, USART2, 9600 baud) ---
 * Only two lines are sent each loop:
 *   RPM: <value> rpm
 *   Velocity: <value> km/hr
 */

#include <stdint.h>
#define	STM32F411xE
#include "stm32f4xx.h"

/* ------------------------------------------------------------------ */
/* Settings                                                            */
/* ------------------------------------------------------------------ */

#define LOOP_DELAY             1000000u
#define CALIBRATION_DELAY      2000000u

#define ADC_CHANNEL_JOY_VRX         12u   /* PC2 */
#define ADC_CHANNEL_JOY_VRY         13u   /* PC3 */
#define ADC_CHANNEL_THROTTLE         0u   /* PA0 */

#define IDLE_RPM                800.0f
#define REDLINE_RPM             4200.0f

#define FINAL_DRIVE             4.300f

/* How much the speed drops per loop while coasting in neutral
   (km/hr per loop tick) - tune this to taste */
#define VELOCITY_DECAY_STEP     3.0f

/* ------------------------------------------------------------------ */
/* Globals                                                             */
/* ------------------------------------------------------------------ */

/* Updated by ADC_IRQHandler (round-robin), read from the main loop */
volatile uint16_t g_joyVrxRaw = 0u;
volatile uint16_t g_joyVryRaw = 0u;
volatile uint16_t g_throttleRaw = 0u;
volatile uint8_t g_adcChannelIndex = 0u; /* 0=VRx, 1=VRy, 2=throttle */

/* Filled in once at startup by CalibrateCenter() */
int32_t g_centerX = 0;
int32_t g_centerY = 0;

/* ------------------------------------------------------------------ */
/* Function prototypes                                                 */
/* ------------------------------------------------------------------ */

void GPIO_Init(void);
void ADC_Init(void);
void ADC_StartChannel(uint8_t channel);
void CalibrateCenter(void);
void Segment_Display(uint8_t number);
float ThrottleToRpm(uint16_t rawValue);
float GearRatioFor(uint8_t gearDigit);
void UART_Init(void);
void UART_SendChar(char c);
void UART_SendString(const char *text);
void UART_SendNumber(int32_t value);
void UART_SendFloatOneDecimal(float value);
void Delay(uint32_t count);

/* ------------------------------------------------------------------ */
/* main                                                                 */
/* ------------------------------------------------------------------ */

int main(void)
{
    int32_t offsetX;
    int32_t offsetY;
    uint8_t currentGear;
    float currentRpm;
    float currentVelocity;
    float gearRatio;

    /* Enable the FPU (CP10/CP11 full access) so float math for RPM
       and velocity actually uses hardware floating point */
    SCB->CPACR |= ((3UL << (10u * 2u)) | (3UL << (11u * 2u)));

    GPIO_Init();
    ADC_Init();
    UART_Init();

    UART_SendString("Calibrating - leave the joystick alone...\r\n");
    CalibrateCenter();

    currentVelocity = 0.0f;

    while (1)
    {
        offsetX = (int32_t)g_joyVrxRaw - g_centerX;
        offsetY = (int32_t)g_joyVryRaw - g_centerY;

        /* ---- Gear detection (unchanged from the original file) ---- */
        if ((offsetX >= -2500 && offsetX <= 120 && offsetY >= -2500 && offsetY <= 120) ||
            (offsetX > 120 && offsetY >= -2500 && offsetY <= 120) ||
            (offsetX < -2500 && offsetY >= -2500 && offsetY <= 120)) {
            currentGear = 0u;
        }
        else if (offsetX < -2500 && offsetY < -2500) {
            currentGear = 1u;
        }
        else if (offsetX < -2500 && offsetY > 120) {
            currentGear = 2u;
        }
        else if (offsetX >= -2500 && offsetX <= 120 && offsetY < -2500) {
            currentGear = 3u;
        }
        else if (offsetX >= -2500 && offsetX <= 120 && offsetY > 120) {
            currentGear = 4u;
        }
        else if (offsetX > 120 && offsetY < -2500) {
            currentGear = 5u;
        }
        else if (offsetX > 120 && offsetY > 120) {
            currentGear = 8u;
        }
        else {
            /* Anything not covered above: stay safe in neutral */
            currentGear = 0u;
        }

        Segment_Display(currentGear);

        /* ---- NEW: throttle pot -> RPM, always active, every gear ---- */
        currentRpm = ThrottleToRpm(g_throttleRaw);

        /* ---- NEW: velocity from RPM + current gear ---- */
        if (currentGear == 0u)
        {
            /* Neutral: coast down to 0 instead of snapping straight down */
            if (currentVelocity > VELOCITY_DECAY_STEP)
            {
                currentVelocity = currentVelocity - VELOCITY_DECAY_STEP;
            }
            else
            {
                currentVelocity = 0.0f;
            }
        }
        else
        {
            gearRatio = GearRatioFor(currentGear);
            currentVelocity = (currentRpm * 0.36f) / (2.6526f * gearRatio * FINAL_DRIVE);
        }

        /* ---- Dashboard: exactly the 2 lines requested ---- */
        UART_SendString("RPM: ");
        UART_SendFloatOneDecimal(currentRpm);
        UART_SendString(" rpm\r\n");

        UART_SendString("Velocity: ");
        UART_SendFloatOneDecimal(currentVelocity);
        UART_SendString(" km/hr\r\n");

        Delay(LOOP_DELAY);
    }
}

/* ------------------------------------------------------------------ */
/* NEW: throttle -> RPM and gear -> ratio helpers                      */
/* ------------------------------------------------------------------ */

/* Maps the raw throttle pot reading (0-4095) linearly onto the
   engine's RPM range, idle to redline */
float ThrottleToRpm(uint16_t rawValue)
{
    float ratio;
    float rpm;

    ratio = (float)rawValue / 4095.0f;
    rpm = IDLE_RPM + (ratio * (REDLINE_RPM - IDLE_RPM));

    return rpm;
}

/* Gear ratio table (Reverse is stored under digit 8, matching the
   7-segment code used for Reverse) */
float GearRatioFor(uint8_t gearDigit)
{
    float ratio;

    switch (gearDigit)
    {
        case 1u:
        {
            ratio = 3.928f;
            break;
        }
        case 2u:
        {
            ratio = 2.333f;
            break;
        }
        case 3u:
        {
            ratio = 1.451f;
            break;
        }
        case 4u:
        {
            ratio = 1.000f;
            break;
        }
        case 5u:
        {
            ratio = 0.851f;
            break;
        }
        case 8u:
        {
            /* Reverse */
            ratio = 4.743f;
            break;
        }
        default:
        {
            /* Neutral or anything unexpected: not used for velocity
               math (neutral is handled separately), but return
               something harmless just in case */
            ratio = 1.000f;
            break;
        }
    }

    return ratio;
}

/* ------------------------------------------------------------------ */
/* Gear display (unchanged from the original file)                     */
/* ------------------------------------------------------------------ */

/* Sends "number" as 4-bit binary to the BCD decoder IC:
   2^0 = PC7, 2^1 = PA8, 2^2 = PB10, 2^3 = PA9 */
void Segment_Display(uint8_t number)
{
    if (number & 0x01u) { GPIOC->ODR |= GPIO_ODR_OD7; }
    else { GPIOC->ODR &= ~GPIO_ODR_OD7; }

    if (number & 0x02u) { GPIOA->ODR |= GPIO_ODR_OD8; }
    else { GPIOA->ODR &= ~GPIO_ODR_OD8; }

    if (number & 0x04u) { GPIOB->ODR |= GPIO_ODR_OD10; }
    else { GPIOB->ODR &= ~GPIO_ODR_OD10; }

    if (number & 0x08u) { GPIOA->ODR |= GPIO_ODR_OD9; }
    else { GPIOA->ODR &= ~GPIO_ODR_OD9; }
}

/* Reads the resting joystick position once and remembers it as center
   (unchanged from the original file) */
void CalibrateCenter(void)
{
    Delay(CALIBRATION_DELAY);
    g_centerX = (int32_t)g_joyVrxRaw;
    g_centerY = (int32_t)g_joyVryRaw;
}

/* ------------------------------------------------------------------ */
/* GPIO / peripheral init                                              */
/* ------------------------------------------------------------------ */

void GPIO_Init(void)
{
    RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN);
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    /* Joystick VRx (PC2) and VRy (PC3), analog input */
    GPIOC->MODER &= ~(GPIO_MODER_MODER2);
    GPIOC->MODER |= (0b11 << GPIO_MODER_MODER2_Pos);

    GPIOC->MODER &= ~(GPIO_MODER_MODER3);
    GPIOC->MODER |= (0b11 << GPIO_MODER_MODER3_Pos);

    /* NEW: throttle potentiometer wiper (PA0), analog input (ADC1_IN0) */
    GPIOA->MODER &= ~(GPIO_MODER_MODER0);
    GPIOA->MODER |= (0b11 << GPIO_MODER_MODER0_Pos);

    /* USART2 TX (PA2), alternate function AF7 */
    GPIOA->MODER &= ~(GPIO_MODER_MODER2);
    GPIOA->MODER |= (0b10 << GPIO_MODER_MODER2_Pos);
    GPIOA->AFR[0] &= ~(0xFu << (2u * 4u));
    GPIOA->AFR[0] |= (7u << (2u * 4u));

    /* 7-segment control pins, output */
    GPIOC->MODER &= ~(GPIO_MODER_MODER7);
    GPIOC->MODER |= (0b01 << GPIO_MODER_MODER7_Pos);

    GPIOA->MODER &= ~(GPIO_MODER_MODER8 | GPIO_MODER_MODER9);
    GPIOA->MODER |= (0b01 << GPIO_MODER_MODER8_Pos) | (0b01 << GPIO_MODER_MODER9_Pos);

    GPIOB->MODER &= ~(GPIO_MODER_MODER10);
    GPIOB->MODER |= (0b01 << GPIO_MODER_MODER10_Pos);
}

void ADC_Init(void)
{
    /* Sample time: channel 0 (throttle) is in SMPR2, channels 12/13
       (joystick) are in SMPR1 */
    ADC1->SMPR2 |= ADC_SMPR2_SMP0;
    ADC1->SMPR1 |= (ADC_SMPR1_SMP12 | ADC_SMPR1_SMP13);

    ADC1->SQR1 &= ~(ADC_SQR1_L);

    /* Fire an interrupt every time a conversion finishes (EOC) */
    ADC1->CR1 |= ADC_CR1_EOCIE;
    NVIC_EnableIRQ(ADC_IRQn);

    ADC1->CR2 |= ADC_CR2_ADON;

    /* Kick off the round-robin: VRx first, the ISR takes it from here */
    g_adcChannelIndex = 0u;
    ADC_StartChannel(ADC_CHANNEL_JOY_VRX);
}

void ADC_StartChannel(uint8_t channel)
{
    ADC1->SQR3 &= ~(ADC_SQR3_SQ1);
    ADC1->SQR3 |= ((uint32_t)channel << ADC_SQR3_SQ1_Pos);
    ADC1->CR2 |= ADC_CR2_SWSTART;
}

/* Round-robin ADC interrupt: VRx -> VRy -> throttle -> back to VRx.
   No polling anywhere - each conversion's result is picked up here
   the moment it's ready, and the next one is started right away. */
void ADC_IRQHandler(void)
{
    uint16_t result;

    if ((ADC1->SR & ADC_SR_EOC) != 0u)
    {
        result = (uint16_t)ADC1->DR;

        if (g_adcChannelIndex == 0u)
        {
            g_joyVrxRaw = result;
            g_adcChannelIndex = 1u;
            ADC_StartChannel(ADC_CHANNEL_JOY_VRY);
        }
        else if (g_adcChannelIndex == 1u)
        {
            g_joyVryRaw = result;
            g_adcChannelIndex = 2u;
            ADC_StartChannel(ADC_CHANNEL_THROTTLE);
        }
        else
        {
            g_throttleRaw = result;
            g_adcChannelIndex = 0u;
            ADC_StartChannel(ADC_CHANNEL_JOY_VRX);
        }
    }
    else
    {
        /* No action */
    }
}

/* ------------------------------------------------------------------ */
/* UART                                                                 */
/* ------------------------------------------------------------------ */

void UART_Init(void)
{
    /* 9600 baud at 16 MHz HSI (the default clock after reset) */
    USART2->BRR = 0x683u;
    USART2->CR1 |= (USART_CR1_UE | USART_CR1_TE);
}

void UART_SendChar(char c)
{
    while ((USART2->SR & USART_SR_TXE) == 0u)
    {
        /* Wait until the transmit buffer is empty */
    }

    USART2->DR = (uint8_t)c;
}

void UART_SendString(const char *text)
{
    uint32_t i;

    i = 0u;

    while (text[i] != '\0')
    {
        UART_SendChar(text[i]);
        i = i + 1u;
    }
}

/* Converts a signed integer to decimal text and sends it */
void UART_SendNumber(int32_t value)
{
    char digits[12];
    uint32_t magnitude;
    uint8_t digitCount;
    uint8_t i;

    if (value < 0)
    {
        UART_SendChar('-');
        magnitude = (uint32_t)(-value);
    }
    else
    {
        magnitude = (uint32_t)value;
    }

    digitCount = 0u;

    if (magnitude == 0u)
    {
        digits[0] = '0';
        digitCount = 1u;
    }
    else
    {
        while (magnitude > 0u)
        {
            digits[digitCount] = (char)('0' + (magnitude % 10u));
            magnitude = magnitude / 10u;
            digitCount = digitCount + 1u;
        }
    }

    i = digitCount;

    while (i > 0u)
    {
        i = i - 1u;
        UART_SendChar(digits[i]);
    }
}

/* NEW: prints a float with exactly one digit after the decimal point
   (e.g. 3241.0, 87.6) - simple manual rounding, no printf needed */
void UART_SendFloatOneDecimal(float value)
{
    int32_t wholePart;
    int32_t tenths;
    float scaledValue;

    if (value < 0.0f)
    {
        UART_SendChar('-');
        value = -value;
    }

    /* Multiply by 10 and round, so we can work in whole "tenths" */
    scaledValue = (value * 10.0f) + 0.5f;
    tenths = (int32_t)scaledValue;

    wholePart = tenths / 10;
    tenths = tenths % 10;

    UART_SendNumber(wholePart);
    UART_SendChar('.');
    UART_SendNumber(tenths);
}

void Delay(uint32_t count)
{
    uint32_t i;

    for (i = 0u; i < count; i++)
    {
        /* Busy wait */
    }
}