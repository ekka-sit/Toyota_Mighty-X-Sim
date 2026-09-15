/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Joystick test - interrupt driven, with center calibration and 7-Segment
 ******************************************************************************
 */

#include <stdint.h>
#define STM32F411xE
#include "stm32f4xx.h"

#define LOOP_DELAY             1000000u
#define CALIBRATION_DELAY      2000000u

#define ADC_CHANNEL_JOY_VRX         12u   /* PC2 */
#define ADC_CHANNEL_JOY_VRY         13u   /* PC3 */

#define JOY_DEAD_ZONE               600

volatile uint16_t g_joyVrxRaw = 0u;
volatile uint16_t g_joyVryRaw = 0u;
volatile uint8_t g_currentChannelIsX = 1u;

int32_t g_centerX = 0;
int32_t g_centerY = 0;

void GPIO_Init(void);
void ADC_Init(void);
void ADC_StartChannel(uint8_t channel);
void CalibrateCenter(void);
void UART_Init(void);
void UART_SendChar(char c);
void UART_SendString(const char *text);
void UART_SendNumber(int32_t value);
void SendZoneName(int32_t offset, const char *lowName, const char *highName);
void Delay(uint32_t count);

// ฟังก์ชันสำหรับ 7-Segment
void Segment_Display(uint8_t number);

int main(void)
{
    int32_t offsetX;
    int32_t offsetY;

    GPIO_Init();
    ADC_Init();
    UART_Init();

    UART_SendString("Calibrating - leave the joystick alone...\r\n");
    CalibrateCenter();

    UART_SendString("Center X: ");
    UART_SendNumber(g_centerX);
    UART_SendString("   Center Y: ");
    UART_SendNumber(g_centerY);
    UART_SendString("\r\n\r\n");

    while (1)
    {
        offsetX = (int32_t)g_joyVrxRaw - g_centerX;
        offsetY = (int32_t)g_joyVryRaw - g_centerY;

        UART_SendString("X: ");
        UART_SendNumber(offsetX);
        UART_SendString(" (");
        SendZoneName(offsetX, "LEFT", "RIGHT");
        UART_SendString(")   Y: ");
        UART_SendNumber(offsetY);
        UART_SendString(" (");
        SendZoneName(offsetY, "UP", "DOWN");
        UART_SendString(")\r\n");

        // -------------------------------------------------------------
        // การประมวลผลเงื่อนไขสำหรับแสดงผล 7-Segment (รวม 2 กรณีย่อยที่เพิ่มเข้ามา)
        // -------------------------------------------------------------

        // แสดงเลข 0: รวมทั้งกรณีเดิม และอีก 2 กรณีย่อย (x > 120 และ x < -2500 เมื่อ y อยู่ในช่วง -2500 ถึง 120)
        if ((offsetX >= -2500 && offsetX <= 120 && offsetY >= -2500 && offsetY <= 120) ||
            (offsetX > 120 && offsetY >= -2500 && offsetY <= 120) ||
            (offsetX < -2500 && offsetY >= -2500 && offsetY <= 120)) {
            Segment_Display(0);
        }
        else if (offsetX < -2500 && offsetY < -2500) {
            Segment_Display(1);
        }
        else if (offsetX < -2500 && offsetY > 120) {
            Segment_Display(2);
        }
        else if (offsetX >= -2500 && offsetX <= 120 && offsetY < -2500) {
            Segment_Display(3);
        }
        else if (offsetX >= -2500 && offsetX <= 120 && offsetY > 120) {
            Segment_Display(4);
        }
        else if (offsetX > 120 && offsetY < -2500) {
            Segment_Display(5);
        }
        else if (offsetX > 120 && offsetY > 120) {
            Segment_Display(8);
        }

        Delay(LOOP_DELAY);
    }
}

// ------------------------------------------------------------------
// ฟังก์ชันแปลงตัวเลขเป็นเลขฐานสอง (BCD) ส่งไปควบคุม 7-Segment
// ข้อมูลอ้างอิง: 2^0 = PC7, 2^1 = PA8, 2^2 = PB10, 2^3 = PA9
// ------------------------------------------------------------------
void Segment_Display(uint8_t number)
{
    // Bit 0: PC7
    if (number & 0x01) { GPIOC->ODR |= GPIO_ODR_OD7; }
    else { GPIOC->ODR &= ~GPIO_ODR_OD7; }

    // Bit 1: PA8
    if (number & 0x02) { GPIOA->ODR |= GPIO_ODR_OD8; }
    else { GPIOA->ODR &= ~GPIO_ODR_OD8; }

    // Bit 2: PB10
    if (number & 0x04) { GPIOB->ODR |= GPIO_ODR_OD10; }
    else { GPIOB->ODR &= ~GPIO_ODR_OD10; }

    // Bit 3: PA9
    if (number & 0x08) { GPIOA->ODR |= GPIO_ODR_OD9; }
    else { GPIOA->ODR &= ~GPIO_ODR_OD9; }
}

void CalibrateCenter(void)
{
    Delay(CALIBRATION_DELAY);
    g_centerX = (int32_t)g_joyVrxRaw;
    g_centerY = (int32_t)g_joyVryRaw;
}

void SendZoneName(int32_t offset, const char *lowName, const char *highName)
{
    if (offset < -JOY_DEAD_ZONE) { UART_SendString(lowName); }
    else if (offset > JOY_DEAD_ZONE) { UART_SendString(highName); }
    else { UART_SendString("center"); }
}

void GPIO_Init(void)
{
    RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN);
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    // ตั้งค่าขา Joystick VRx (PC2) และ VRy (PC3) เป็น Analog Input
    GPIOC->MODER &= ~(GPIO_MODER_MODER2);
    GPIOC->MODER |= (0b11 << GPIO_MODER_MODER2_Pos);

    GPIOC->MODER &= ~(GPIO_MODER_MODER3);
    GPIOC->MODER |= (0b11 << GPIO_MODER_MODER3_Pos);

    // ตั้งค่าขา USART2 TX (PA2)
    GPIOA->MODER &= ~(GPIO_MODER_MODER2);
    GPIOA->MODER |= (0b10 << GPIO_MODER_MODER2_Pos);
    GPIOA->AFR[0] &= ~(0xFu << (2u * 4u));
    GPIOA->AFR[0] |= (7u << (2u * 4u));

    // ตั้งค่าขาควบคุม 7-Segment เป็น Output (0b01)
    // PC7 (2^0)
    GPIOC->MODER &= ~(GPIO_MODER_MODER7);
    GPIOC->MODER |= (0b01 << GPIO_MODER_MODER7_Pos);

    // PA8 (2^1) และ PA9 (2^3)
    GPIOA->MODER &= ~(GPIO_MODER_MODER8 | GPIO_MODER_MODER9);
    GPIOA->MODER |= (0b01 << GPIO_MODER_MODER8_Pos) | (0b01 << GPIO_MODER_MODER9_Pos);

    // PB10 (2^2)
    GPIOB->MODER &= ~(GPIO_MODER_MODER10);
    GPIOB->MODER |= (0b01 << GPIO_MODER_MODER10_Pos);
}

void ADC_Init(void)
{
    ADC1->SMPR1 |= (ADC_SMPR1_SMP12 | ADC_SMPR1_SMP13);
    ADC1->SQR1 &= ~(ADC_SQR1_L);
    ADC1->CR1 |= ADC_CR1_EOCIE;
    NVIC_EnableIRQ(ADC_IRQn);
    ADC1->CR2 |= ADC_CR2_ADON;
    g_currentChannelIsX = 1u;
    ADC_StartChannel(ADC_CHANNEL_JOY_VRX);
}

void ADC_StartChannel(uint8_t channel)
{
    ADC1->SQR3 &= ~(ADC_SQR3_SQ1);
    ADC1->SQR3 |= ((uint32_t)channel << ADC_SQR3_SQ1_Pos);
    ADC1->CR2 |= ADC_CR2_SWSTART;
}

void ADC_IRQHandler(void)
{
    uint16_t result;
    if ((ADC1->SR & ADC_SR_EOC) != 0u)
    {
        result = (uint16_t)ADC1->DR;
        if (g_currentChannelIsX == 1u)
        {
            g_joyVrxRaw = result;
            g_currentChannelIsX = 0u;
            ADC_StartChannel(ADC_CHANNEL_JOY_VRY);
        }
        else
        {
            g_joyVryRaw = result;
            g_currentChannelIsX = 1u;
            ADC_StartChannel(ADC_CHANNEL_JOY_VRX);
        }
    }
}

void UART_Init(void)
{
    USART2->BRR = 0x683u;
    USART2->CR1 |= (USART_CR1_UE | USART_CR1_TE);
}

void UART_SendChar(char c)
{
    while ((USART2->SR & USART_SR_TXE) == 0u) { }
    USART2->DR = (uint8_t)c;
}

void UART_SendString(const char *text)
{
    uint32_t i = 0u;
    while (text[i] != '\0')
    {
        UART_SendChar(text[i]);
        i = i + 1u;
    }
}

void UART_SendNumber(int32_t value)
{
    char digits[12];
    uint32_t magnitude;
    uint8_t digitCount = 0u;
    uint8_t i;

    if (value < 0) {
        UART_SendChar('-');
        magnitude = (uint32_t)(-value);
    } else {
        magnitude = (uint32_t)value;
    }

    if (magnitude == 0u) {
        digits[0] = '0';
        digitCount = 1u;
    } else {
        while (magnitude > 0u) {
            digits[digitCount] = (char)('0' + (magnitude % 10u));
            magnitude = magnitude / 10u;
            digitCount = digitCount + 1u;
        }
    }

    i = digitCount;
    while (i > 0u) {
        i = i - 1u;
        UART_SendChar(digits[i]);
    }
}

void Delay(uint32_t count)
{
    uint32_t i;
    for (i = 0u; i < count; i++) { }
}
