/**
 ******************************************************************************
 * @file           : main.c / mighty_x_sim.c
 * @brief          : Toyota Mighty-X (2L) Driving Simulator - STM32F411RE
 ******************************************************************************
 *
 * [การปรับปรุงโครงสร้างตามข้อกำหนดของโครงการ]
 * 1. Hardware Drivers & Interrupt System:
 *    - Hardware Timer (TIM2): สร้างสัญญาณจังหวะ 10ms (Time-base)
 *    - ADC1 + DMA2: อ่านค่า Analogs (คันเร่ง, Joystick VRx/VRy, Potentiometer) 
 *      แบบ Scan Mode + DMA อัตโนมัติทุก 10ms โดยไม่มีการ Polling หรือสั่งใน ISR
 *    - EXTI: รองรับสวิทช์คลัทช์ (PB3), สวิทช์เบรก (PB4) และปุ่มกุญแจ (PA10)
 *    - UART2: ส่งข้อมูล Dashboard แบบ Non-blocking ผ่าน TX Interrupt + Ring Buffer
 *
 * 2. Software Debounce Filter:
 *    - ใช้ Software Filter กรองสัญญาณ Mechanical Bounce ของปุ่ม EXTI ใน 10ms Loop
 *
 * 3. Physics & Drivetrain Refactoring:
 *    - Decay Calculations (Linear Scaling): Engine Brake แปรผันตามอัตราอัตราทดเกียร์
 *    - Power Disconnect Sequence: แยกสภาวะตัดกำลัง (เหยียบคลัทช์/เกียร์ว่าง) และข้ามการเช็คเครื่องดับ
 *    - Smooth Re-engagement (Rev-Sync): ไม่ดับเครื่องขณะปล่อยคลัทช์ตอนรถกำลังไหล
 *
 * 4. MISRA-C Compliance & Code Readability:
 *    - แยก Layer ระหว่าง Hardware Driver และ Application Logic ชัดเจน
 *    - ใช้ stdint.h Explicit Type Casting และโครงสร้างโค้ดที่เข้าใจง่าย
 ******************************************************************************
 */

#include <stdint.h>
#define STM32F411xE
#include "stm32f4xx.h"

/* ==================================================================== */
/* 1. CONSTANTS & DEFINITIONS (การกำหนดค่าคงที่)                          */
/* ==================================================================== */

/* --- จังหวะการทำงานระบบ (Timebase Settings) --- */
#define CONTROL_LOOP_PERIOD_MS       10u         /* ลูปหลักทำงานทุกๆ 10ms */
#define UART_REPORT_PERIOD_TICKS     20u         /* ส่งออก UART ทุกๆ 20 ticks (200ms) */

/* --- กุญแจ และ สตาร์ท --- */
#define HALF_ADC_VALUE             2048u
#define BLINK_NORMAL_TICKS           50u         /* กระพริบปกติทุก 500ms */
#define BLINK_STALL_TICKS            10u         /* กระพริบเตือนดับทุก 100ms */
#define KEY_DEBOUNCE_MS             200u         /* หน่วงปุ่มกุญแจ 200ms */

/* --- เครื่องยนต์ และ ฟิสิกส์การขับเคลื่อน --- */
#define IDLE_RPM                   800.0f
#define REDLINE_RPM               4200.0f
#define STALL_RPM_THRESHOLD        600.0f       /* เกณฑ์รอบเครื่องตกจนดับ */
#define GEAR2_LAUNCH_RPM_MIN      1200.0f       /* รอบขั้นต่ำในการออกตัวเกียร์ 2 */
#define FINAL_DRIVE                4.300f       /* อัตราทดเฟืองท้าย */

/* --- อัตราการชะลอตัว (Decay Rates - หน่วย: km/h ต่อ 10ms tick) --- */
#define BASE_DECAY                 0.020f       /* Engine brake base decay (Linear Scaling) */
#define COAST_DECAY                0.003f       /* ชะลอตัวตอนเหยียบคลัทช์ / เกียร์ว่าง */
#define BRAKE_DECAY                0.080f       /* ชะลอตัวตอนเหยียบเบรก */

#define THROTTLE_IDLE_RAW_MAX        200u       /* Threshold คันเร่งเดินเบา */

/* --- UART TX Ring Buffer --- */
#define UART_TX_BUFFER_SIZE         128u

/* --- รหัสเกียร์ (Gear Codes) --- */
#define GEAR_NEUTRAL               0u
#define GEAR_1                     1u
#define GEAR_2                     2u
#define GEAR_3                     3u
#define GEAR_4                     4u
#define GEAR_5                     5u
#define GEAR_REVERSE               8u

/* ==================================================================== */
/* 2. GLOBAL VARIABLES (ตัวแปรโกลบอล)                                     */
/* ==================================================================== */

/* --- ADC Buffer (อัปเดตอัตโนมัติด้วย DMA2) --- */
volatile uint16_t g_adcBuffer[4] = {0u, 0u, 0u, 0u};
volatile uint16_t g_throttleRaw  = 0u;   /* CH0  - PA0 */
volatile uint16_t g_joyVrxRaw    = 0u;   /* CH12 - PC2 */
volatile uint16_t g_joyVryRaw    = 0u;   /* CH13 - PC3 */
volatile uint16_t g_ignitionRaw  = 0u;   /* CH4  - PA4 */

/* --- ระบบเวลา --- */
volatile uint32_t g_systemTickMs   = 0u;
volatile uint8_t  g_controlLoopFlag = 0u;

/* --- ค่า Center ของ Joystick --- */
int32_t g_centerX = 0;
int32_t g_centerY = 0;

/* --- สถานะระบบกุญแจ และ เครื่องยนต์ --- */
volatile uint8_t  g_keyInserted        = 0u;
volatile uint8_t  g_engineStarted      = 0u;
volatile uint8_t  g_engineStalled      = 0u;
volatile uint32_t g_lastKeyPressTickMs = 0u;

uint8_t  g_ledBlinkState    = 0u;
uint32_t g_blinkTickCounter = 0u;

/* --- สถานะสวิทช์คลัทช์ และ เบรก (พร้อม Software Debounce) --- */
volatile uint8_t g_clutchPressedLive = 0u;
volatile uint8_t g_brakePressedLive  = 0u;

uint8_t g_clutchPressed         = 0u;
uint8_t g_previousClutchPressed = 0u;
uint8_t g_brakePressed          = 0u;

/* --- สถานะเกียร์ และ ระบบขับเคลื่อน --- */
uint8_t g_currentGear  = GEAR_NEUTRAL;
uint8_t g_previousGear = GEAR_NEUTRAL;

float g_currentRpm      = 0.0f;
float g_currentVelocity = 0.0f;

/* --- ตัวนับเวลาสำหรับ Dashboard UART --- */
uint32_t g_uartReportTickCounter = 0u;

/* --- UART TX FIFO Ring Buffer --- */
typedef struct
{
    volatile uint8_t  data[UART_TX_BUFFER_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
    volatile uint16_t count;
} UartTxBuffer_t;

UartTxBuffer_t g_uartTx = { {0u}, 0u, 0u, 0u };

/* ==================================================================== */
/* 3. FUNCTION PROTOTYPES (ประกาศฟังก์ชัน)                                */
/* ==================================================================== */

/* Hardware Drivers Layer */
void Hardware_FPU_Enable(void);
void Hardware_GPIO_Init(void);
void Hardware_TIM2_Init(void);
void Hardware_ADC1_DMA_Init(void);
void Hardware_EXTI_Init(void);
void Hardware_UART2_Init(void);

/* Utility & System Setup */
void System_DelayMs(uint32_t ms);
void System_CalibrateJoystick(void);

/* Application Layer */
void App_ProcessInputDebounce(void);
void App_UpdateIgnitionSystem(void);
void App_UpdateStatusLed(void);
void App_UpdateGearAndDisplay(void);
void App_UpdateDrivetrain(void);
void App_LaunchCheck(uint8_t gear);
void App_TriggerStall(void);
void App_ReportDashboard(void);

uint8_t Driver_ReadClutchRaw(void);
uint8_t Driver_ReadBrakeRaw(void);
uint8_t App_ComputeCurrentGear(void);

/* Physics Calculation Helpers */
float Physics_ThrottleToRpm(uint16_t rawValue);
float Physics_GearRatioFor(uint8_t gearDigit);
float Physics_VelocityFromRpm(float rpm, float gearRatio);
float Physics_RpmFromVelocity(float velocity, float gearRatio);

/* Display & Communication Helpers */
void Driver_SegmentDisplay(uint8_t number);
void Driver_UART_SendChar(char c);
void Driver_UART_SendString(const char *text);
void Driver_UART_SendNumber(int32_t value);
void Driver_UART_SendFloatOneDecimal(float value);

/* ==================================================================== */
/* 4. MAIN PROGRAM                                                      */
/* ==================================================================== */

int main(void)
{
    /* 1. Initialize Hardware Peripherals */
    Hardware_FPU_Enable();
    Hardware_GPIO_Init();
    Hardware_ADC1_DMA_Init();
    Hardware_UART2_Init();
    Hardware_TIM2_Init();

    /* 2. Calibrate Sensors */
    System_CalibrateJoystick();

    /* 3. Initialize External Interrupts after calibration */
    Hardware_EXTI_Init();

    /* 4. Priming Initial States */
    g_currentGear = App_ComputeCurrentGear();
    g_previousGear = g_currentGear;
    g_clutchPressed = Driver_ReadClutchRaw();
    g_previousClutchPressed = g_clutchPressed;

    /* 5. Main Control Loop (Non-blocking, Timer Driven) */
    while (1)
    {
        if (g_controlLoopFlag == 1u)
        {
            g_controlLoopFlag = 0u;

            /* ดึงค่า ADC ล่าสุดจาก DMA Buffer */
            g_throttleRaw = g_adcBuffer[0];
            g_joyVrxRaw   = g_adcBuffer[1];
            g_joyVryRaw   = g_adcBuffer[2];
            g_ignitionRaw = g_adcBuffer[3];

            /* กรองสัญญาณปุ่มด้วย Software Debounce */
            App_ProcessInputDebounce();

            /* ทำงานตามระบบขับเคลื่อน */
            App_UpdateIgnitionSystem();
            App_UpdateStatusLed();

            /* ควบคุมไฟเบรก LED (PA6) */
            if (g_brakePressed == 1u)
            {
                GPIOA->ODR |= GPIO_ODR_OD6;
            }
            else
            {
                GPIOA->ODR &= ~(GPIO_ODR_OD6);
            }

            App_UpdateGearAndDisplay();
            App_UpdateDrivetrain();
            App_ReportDashboard();

            /* บันทึกค่าเพื่อใช้เปรียบเทียบใน Loop ถัดไป */
            g_previousGear = g_currentGear;
            g_previousClutchPressed = g_clutchPressed;
        }
    }
}

/* ==================================================================== */
/* 5. HARDWARE DRIVERS LAYER (ควบคุมอุปกรณ์ฮาร์ดแวร์)                       */
/* ==================================================================== */

void Hardware_FPU_Enable(void)
{
    SCB->CPACR |= ((3UL << (10u * 2u)) | (3UL << (11u * 2u)));
    __DSB();
    __ISB();
}

void Hardware_GPIO_Init(void)
{
    RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN);

    /* PA10: ปุ่มกุญแจ (Input Pull-up) */
    GPIOA->MODER &= ~(GPIO_MODER_MODER10);
    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD10);
    GPIOA->PUPDR |= (0b01 << GPIO_PUPDR_PUPD10_Pos);

    /* PB3: ปุ่มคลัทช์ (Input Pull-up) */
    GPIOB->MODER &= ~(GPIO_MODER_MODER3);
    GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD3);
    GPIOB->PUPDR |= (0b01 << GPIO_PUPDR_PUPD3_Pos);

    /* PB4: ปุ่มเบรก (Input Pull-up) */
    GPIOB->MODER &= ~(GPIO_MODER_MODER4);
    GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD4);
    GPIOB->PUPDR |= (0b01 << GPIO_PUPDR_PUPD4_Pos);

    /* PB6: LED เขียว (Output) */
    GPIOB->MODER &= ~(GPIO_MODER_MODER6);
    GPIOB->MODER |= (0b01 << GPIO_MODER_MODER6_Pos);

    /* PA7: LED เหลือง - ไฟถอยหลัง (Output) */
    GPIOA->MODER &= ~(GPIO_MODER_MODER7);
    GPIOA->MODER |= (0b01 << GPIO_MODER_MODER7_Pos);

    /* PA6: LED แดง - ไฟเบรก (Output) */
    GPIOA->MODER &= ~(GPIO_MODER_MODER6);
    GPIOA->MODER |= (0b01 << GPIO_MODER_MODER6_Pos);

    /* Analog Inputs (PA0, PA4, PC2, PC3) */
    GPIOA->MODER |= (0b11 << GPIO_MODER_MODER0_Pos) | (0b11 << GPIO_MODER_MODER4_Pos);
    GPIOC->MODER |= (0b11 << GPIO_MODER_MODER2_Pos) | (0b11 << GPIO_MODER_MODER3_Pos);

    /* 7-Segment Output Pins (PC7, PA8, PB10, PA9) */
    GPIOC->MODER &= ~(GPIO_MODER_MODER7);
    GPIOC->MODER |= (0b01 << GPIO_MODER_MODER7_Pos);

    GPIOA->MODER &= ~(GPIO_MODER_MODER8 | GPIO_MODER_MODER9);
    GPIOA->MODER |= (0b01 << GPIO_MODER_MODER8_Pos) | (0b01 << GPIO_MODER_MODER9_Pos);

    GPIOB->MODER &= ~(GPIO_MODER_MODER10);
    GPIOB->MODER |= (0b01 << GPIO_MODER_MODER10_Pos);

    /* PA2: USART2 TX (AF7) */
    GPIOA->MODER &= ~(GPIO_MODER_MODER2);
    GPIOA->MODER |= (0b10 << GPIO_MODER_MODER2_Pos);
    GPIOA->AFR[0] &= ~(0xFu << (2u * 4u));
    GPIOA->AFR[0] |= (7u << (2u * 4u));
}

void Hardware_TIM2_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    /* Timer 2: ตั้งจังหวะ 10ms (100Hz) @ HSI 16MHz */
    TIM2->PSC = 16000u - 1u;  /* 1 kHz count clock (1ms resolution) */
    TIM2->ARR = 10u - 1u;     /* Interrupt ทุกๆ 10ms */
    TIM2->CNT = 0u;

    TIM2->DIER |= TIM_DIER_UIE;
    NVIC_EnableIRQ(TIM2_IRQn);

    TIM2->CR1 |= TIM_CR1_CEN;
}

void Hardware_ADC1_DMA_Init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

    /* ตั้ง Sample Time สำหรับทุก Channel */
    ADC1->SMPR2 |= (ADC_SMPR2_SMP0 | ADC_SMPR2_SMP4);
    ADC1->SMPR1 |= (ADC_SMPR1_SMP12 | ADC_SMPR1_SMP13);

    /* ตั้งค่า Sequence Length = 4 Conversions (L = 3) */
    ADC1->SQR1 &= ~(ADC_SQR1_L);
    ADC1->SQR1 |= (3u << ADC_SQR1_L_Pos);

    /* ลำดับการอ่าน Scan: SQ1=PA0(0), SQ2=PC2(12), SQ3=PC3(13), SQ4=PA4(4) */
    ADC1->SQR3 &= ~(ADC_SQR3_SQ1 | ADC_SQR3_SQ2 | ADC_SQR3_SQ3 | ADC_SQR3_SQ4);
    ADC1->SQR3 |= (0u << ADC_SQR3_SQ1_Pos)  |
                  (12u << ADC_SQR3_SQ2_Pos) |
                  (13u << ADC_SQR3_SQ3_Pos) |
                  (4u << ADC_SQR3_SQ4_Pos);

    /* เปิดโหมด Scan และ DMA */
    ADC1->CR1 |= ADC_CR1_SCAN;
    ADC1->CR2 |= ADC_CR2_DMA | ADC_CR2_DDS | ADC_CR2_ADON;

    /* ตั้งค่า DMA2 Stream 0 Channel 0 สำหรับ ADC1 */
    DMA2_Stream0->CR &= ~DMA_SxCR_EN;
    while ((DMA2_Stream0->CR & DMA_SxCR_EN) != 0u) {}

    DMA2_Stream0->PAR  = (uint32_t)&(ADC1->DR);
    DMA2_Stream0->M0AR = (uint32_t)g_adcBuffer;
    DMA2_Stream0->NDTR = 4u;

    DMA2_Stream0->CR = (0u << DMA_SxCR_CHSEL_Pos)  | /* Channel 0 */
                       (1u << DMA_SxCR_PL_Pos)     | /* Priority High */
                       (1u << DMA_SxCR_MSIZE_Pos)  | /* 16-bit memory */
                       (1u << DMA_SxCR_PSIZE_Pos)  | /* 16-bit peripheral */
                       DMA_SxCR_MINC              | /* Memory increment */
                       DMA_SxCR_CIRC;               /* Circular mode */

    DMA2_Stream0->CR |= DMA_SxCR_EN;
}

void Hardware_EXTI_Init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    /* แมปขาเข้า EXTI Lines: PB3->EXTI3, PB4->EXTI4, PA10->EXTI10 */
    SYSCFG->EXTICR[0] &= ~(0xFu << 12u);
    SYSCFG->EXTICR[0] |= (0x1u << 12u);   /* PB3 */

    SYSCFG->EXTICR[1] &= ~(0xFu << 0u);
    SYSCFG->EXTICR[1] |= (0x1u << 0u);    /* PB4 */

    SYSCFG->EXTICR[2] &= ~(0xFu << 8u);
    SYSCFG->EXTICR[2] |= (0x0u << 8u);    /* PA10 */

    /* Trigger: PA10 (Falling Edge), PB3 & PB4 (Both Edges) */
    EXTI->FTSR |= (EXTI_FTSR_TR10 | EXTI_FTSR_TR3 | EXTI_FTSR_TR4);
    EXTI->RTSR |= (EXTI_RTSR_TR3 | EXTI_RTSR_TR4);

    /* อ่านค่าแรกเริ่ม */
    g_clutchPressedLive = Driver_ReadClutchRaw();
    g_brakePressedLive  = Driver_ReadBrakeRaw();

    EXTI->PR = (EXTI_PR_PR3 | EXTI_PR_PR4 | EXTI_PR_PR10);
    EXTI->IMR |= (EXTI_IMR_MR3 | EXTI_IMR_MR4 | EXTI_IMR_MR10);

    NVIC_EnableIRQ(EXTI3_IRQn);
    NVIC_EnableIRQ(EXTI4_IRQn);
    NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void Hardware_UART2_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    /* Baud Rate 9600 @ 16MHz */
    USART2->BRR = 0x683u;
    USART2->CR1 |= (USART_CR1_UE | USART_CR1_TE);

    NVIC_EnableIRQ(USART2_IRQn);
}

/* ==================================================================== */
/* 6. INTERRUPT SERVICE ROUTINES (ISRs)                                 */
/* ==================================================================== */

/* TIM2 Interrupt (ยิงทุก 10ms - Timebase หลัก) */
void TIM2_IRQHandler(void)
{
    if ((TIM2->SR & TIM_SR_UIF) != 0u)
    {
        TIM2->SR &= ~TIM_SR_UIF;

        g_systemTickMs += 10u;
        g_controlLoopFlag = 1u;

        /* สั่ง ADC1 Scan ทั้ง 4 ช่องทาง Hardware Trigger (No Polling) */
        ADC1->CR2 |= ADC_CR2_SWSTART;
    }
}

/* EXTI3 ISR (สวิทช์คลัทช์ PB3) */
void EXTI3_IRQHandler(void)
{
    if ((EXTI->PR & EXTI_PR_PR3) != 0u)
    {
        EXTI->PR = EXTI_PR_PR3;
        g_clutchPressedLive = Driver_ReadClutchRaw();
    }
}

/* EXTI4 ISR (สวิทช์เบรก PB4) */
void EXTI4_IRQHandler(void)
{
    if ((EXTI->PR & EXTI_PR_PR4) != 0u)
    {
        EXTI->PR = EXTI_PR_PR4;
        g_brakePressedLive = Driver_ReadBrakeRaw();
    }
}

/* EXTI15_10 ISR (ปุ่มกุญแจ PA10) */
void EXTI15_10_IRQHandler(void)
{
    if ((EXTI->PR & EXTI_PR_PR10) != 0u)
    {
        EXTI->PR = EXTI_PR_PR10;

        if (((g_systemTickMs - g_lastKeyPressTickMs) >= KEY_DEBOUNCE_MS) &&
            ((GPIOA->IDR & GPIO_IDR_ID10) == 0u))
        {
            g_lastKeyPressTickMs = g_systemTickMs;

            if (g_engineStalled == 0u)
            {
                if (g_keyInserted == 0u)
                {
                    g_keyInserted = 1u;
                }
                else if (g_engineStarted == 0u)
                {
                    g_keyInserted = 0u;
                }
            }
        }
    }
}

/* USART2 Interrupt (ส่งข้อมูลแบบ Ring Buffer Non-blocking) */
void USART2_IRQHandler(void)
{
    if (((USART2->SR & USART_SR_TXE) != 0u) && ((USART2->CR1 & USART_CR1_TXEIE) != 0u))
    {
        if (g_uartTx.count > 0u)
        {
            USART2->DR = g_uartTx.data[g_uartTx.tail];
            g_uartTx.tail = (uint16_t)((g_uartTx.tail + 1u) % UART_TX_BUFFER_SIZE);
            g_uartTx.count--;
        }
        else
        {
            USART2->CR1 &= ~USART_CR1_TXEIE;
        }
    }
}

/* ==================================================================== */
/* 7. APPLICATION LAYER & LOGIC (ระบบฟิสิกส์และการควบคุม)                    */
/* ==================================================================== */

/* Software Debounce Filter สำหรับสวิทช์ EXTI */
void App_ProcessInputDebounce(void)
{
    static uint8_t clutchDebounceCount = 0u;
    static uint8_t brakeDebounceCount  = 0u;

    uint8_t rawClutch = g_clutchPressedLive;
    uint8_t rawBrake  = g_brakePressedLive;

    if (rawClutch != g_clutchPressed)
    {
        clutchDebounceCount++;
        if (clutchDebounceCount >= 2u) /* เสถียรต่อเนื่อง 20ms */
        {
            g_clutchPressed = rawClutch;
            clutchDebounceCount = 0u;
        }
    }
    else
    {
        clutchDebounceCount = 0u;
    }

    if (rawBrake != g_brakePressed)
    {
        brakeDebounceCount++;
        if (brakeDebounceCount >= 2u)
        {
            g_brakePressed = rawBrake;
            brakeDebounceCount = 0u;
        }
    }
    else
    {
        brakeDebounceCount = 0u;
    }
}

void App_UpdateIgnitionSystem(void)
{
    if (g_engineStalled == 1u)
    {
        if (g_ignitionRaw < HALF_ADC_VALUE)
        {
            g_engineStalled = 0u;
        }
        return;
    }

    if (g_keyInserted == 1u)
    {
        if (g_ignitionRaw >= HALF_ADC_VALUE)
        {
            g_engineStarted = 1u;
        }
        else
        {
            g_engineStarted = 0u;
        }
    }
    else
    {
        g_engineStarted = 0u;
    }
}

void App_UpdateStatusLed(void)
{
    uint32_t blinkTicks = (g_engineStalled == 1u) ? BLINK_STALL_TICKS : BLINK_NORMAL_TICKS;

    if (g_engineStalled == 1u || (g_keyInserted == 1u && g_engineStarted == 0u))
    {
        g_blinkTickCounter++;
        if (g_blinkTickCounter >= blinkTicks)
        {
            g_blinkTickCounter = 0u;
            g_ledBlinkState ^= 1u;

            if (g_ledBlinkState == 1u)
            {
                GPIOB->ODR |= GPIO_ODR_OD6;
            }
            else
            {
                GPIOB->ODR &= ~(GPIO_ODR_OD6);
            }
        }
    }
    else if (g_engineStarted == 1u)
    {
        GPIOB->ODR |= GPIO_ODR_OD6;
        g_blinkTickCounter = 0u;
    }
    else
    {
        GPIOB->ODR &= ~(GPIO_ODR_OD6);
        g_blinkTickCounter = 0u;
    }
}

uint8_t App_ComputeCurrentGear(void)
{
    int32_t offsetX = (int32_t)g_joyVrxRaw - g_centerX;
    int32_t offsetY = (int32_t)g_joyVryRaw - g_centerY;

    if ((offsetX >= -2500 && offsetX <= 120 && offsetY >= -2500 && offsetY <= 120) ||
        (offsetX > 120 && offsetY >= -2500 && offsetY <= 120) ||
        (offsetX < -2500 && offsetY >= -2500 && offsetY <= 120)) {
        return GEAR_NEUTRAL;
    }
    else if (offsetX < -2500 && offsetY < -2500) { return GEAR_1; }
    else if (offsetX < -2500 && offsetY > 120)   { return GEAR_2; }
    else if (offsetX >= -2500 && offsetX <= 120 && offsetY < -2500) { return GEAR_3; }
    else if (offsetX >= -2500 && offsetX <= 120 && offsetY > 120)   { return GEAR_4; }
    else if (offsetX > 120 && offsetY < -2500) { return GEAR_5; }
    else if (offsetX > 120 && offsetY > 120)   { return GEAR_REVERSE; }

    return GEAR_NEUTRAL;
}

void App_UpdateGearAndDisplay(void)
{
    g_currentGear = App_ComputeCurrentGear();
    Driver_SegmentDisplay(g_currentGear);

    if (g_currentGear == GEAR_REVERSE)
    {
        GPIOA->ODR |= GPIO_ODR_OD7;
    }
    else
    {
        GPIOA->ODR &= ~(GPIO_ODR_OD7);
    }
}

/* คำนวณฟิสิกส์การขับขี่และระบบส่งกำลัง */
void App_UpdateDrivetrain(void)
{
    float currentGearRatio = Physics_GearRatioFor(g_currentGear);
    float throttleRpm      = Physics_ThrottleToRpm(g_throttleRaw);
    float rpmFromVel       = 0.0f;

    /* 1. กรณีเครื่องยนต์ดับอยู่ */
    if (g_engineStarted == 0u)
    {
        g_currentRpm = 0.0f;
        if (g_currentVelocity > COAST_DECAY)
        {
            g_currentVelocity -= COAST_DECAY;
        }
        else
        {
            g_currentVelocity = 0.0f;
        }
        return;
    }

    /* 2. เปลี่ยนเกียร์โดยไม่เหยียบคลัทช์ -> เครื่องดับทันที */
    if ((g_currentGear != g_previousGear) && (g_clutchPressed == 0u))
    {
        App_TriggerStall();
        return;
    }

    /* 3. สภาวะตัดกำลัง (Power Disconnected Sequence): เหยียบคลัทช์ หรือ อยู่เกียร์ว่าง */
    if ((g_clutchPressed == 1u) || (g_currentGear == GEAR_NEUTRAL))
    {
        /* รอบเครื่องเร่งฟรีตามตำแหน่งคันเร่ง */
        g_currentRpm = throttleRpm;

        /* ชะลอความเร็วแบบ Coasting (ไม่เช็คสภาวะเครื่องดับ) */
        float decayRate = (g_brakePressed == 1u) ? BRAKE_DECAY : COAST_DECAY;

        if (g_currentVelocity > decayRate)
        {
            g_currentVelocity -= decayRate;
        }
        else
        {
            g_currentVelocity = 0.0f;
        }
        return;
    }

    /* 4. สภาวะต่อเกียร์ขับขี่ (In-Gear Driving) */

    /* 4a. จังหวะเพิ่งปล่อยคลัทช์ (Clutch Re-engagement Edge) */
    if (g_previousClutchPressed == 1u)
    {
        /* ตรวจสอบ LaunchCheck เฉพาะตอนความเร็วหยุดนิ่งใกล้ศูนย์ (< 0.5 km/h) */
        if (g_currentVelocity < 0.5f)
        {
            App_LaunchCheck(g_currentGear);
        }
        else
        {
            /* ถ้ารถกำลังไหลอยู่ ให้ทำ Rev-Sync ปรับรอบเครื่องตามความเร็วล้อโดยไม่ดับ */
            if ((g_previousGear == GEAR_REVERSE) && (g_currentGear == GEAR_1))
            {
                App_TriggerStall(); /* ถอยหลังอยู่แล้วใส่เกียร์ 1 ทันที -> ดับ */
            }
            else
            {
                rpmFromVel = Physics_RpmFromVelocity(g_currentVelocity, currentGearRatio);

                if (rpmFromVel >= STALL_RPM_THRESHOLD)
                {
                    g_currentRpm = rpmFromVel;
                }
                else if ((g_throttleRaw > THROTTLE_IDLE_RAW_MAX) && (throttleRpm >= IDLE_RPM))
                {
                    g_currentRpm = throttleRpm;
                    g_currentVelocity = Physics_VelocityFromRpm(g_currentRpm, currentGearRatio);
                }
                else
                {
                    App_TriggerStall();
                }
            }
        }
        return;
    }

    /* 4b. ขับขี่ต่อเนื่องปกติ */
    if (g_brakePressed == 1u)
    {
        if (g_currentVelocity > BRAKE_DECAY)
        {
            g_currentVelocity -= BRAKE_DECAY;
        }
        else
        {
            g_currentVelocity = 0.0f;
        }

        g_currentRpm = Physics_RpmFromVelocity(g_currentVelocity, currentGearRatio);

        if (g_currentRpm < STALL_RPM_THRESHOLD)
        {
            App_TriggerStall();
        }
        return;
    }

    /* การเหยียบเร่ง หรือ การชะลอแบบ Engine Brake */
    if (throttleRpm >= g_currentRpm)
    {
        /* เร่งเครื่อง: รอบเครื่องควบคุมความเร็ว */
        g_currentRpm = throttleRpm;
        g_currentVelocity = Physics_VelocityFromRpm(g_currentRpm, currentGearRatio);
    }
    else
    {
        /* ผ่อนคันเร่ง: Engine Brake แบบ Linear Scaling */
        float effectiveDecay = BASE_DECAY * currentGearRatio;

        if (g_currentVelocity > effectiveDecay)
        {
            g_currentVelocity -= effectiveDecay;
        }
        else
        {
            g_currentVelocity = 0.0f;
        }

        g_currentRpm = Physics_RpmFromVelocity(g_currentVelocity, currentGearRatio);

        if (g_currentRpm < STALL_RPM_THRESHOLD)
        {
            App_TriggerStall();
        }
    }
}

void App_LaunchCheck(uint8_t gear)
{
    float throttleRpm      = Physics_ThrottleToRpm(g_throttleRaw);
    float currentGearRatio = Physics_GearRatioFor(gear);

    if ((gear == GEAR_1) || (gear == GEAR_REVERSE))
    {
        g_currentRpm = (g_throttleRaw > THROTTLE_IDLE_RAW_MAX) ? throttleRpm : IDLE_RPM;
        g_currentVelocity = Physics_VelocityFromRpm(g_currentRpm, currentGearRatio);
    }
    else if (gear == GEAR_2)
    {
        if (throttleRpm > GEAR2_LAUNCH_RPM_MIN)
        {
            g_currentRpm = throttleRpm;
            g_currentVelocity = Physics_VelocityFromRpm(g_currentRpm, currentGearRatio);
        }
        else
        {
            App_TriggerStall();
        }
    }
    else
    {
        /* เกียร์ 3, 4, 5 ออกตัวจากจุดหยุดนิ่งไม่ได้ -> ดับ */
        App_TriggerStall();
    }
}

void App_TriggerStall(void)
{
    g_engineStalled = 1u;
    g_engineStarted = 0u;
    g_currentRpm    = 0.0f;
}

void App_ReportDashboard(void)
{
    g_uartReportTickCounter++;

    if (g_uartReportTickCounter >= UART_REPORT_PERIOD_TICKS)
    {
        g_uartReportTickCounter = 0u;

        /* ส่งข้อมูล CSV: <RPM>,<VELOCITY>\r\n */
        Driver_UART_SendFloatOneDecimal(g_currentRpm);
        Driver_UART_SendChar(',');
        Driver_UART_SendFloatOneDecimal(g_currentVelocity);
        Driver_UART_SendString("\r\n");
    }
}

/* ==================================================================== */
/* 8. HELPERS & UTILITIES (สูตรคำนวณและฟังก์ชันช่วย)                          */
/* ==================================================================== */

uint8_t Driver_ReadClutchRaw(void)
{
    return ((GPIOB->IDR & GPIO_IDR_ID3) == 0u) ? 1u : 0u;
}

uint8_t Driver_ReadBrakeRaw(void)
{
    return ((GPIOB->IDR & GPIO_IDR_ID4) == 0u) ? 1u : 0u;
}

float Physics_ThrottleToRpm(uint16_t rawValue)
{
    float ratio = (float)rawValue / 4095.0f;
    return IDLE_RPM + (ratio * (REDLINE_RPM - IDLE_RPM));
}

float Physics_GearRatioFor(uint8_t gearDigit)
{
    switch (gearDigit)
    {
        case GEAR_1:       return 3.928f;
        case GEAR_2:       return 2.333f;
        case GEAR_3:       return 1.451f;
        case GEAR_4:       return 1.000f;
        case GEAR_5:       return 0.851f;
        case GEAR_REVERSE: return 4.743f;
        default:           return 1.000f;
    }
}

float Physics_VelocityFromRpm(float rpm, float gearRatio)
{
    return (rpm * 0.36f) / (2.6526f * gearRatio * FINAL_DRIVE);
}

float Physics_RpmFromVelocity(float velocity, float gearRatio)
{
    return (velocity * 2.6526f * gearRatio * FINAL_DRIVE) / 0.36f;
}

void Driver_SegmentDisplay(uint8_t number)
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

void System_DelayMs(uint32_t ms)
{
    uint32_t startTick = g_systemTickMs;
    while ((g_systemTickMs - startTick) < ms) {}
}

void System_CalibrateJoystick(void)
{
    System_DelayMs(500u);
    g_centerX = (int32_t)g_adcBuffer[1];
    g_centerY = (int32_t)g_adcBuffer[2];
}

void Driver_UART_SendChar(char c)
{
    if (g_uartTx.count < UART_TX_BUFFER_SIZE)
    {
        USART2->CR1 &= ~USART_CR1_TXEIE;

        g_uartTx.data[g_uartTx.head] = (uint8_t)c;
        g_uartTx.head = (uint16_t)((g_uartTx.head + 1u) % UART_TX_BUFFER_SIZE);
        g_uartTx.count++;

        USART2->CR1 |= USART_CR1_TXEIE;
    }
}

void Driver_UART_SendString(const char *text)
{
    uint32_t i = 0u;
    while (text[i] != '\0')
    {
        Driver_UART_SendChar(text[i]);
        i++;
    }
}

void Driver_UART_SendNumber(int32_t value)
{
    char digits[12];
    uint32_t magnitude = (value < 0) ? (uint32_t)(-value) : (uint32_t)value;
    uint8_t digitCount = 0u;

    if (value < 0)
    {
        Driver_UART_SendChar('-');
    }

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
            magnitude /= 10u;
            digitCount++;
        }
    }

    while (digitCount > 0u)
    {
        digitCount--;
        Driver_UART_SendChar(digits[digitCount]);
    }
}

void Driver_UART_SendFloatOneDecimal(float value)
{
    if (value < 0.0f)
    {
        Driver_UART_SendChar('-');
        value = -value;
    }

    int32_t tenths = (int32_t)((value * 10.0f) + 0.5f);
    int32_t wholePart = tenths / 10;
    tenths %= 10;

    Driver_UART_SendNumber(wholePart);
    Driver_UART_SendChar('.');
    Driver_UART_SendNumber(tenths);
}