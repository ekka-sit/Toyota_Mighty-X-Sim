/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Toyota Mighty-X (2L) Full Driving Simulator - STM32F411RE
 ******************************************************************************
 *
 * ระบบทั้งหมดขับเคลื่อนด้วย Interrupt ล้วนๆ ไม่มี Polling หรือ Blocking Delay
 * อยู่ใน main loop เลย:
 *
 *   - ADC (คันเร่ง, Joystick x2, Potentiometer สตาร์ท) อ่านแบบ Round-Robin
 *     ผ่าน ADC_IRQHandler เท่านั้น ไม่มีการ "รอ EOC" ที่ไหนเลย
 *   - จังหวะเวลา (Timebase) ของระบบทั้งหมดมาจาก SysTick interrupt (ติ๊กทุก 1ms)
 *     main loop แค่เช็ค flag ที่ SysTick ตั้งไว้ (ทุก 10ms) แล้วเรียกฟังก์ชัน
 *     ควบคุมหลัก - ไม่มีการวนรอฮาร์ดแวร์ตรงๆ
 *   - ข้อยกเว้นเดียว: การ "รอ" ตอน Calibrate joystick ตอนเปิดเครื่อง ซึ่งเกิด
 *     ก่อนเข้า while(1) หลัก (ไม่ถือว่าเป็นการ Poll ใน Main Loop)
 *
 * ------------------------------------------------------------------
 * ตำแหน่งขา (Pin Assignment)
 * ------------------------------------------------------------------
 *  PA10 = ปุ่มเสียบ/ถอดกุญแจ            PC2  = Joystick VRx (ADC1_IN12)
 *  PA4  = Potentiometer สตาร์ท (ADC1_IN4) PC3  = Joystick VRy (ADC1_IN13)
 *  PB6  = LED เขียว (กุญแจ/เครื่องยนต์)   PC7,PA8,PB10,PA9 = 7-Segment (BCD)
 *  PA0  = Potentiometer คันเร่ง (ADC1_IN0) PB3 = ปุ่มคลัทช์
 *  PA7  = LED เหลือง (ไฟถอยหลัง)          PB4 = ปุ่มเบรก
 *  PA6  = LED แดง (ไฟเบรก)                PA2 = USART2 TX (Dashboard)
 * ------------------------------------------------------------------
 */

#include <stdint.h>
#define	STM32F411xE
#include "stm32f4xx.h"

/* ==================================================================== */
/* ค่าคงที่ทั้งหมดของระบบ (Settings)                                      */
/* ==================================================================== */

/* --- จังหวะเวลา --- */
#define SYSTICK_RELOAD_1MS        (16000u - 1u)  /* สมมติ core clock = HSI 16MHz */
#define CONTROL_LOOP_PERIOD_MS       10u          /* ระบบควบคุมหลักทำงานทุกกี่ ms */
#define UART_REPORT_PERIOD_TICKS     20u          /* พิมพ์ dashboard ทุกกี่ tick (20*10ms=200ms) */

/* --- กุญแจ/สตาร์ท --- */
#define HALF_ADC_VALUE             2048u
#define BLINK_NORMAL_TICKS           50u   /* กระพริบปกติ: ครึ่งคาบ = 50*10ms = 500ms */
#define BLINK_STALL_TICKS            10u   /* กระพริบถี่ตอนดับผิดปกติ: 10*10ms = 100ms */

/* --- เครื่องยนต์/ความเร็ว --- */
#define IDLE_RPM                   800.0f
#define REDLINE_RPM               4200.0f
#define STALL_RPM_THRESHOLD        600.0f   /* 75% ของรอบเดินเบา */
#define GEAR2_LAUNCH_RPM_MIN      1200.0f
#define FINAL_DRIVE                4.300f

#define VELOCITY_DECAY_CLUTCH        1.5f   /* km/hr ต่อ tick ตอนเหยียบคลัทช์ */
#define VELOCITY_DECAY_ENGINEBRAKE   4.0f   /* km/hr ต่อ tick ตอนถอนคันเร่ง/เครื่องดับ */
#define VELOCITY_DECAY_BRAKE        10.0f   /* km/hr ต่อ tick ตอนเหยียบเบรก */

#define THROTTLE_IDLE_RAW_MAX        200u   /* ต่ำกว่านี้ถือว่า "ไม่ได้เหยียบคันเร่ง" */

/* --- ช่อง ADC (Round-Robin ตามลำดับที่ spec กำหนด) --- */
#define ADC_CHANNEL_THROTTLE          0u   /* PA0 */
#define ADC_CHANNEL_JOY_VRX          12u   /* PC2 */
#define ADC_CHANNEL_JOY_VRY          13u   /* PC3 */
#define ADC_CHANNEL_IGNITION          4u   /* PA4 */

/* --- เกียร์ (เลขตรงกับที่โชว์บน 7-segment เลย) --- */
#define GEAR_NEUTRAL   0u
#define GEAR_1         1u
#define GEAR_2         2u
#define GEAR_3         3u
#define GEAR_4         4u
#define GEAR_5         5u
#define GEAR_REVERSE   8u

/* ==================================================================== */
/* ตัวแปร Global                                                          */
/* ==================================================================== */

/* --- อัปเดตโดย ADC_IRQHandler (background, round-robin 4 ช่อง) --- */
volatile uint16_t g_throttleRaw = 0u;
volatile uint16_t g_joyVrxRaw = 0u;
volatile uint16_t g_joyVryRaw = 0u;
volatile uint16_t g_ignitionRaw = 0u;
volatile uint8_t g_adcChannelIndex = 0u; /* 0=throttle,1=VRx,2=VRy,3=ignition */

/* --- อัปเดตโดย SysTick_Handler --- */
volatile uint32_t g_systemTickMs = 0u;
volatile uint8_t g_controlLoopFlag = 0u;

/* --- จุดกลางของ joystick (Calibrate ตอนเปิดเครื่องครั้งเดียว) --- */
int32_t g_centerX = 0;
int32_t g_centerY = 0;

/* --- สถานะระบบกุญแจ/เครื่องยนต์ --- */
uint8_t g_keyInserted = 0u;
uint8_t g_engineStarted = 0u;
uint8_t g_engineStalled = 0u;
uint8_t g_keyButtonLatched = 0u;
uint8_t g_ledBlinkState = 0u;
uint32_t g_blinkTickCounter = 0u;

/* --- สถานะปุ่มคลัทช์/เบรก --- */
uint8_t g_clutchPressed = 0u;
uint8_t g_previousClutchPressed = 0u;
uint8_t g_brakePressed = 0u;

/* --- สถานะเกียร์ --- */
uint8_t g_currentGear = GEAR_NEUTRAL;
uint8_t g_previousGear = GEAR_NEUTRAL;

/* --- สถานะขับเคลื่อน --- */
float g_currentRpm = 0.0f;
float g_currentVelocity = 0.0f;

/* --- ตัวนับสำหรับพิมพ์ dashboard --- */
uint32_t g_uartReportTickCounter = 0u;

/* ==================================================================== */
/* Prototype ฟังก์ชัน                                                      */
/* ==================================================================== */

void GPIO_Init(void);
void ADC_Init(void);
void ADC_StartChannel(uint8_t channel);
void SysTick_Init(void);
void FPU_Enable(void);
void WaitStartupMs(uint32_t ms);
void CalibrateJoystickCenter(void);

uint8_t ReadClutchButtonRaw(void);
uint8_t ReadBrakeButtonRaw(void);
uint8_t ComputeCurrentGear(void);

void RunControlLoop(void);
void UpdateIgnitionSystem(void);
void UpdateStatusLed(void);
void UpdateGearAndDisplay(void);
void UpdateDrivetrain(void);
void LaunchCheck(uint8_t gear);
void TriggerStall(void);
void ReportDashboard(void);

float ThrottleToRpm(uint16_t rawValue);
float GearRatioFor(uint8_t gearDigit);
float VelocityFromRpm(float rpm, float gearRatio);
float RpmFromVelocity(float velocity, float gearRatio);

void Segment_Display(uint8_t number);
void UART_Init(void);
void UART_SendChar(char c);
void UART_SendString(const char *text);
void UART_SendNumber(int32_t value);
void UART_SendFloatOneDecimal(float value);

/* ==================================================================== */
/* main                                                                    */
/* ==================================================================== */

int main(void)
{
    FPU_Enable();

    GPIO_Init();
    ADC_Init();
    UART_Init();
    SysTick_Init();

    /* Calibrate joystick center ก่อนเริ่มระบบ (ไม่มีข้อความ debug ออกทาง UART
       แล้ว เพื่อให้สาย UART ส่งเฉพาะ CSV data frame ล้วนๆ ตามที่ต้องการ) */
    CalibrateJoystickCenter();

    /* Priming: อ่านค่าตั้งต้นก่อนเข้า loop กันไม่ให้ tick แรกเข้าใจผิดว่า
       "เพิ่งเปลี่ยนเกียร์" หรือ "เพิ่งปล่อยคลัทช์" ทั้งที่ยังไม่ได้ทำอะไรเลย */
    g_currentGear = ComputeCurrentGear();
    g_previousGear = g_currentGear;
    g_clutchPressed = ReadClutchButtonRaw();
    g_previousClutchPressed = g_clutchPressed;

    while (1)
    {
        if (g_controlLoopFlag == 1u)
        {
            g_controlLoopFlag = 0u;
            RunControlLoop();
        }
        else
        {
            /* ยังไม่ถึงจังหวะ ไม่ต้องทำอะไร - งานทั้งหมดขับเคลื่อนด้วย
               interrupt อยู่แล้ว (ADC และ SysTick) การเช็ค flag เฉยๆ แบบนี้
               ไม่ใช่การ Poll ฮาร์ดแวร์ตามที่ spec ห้าม */
        }
    }
}

/* ==================================================================== */
/* ฟังก์ชันควบคุมหลัก เรียกทุก CONTROL_LOOP_PERIOD_MS                       */
/* ==================================================================== */

void RunControlLoop(void)
{
    UpdateIgnitionSystem();
    UpdateStatusLed();

    g_clutchPressed = ReadClutchButtonRaw();
    g_brakePressed = ReadBrakeButtonRaw();

    if (g_brakePressed == 1u)
    {
        GPIOA->ODR |= GPIO_ODR_OD6;
    }
    else
    {
        GPIOA->ODR &= ~(GPIO_ODR_OD6);
    }

    UpdateGearAndDisplay();
    UpdateDrivetrain();
    ReportDashboard();

    /* เก็บค่าของ tick นี้ไว้เทียบใน tick ถัดไป */
    g_previousGear = g_currentGear;
    g_previousClutchPressed = g_clutchPressed;
}

/* ==================================================================== */
/* 1) ระบบกุญแจ/สตาร์ท + ระบบฟื้นคืนจากเครื่องดับผิดปกติ                     */
/* ==================================================================== */

void UpdateIgnitionSystem(void)
{
    /* ตอนเครื่องดับผิดปกติ: ล็อกปุ่มกุญแจทั้งหมด รอแค่หมุน pot กลับต่ำกว่าครึ่ง */
    if (g_engineStalled == 1u)
    {
        if (g_ignitionRaw < HALF_ADC_VALUE)
        {
            g_engineStalled = 0u; /* เคลียร์สถานะ กลับเข้าสู่ระบบสตาร์ทปกติ */
        }
        else
        {
            /* ยังไม่เคลียร์ ห้ามทำอะไรกับปุ่มกุญแจ/สตาร์ท */
        }

        return;
    }

    /* --- ปุ่มเสียบ/ถอดกุญแจ (PA10), กด = pin อ่านได้ 0 --- */
    if ((GPIOA->IDR & GPIO_IDR_ID10) == 0u)
    {
        if (g_keyButtonLatched == 0u)
        {
            g_keyButtonLatched = 1u;

            if (g_keyInserted == 1u)
            {
                if (g_engineStarted == 0u)
                {
                    /* ถอดกุญแจได้เฉพาะตอนไฟกระพริบ (เครื่องไม่ได้สตาร์ท) */
                    g_keyInserted = 0u;
                }
                else
                {
                    /* เครื่องกำลังติดอยู่: กุญแจล็อก กดไม่มีผล */
                }
            }
            else
            {
                g_keyInserted = 1u;
            }
        }
        else
        {
            /* ยังกดค้างอยู่ ไม่ทำอะไรซ้ำ */
        }
    }
    else
    {
        g_keyButtonLatched = 0u;
    }

    /* --- เช็ค potentiometer สตาร์ท --- */
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

/* ==================================================================== */
/* 2) ไฟ LED เขียว (PB6) แสดงสถานะกุญแจ/เครื่องยนต์/สตอล                    */
/* ==================================================================== */

void UpdateStatusLed(void)
{
    uint32_t blinkPeriodTicks;

    if (g_engineStalled == 1u)
    {
        blinkPeriodTicks = BLINK_STALL_TICKS;
    }
    else
    {
        blinkPeriodTicks = BLINK_NORMAL_TICKS;
    }

    if (g_engineStalled == 1u)
    {
        /* กระพริบถี่เตือนเครื่องดับผิดปกติ */
        g_blinkTickCounter = g_blinkTickCounter + 1u;

        if (g_blinkTickCounter >= blinkPeriodTicks)
        {
            g_blinkTickCounter = 0u;

            if (g_ledBlinkState == 0u)
            {
                g_ledBlinkState = 1u;
                GPIOB->ODR |= GPIO_ODR_OD6;
            }
            else
            {
                g_ledBlinkState = 0u;
                GPIOB->ODR &= ~(GPIO_ODR_OD6);
            }
        }
        else
        {
            /* ยังไม่ถึงจังหวะสลับ */
        }
    }
    else if (g_engineStarted == 1u)
    {
        GPIOB->ODR |= GPIO_ODR_OD6;
        g_blinkTickCounter = 0u;
    }
    else if (g_keyInserted == 1u)
    {
        g_blinkTickCounter = g_blinkTickCounter + 1u;

        if (g_blinkTickCounter >= blinkPeriodTicks)
        {
            g_blinkTickCounter = 0u;

            if (g_ledBlinkState == 0u)
            {
                g_ledBlinkState = 1u;
                GPIOB->ODR |= GPIO_ODR_OD6;
            }
            else
            {
                g_ledBlinkState = 0u;
                GPIOB->ODR &= ~(GPIO_ODR_OD6);
            }
        }
        else
        {
            /* ยังไม่ถึงจังหวะสลับ */
        }
    }
    else
    {
        GPIOB->ODR &= ~(GPIO_ODR_OD6);
        g_ledBlinkState = 0u;
        g_blinkTickCounter = 0u;
    }
}

/* ==================================================================== */
/* 3) ปุ่มคลัทช์ / เบรก (อ่านแบบ level ตรงๆ ไม่ latch เพราะเป็นปุ่มกดค้าง)     */
/* ==================================================================== */

uint8_t ReadClutchButtonRaw(void)
{
    uint8_t pressed;

    if ((GPIOB->IDR & GPIO_IDR_ID3) == 0u)
    {
        pressed = 1u;
    }
    else
    {
        pressed = 0u;
    }

    return pressed;
}

uint8_t ReadBrakeButtonRaw(void)
{
    uint8_t pressed;

    if ((GPIOB->IDR & GPIO_IDR_ID4) == 0u)
    {
        pressed = 1u;
    }
    else
    {
        pressed = 0u;
    }

    return pressed;
}

/* ==================================================================== */
/* 4) ระบบเกียร์ (ตรรกะเดิมจาก Gear.c ไม่แก้ไข) + แสดงผล 7-segment/ไฟถอย    */
/* ==================================================================== */

uint8_t ComputeCurrentGear(void)
{
    int32_t offsetX;
    int32_t offsetY;
    uint8_t gearResult;

    offsetX = (int32_t)g_joyVrxRaw - g_centerX;
    offsetY = (int32_t)g_joyVryRaw - g_centerY;

    /* --- ตรรกะเดิมจาก Gear.c เป๊ะๆ --- */
    if ((offsetX >= -2500 && offsetX <= 120 && offsetY >= -2500 && offsetY <= 120) ||
        (offsetX > 120 && offsetY >= -2500 && offsetY <= 120) ||
        (offsetX < -2500 && offsetY >= -2500 && offsetY <= 120)) {
        gearResult = GEAR_NEUTRAL;
    }
    else if (offsetX < -2500 && offsetY < -2500) {
        gearResult = GEAR_1;
    }
    else if (offsetX < -2500 && offsetY > 120) {
        gearResult = GEAR_2;
    }
    else if (offsetX >= -2500 && offsetX <= 120 && offsetY < -2500) {
        gearResult = GEAR_3;
    }
    else if (offsetX >= -2500 && offsetX <= 120 && offsetY > 120) {
        gearResult = GEAR_4;
    }
    else if (offsetX > 120 && offsetY < -2500) {
        gearResult = GEAR_5;
    }
    else if (offsetX > 120 && offsetY > 120) {
        gearResult = GEAR_REVERSE;
    }
    else {
        gearResult = GEAR_NEUTRAL;
    }

    return gearResult;
}

void UpdateGearAndDisplay(void)
{
    g_currentGear = ComputeCurrentGear();

    /* 7-segment โชว์เลขเกียร์ตรงๆ (0=N, 1-5, 8=R) */
    Segment_Display(g_currentGear);

    /* ไฟเหลืองถอยหลัง (PA7) */
    if (g_currentGear == GEAR_REVERSE)
    {
        GPIOA->ODR |= GPIO_ODR_OD7;
    }
    else
    {
        GPIOA->ODR &= ~(GPIO_ODR_OD7);
    }
}

/* ==================================================================== */
/* 5) ระบบขับเคลื่อน: คันเร่ง, คลัทช์, เบรก, Crawling, Stall                 */
/* ==================================================================== */

void UpdateDrivetrain(void)
{
    float currentGearRatio;
    float throttleRpm;
    float targetVelocity;
    float decayAmount;

    if (g_engineStarted == 0u)
    {
        /* เครื่องไม่ทำงาน (ยังไม่สตาร์ท/ดับ/สตอล): ไม่มีรอบเครื่อง
           รถไหลต่อด้วยแรงเสียดทานธรรมชาติจนหยุด (ไม่ใช่หยุดกึกทันที) */
        g_currentRpm = 0.0f;

        if (g_currentVelocity > VELOCITY_DECAY_ENGINEBRAKE)
        {
            g_currentVelocity = g_currentVelocity - VELOCITY_DECAY_ENGINEBRAKE;
        }
        else
        {
            g_currentVelocity = 0.0f;
        }

        return;
    }

    if ((g_currentGear != g_previousGear) && (g_clutchPressed == 0u))
    {
        /* เปลี่ยนเกียร์โดยไม่ได้เหยียบคลัทช์ -> เครื่องดับทันที */
        TriggerStall();
        return;
    }

    if (g_currentGear == GEAR_NEUTRAL)
    {
        /* เกียร์ว่าง: เครื่องฟรีตามคันเร่งเสมอ (ทุกเกียร์รวมเกียร์ว่างต้องตอบสนองคันเร่ง) */
        g_currentRpm = ThrottleToRpm(g_throttleRaw);

        if (g_currentVelocity > VELOCITY_DECAY_ENGINEBRAKE)
        {
            g_currentVelocity = g_currentVelocity - VELOCITY_DECAY_ENGINEBRAKE;
        }
        else
        {
            g_currentVelocity = 0.0f;
        }

        return;
    }

    /* --- เข้าเกียร์อยู่ (1-5 หรือ R) --- */
    currentGearRatio = GearRatioFor(g_currentGear);

    if (g_clutchPressed == 1u)
    {
        /* เหยียบคลัทช์ค้าง: เครื่องตัดออกจากระบบส่งกำลัง รอบเครื่องจึงตอบสนอง
           ตามตำแหน่งคันเร่งตรงๆ เหมือนตอนอยู่เกียร์ว่าง (เบิ้ลเครื่องได้ถ้าเหยียบ
           คันเร่ง, ถอนคันเร่งแล้วรอบเครื่องจะไหลกลับลงมาที่รอบเดินเบาเอง เพราะ
           ThrottleToRpm() คืนค่า IDLE_RPM พอดีตอนคันเร่งอยู่ตำแหน่งต่ำสุด) */
        g_currentRpm = ThrottleToRpm(g_throttleRaw);

        /* ความเร็วไหลลงช้าๆ (Clutch Coasting) เว้นแต่กำลังเหยียบเบรกด้วย
           ซึ่งเบรกจะแรงกว่าและมีสิทธิ์เหนือกว่า */
        if (g_brakePressed == 1u)
        {
            decayAmount = VELOCITY_DECAY_BRAKE;
        }
        else
        {
            decayAmount = VELOCITY_DECAY_CLUTCH;
        }

        if (g_currentVelocity > decayAmount)
        {
            g_currentVelocity = g_currentVelocity - decayAmount;
        }
        else
        {
            g_currentVelocity = 0.0f;
        }

        return;
    }

    /* --- ปล่อยคลัทช์ ขับขี่ในเกียร์ --- */

    if (g_previousClutchPressed == 1u)
    {
        /* จังหวะเพิ่งปล่อยคลัทช์ (Edge) */
        if (g_currentVelocity == 0.0f)
        {
            /* กำลังออกตัวจากจุดหยุดนิ่ง */
            LaunchCheck(g_currentGear);
        }
        else
        {
            /* ปล่อยคลัทช์ขณะรถยังไหลอยู่ (เช่น เปลี่ยนเกียร์ขณะขับ) */
            if ((g_previousGear == GEAR_REVERSE) && (g_currentGear == GEAR_1))
            {
                /* เงื่อนไขความปลอดภัย: ยังถอยหลังไม่หยุดสนิทแล้วปล่อยคลัทช์เข้าเกียร์ 1 -> ดับ */
                TriggerStall();
            }
            else
            {
                /* คลัทช์ดีด/จับกลับ (Clutch Rebound): รอบเครื่องปรับมาจับกับ
                   ความเร็วล้อปัจจุบันทันที (ความเร็วคุมรอบในจังหวะนี้) */
                g_currentRpm = RpmFromVelocity(g_currentVelocity, currentGearRatio);
            }
        }

        return;
    }

    /* --- ขับขี่ต่อเนื่อง (ไม่ใช่จังหวะปล่อยคลัทช์ครั้งแรก) --- */

    if (g_brakePressed == 1u)
    {
        /* เหยียบเบรก: ความเร็วลดเร็ว, รอบเครื่องคำนวณย้อนกลับจากความเร็ว
           (ความเร็วคุมรอบ) ถ้ารอบตกต่ำกว่าเกณฑ์ -> เครื่องดับ */
        if (g_currentVelocity > VELOCITY_DECAY_BRAKE)
        {
            g_currentVelocity = g_currentVelocity - VELOCITY_DECAY_BRAKE;
        }
        else
        {
            g_currentVelocity = 0.0f;
        }

        g_currentRpm = RpmFromVelocity(g_currentVelocity, currentGearRatio);

        if (g_currentRpm < STALL_RPM_THRESHOLD)
        {
            TriggerStall();
        }
        else
        {
            /* ยังไม่ดับ */
        }

        return;
    }

    /* --- ไม่เบรก ไม่เหยียบคลัทช์: เทียบรอบที่คันเร่งสั่งกับรอบปัจจุบัน --- */
    throttleRpm = ThrottleToRpm(g_throttleRaw);

    if (throttleRpm >= g_currentRpm)
    {
        /* เร่งเครื่อง: "รอบคุมความเร็ว" - กำหนดรอบตามคันเร่งตรงๆ แล้วให้
           ความเร็วตามรอบนั้นทันที */
        g_currentRpm = throttleRpm;
        g_currentVelocity = VelocityFromRpm(g_currentRpm, currentGearRatio);
    }
    else
    {
        /* ผ่อนคันเร่ง (Engine Brake): "ความเร็วคุมรอบ" - ห้ามเอาค่าคันเร่งมา
           กำหนดรอบ/ความเร็วตรงๆ อีกต่อไป ให้ความเร็วค่อยๆ ไหลลงหาความเร็ว
           เป้าหมายของตำแหน่งคันเร่งใหม่ทีละ VELOCITY_DECAY_ENGINEBRAKE ก่อน
           (ถ้าคันเร่งอยู่ต่ำสุด เป้าหมายนี้จะเท่ากับความเร็วปล่อยไหลที่รอบ
           เดินเบาพอดี คือ Crawling Speed) แล้วค่อยคำนวณรอบเครื่องย้อนกลับ
           จากความเร็วที่ลดลงนั้นอีกที */
        targetVelocity = VelocityFromRpm(throttleRpm, currentGearRatio);

        if (g_currentVelocity > (targetVelocity + VELOCITY_DECAY_ENGINEBRAKE))
        {
            g_currentVelocity = g_currentVelocity - VELOCITY_DECAY_ENGINEBRAKE;
        }
        else
        {
            g_currentVelocity = targetVelocity;
        }

        g_currentRpm = RpmFromVelocity(g_currentVelocity, currentGearRatio);
    }
}

/* ตรวจสอบการออกตัวจากจุดหยุดนิ่ง (เรียกตอนปล่อยคลัทช์ขณะความเร็ว = 0) */
void LaunchCheck(uint8_t gear)
{
    float throttleRpm;
    float currentGearRatio;

    throttleRpm = ThrottleToRpm(g_throttleRaw);
    currentGearRatio = GearRatioFor(gear);

    if ((gear == GEAR_1) || (gear == GEAR_REVERSE))
    {
        /* ออกตัวได้เสมอ: ไม่เหยียบคันเร่ง -> ไหลด้วยรอบเดินเบา, เหยียบ -> ตามคันเร่ง */
        if (g_throttleRaw > THROTTLE_IDLE_RAW_MAX)
        {
            g_currentRpm = throttleRpm;
        }
        else
        {
            g_currentRpm = IDLE_RPM;
        }

        g_currentVelocity = VelocityFromRpm(g_currentRpm, currentGearRatio);
    }
    else if (gear == GEAR_2)
    {
        /* ต้องเร่งเครื่องเกิน 1200 rpm ตอนปล่อยคลัทช์ ไม่งั้นดับ */
        if (throttleRpm > GEAR2_LAUNCH_RPM_MIN)
        {
            g_currentRpm = throttleRpm;
            g_currentVelocity = VelocityFromRpm(g_currentRpm, currentGearRatio);
        }
        else
        {
            TriggerStall();
        }
    }
    else
    {
        /* เกียร์ 3, 4, 5: ออกตัวจากจุดหยุดนิ่งไม่ได้ ดับเสมอ */
        TriggerStall();
    }
}

void TriggerStall(void)
{
    g_engineStalled = 1u;
    g_engineStarted = 0u;
    g_currentRpm = 0.0f;
    /* ความเร็วไม่รีเซ็ตทันที ปล่อยให้ทางแยก "เครื่องไม่ทำงาน" ใน
       UpdateDrivetrain() ไล่ลดลงตามแรงเสียดทานในติ๊กถัดไป */
}

/* ==================================================================== */
/* สูตรคำนวณ                                                              */
/* ==================================================================== */

float ThrottleToRpm(uint16_t rawValue)
{
    float ratio;
    float rpm;

    ratio = (float)rawValue / 4095.0f;
    rpm = IDLE_RPM + (ratio * (REDLINE_RPM - IDLE_RPM));

    return rpm;
}

float GearRatioFor(uint8_t gearDigit)
{
    float ratio;

    switch (gearDigit)
    {
        case GEAR_1: { ratio = 3.928f; break; }
        case GEAR_2: { ratio = 2.333f; break; }
        case GEAR_3: { ratio = 1.451f; break; }
        case GEAR_4: { ratio = 1.000f; break; }
        case GEAR_5: { ratio = 0.851f; break; }
        case GEAR_REVERSE: { ratio = 4.743f; break; }
        default: { ratio = 1.000f; break; }
    }

    return ratio;
}

float VelocityFromRpm(float rpm, float gearRatio)
{
    return (rpm * 0.36f) / (2.6526f * gearRatio * FINAL_DRIVE);
}

float RpmFromVelocity(float velocity, float gearRatio)
{
    return (velocity * 2.6526f * gearRatio * FINAL_DRIVE) / 0.36f;
}

/* ==================================================================== */
/* Dashboard (UART2, 9600 baud) - พิมพ์ทุก UART_REPORT_PERIOD_TICKS         */
/* ==================================================================== */

void ReportDashboard(void)
{
    g_uartReportTickCounter = g_uartReportTickCounter + 1u;

    if (g_uartReportTickCounter >= UART_REPORT_PERIOD_TICKS)
    {
        g_uartReportTickCounter = 0u;

        /* ส่งเป็นบรรทัดเดียว รูปแบบ CSV: <RPM>,<VELOCITY>\r\n
           เพื่อให้ฝั่ง JavaScript ใช้ data.split(',') อ่านค่าทั้งคู่พร้อมกันได้
           ในจังหวะเดียว ไม่ต้องแยกบรรทัดแบบเดิม */
        UART_SendFloatOneDecimal(g_currentRpm);
        UART_SendChar(',');
        UART_SendFloatOneDecimal(g_currentVelocity);
        UART_SendString("\r\n");
    }
    else
    {
        /* ยังไม่ถึงจังหวะพิมพ์ */
    }
}

/* ==================================================================== */
/* 7-Segment (BCD decoder) - เหมือนโปรเจกต์เดิม                            */
/* ==================================================================== */

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

/* ==================================================================== */
/* FPU / SysTick / Startup wait / Calibration                             */
/* ==================================================================== */

void FPU_Enable(void)
{
    /* เปิด full access ให้ coprocessor CP10/CP11 (Hardware FPU) */
    SCB->CPACR |= ((3UL << (10u * 2u)) | (3UL << (11u * 2u)));
    __DSB();
    __ISB();
}

void SysTick_Init(void)
{
    SysTick->LOAD = SYSTICK_RELOAD_1MS;
    SysTick->VAL = 0u;
    SysTick->CTRL = (SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk);
}

/* ตัวจับเวลาแบบ Non-blocking โดยอิง SysTick tick counter - ใช้ตอน Startup
   เท่านั้น (ก่อนเข้า while(1) หลัก) ไม่ใช่ Polling ใน Main Loop */
void WaitStartupMs(uint32_t ms)
{
    uint32_t targetTick;

    targetTick = g_systemTickMs + ms;

    while (g_systemTickMs < targetTick)
    {
        /* รอเฉพาะตอนเริ่มระบบครั้งเดียว */
    }
}

void CalibrateJoystickCenter(void)
{
    WaitStartupMs(500u); /* ให้ ADC round-robin อ่านค่าเข้ามาก่อน */

    g_centerX = (int32_t)g_joyVrxRaw;
    g_centerY = (int32_t)g_joyVryRaw;

    /* ไม่พิมพ์ค่า Center ออกทาง UART แล้ว - สาย UART ต้องมีแต่ CSV data
       frame ของ RPM/Velocity เท่านั้น ห้ามมีข้อความอื่นหลุดออกไปปนเด็ดขาด */
}

/* ==================================================================== */
/* GPIO / ADC / SysTick ISR                                               */
/* ==================================================================== */

void GPIO_Init(void)
{
    RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN);
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    /* PA10 = ปุ่มกุญแจ, input pull-up */
    GPIOA->MODER &= ~(GPIO_MODER_MODER10);
    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD10);
    GPIOA->PUPDR |= (0b01 << GPIO_PUPDR_PUPD10_Pos);

    /* PB3 = ปุ่มคลัทช์, input pull-up */
    GPIOB->MODER &= ~(GPIO_MODER_MODER3);
    GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD3);
    GPIOB->PUPDR |= (0b01 << GPIO_PUPDR_PUPD3_Pos);

    /* PB4 = ปุ่มเบรก, input pull-up */
    GPIOB->MODER &= ~(GPIO_MODER_MODER4);
    GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD4);
    GPIOB->PUPDR |= (0b01 << GPIO_PUPDR_PUPD4_Pos);

    /* PB6 = LED เขียว, output */
    GPIOB->MODER &= ~(GPIO_MODER_MODER6);
    GPIOB->MODER |= (0b01 << GPIO_MODER_MODER6_Pos);

    /* PA7 = LED เหลือง (ถอยหลัง), output */
    GPIOA->MODER &= ~(GPIO_MODER_MODER7);
    GPIOA->MODER |= (0b01 << GPIO_MODER_MODER7_Pos);

    /* PA6 = LED แดง (เบรก), output */
    GPIOA->MODER &= ~(GPIO_MODER_MODER6);
    GPIOA->MODER |= (0b01 << GPIO_MODER_MODER6_Pos);

    /* PA4 = potentiometer สตาร์ท, analog input (ADC1_IN4) */
    GPIOA->MODER &= ~(GPIO_MODER_MODER4);
    GPIOA->MODER |= (0b11 << GPIO_MODER_MODER4_Pos);

    /* PA0 = potentiometer คันเร่ง, analog input (ADC1_IN0) */
    GPIOA->MODER &= ~(GPIO_MODER_MODER0);
    GPIOA->MODER |= (0b11 << GPIO_MODER_MODER0_Pos);

    /* PC2 = joystick VRx, analog input (ADC1_IN12) */
    GPIOC->MODER &= ~(GPIO_MODER_MODER2);
    GPIOC->MODER |= (0b11 << GPIO_MODER_MODER2_Pos);

    /* PC3 = joystick VRy, analog input (ADC1_IN13) */
    GPIOC->MODER &= ~(GPIO_MODER_MODER3);
    GPIOC->MODER |= (0b11 << GPIO_MODER_MODER3_Pos);

    /* 7-segment ผ่าน BCD decoder: PC7, PA8, PB10, PA9 เป็น output */
    GPIOC->MODER &= ~(GPIO_MODER_MODER7);
    GPIOC->MODER |= (0b01 << GPIO_MODER_MODER7_Pos);

    GPIOA->MODER &= ~(GPIO_MODER_MODER8 | GPIO_MODER_MODER9);
    GPIOA->MODER |= (0b01 << GPIO_MODER_MODER8_Pos) | (0b01 << GPIO_MODER_MODER9_Pos);

    GPIOB->MODER &= ~(GPIO_MODER_MODER10);
    GPIOB->MODER |= (0b01 << GPIO_MODER_MODER10_Pos);

    /* PA2 = USART2 TX, alternate function AF7 */
    GPIOA->MODER &= ~(GPIO_MODER_MODER2);
    GPIOA->MODER |= (0b10 << GPIO_MODER_MODER2_Pos);
    GPIOA->AFR[0] &= ~(0xFu << (2u * 4u));
    GPIOA->AFR[0] |= (7u << (2u * 4u));
}

void ADC_Init(void)
{
    /* Sample time: channel 0,4 อยู่ใน SMPR2, channel 12,13 อยู่ใน SMPR1 */
    ADC1->SMPR2 |= (ADC_SMPR2_SMP0 | ADC_SMPR2_SMP4);
    ADC1->SMPR1 |= (ADC_SMPR1_SMP12 | ADC_SMPR1_SMP13);

    ADC1->SQR1 &= ~(ADC_SQR1_L);

    /* เปิด interrupt ทุกครั้งที่แปลงเสร็จ (EOC) */
    ADC1->CR1 |= ADC_CR1_EOCIE;
    NVIC_EnableIRQ(ADC_IRQn);

    ADC1->CR2 |= ADC_CR2_ADON;

    /* เริ่มวง Round-Robin: throttle -> VRx -> VRy -> ignition -> วนกลับ */
    g_adcChannelIndex = 0u;
    ADC_StartChannel(ADC_CHANNEL_THROTTLE);
}

void ADC_StartChannel(uint8_t channel)
{
    ADC1->SQR3 &= ~(ADC_SQR3_SQ1);
    ADC1->SQR3 |= ((uint32_t)channel << ADC_SQR3_SQ1_Pos);
    ADC1->CR2 |= ADC_CR2_SWSTART;
}

/* Round-Robin 4 ช่อง: throttle -> VRx -> VRy -> ignition -> วนกลับ
   ไม่มี Polling เลย - ทุกอย่างเกิดขึ้นเองเมื่อ EOC เกิด */
void ADC_IRQHandler(void)
{
    uint16_t result;

    if ((ADC1->SR & ADC_SR_EOC) != 0u)
    {
        result = (uint16_t)ADC1->DR;

        if (g_adcChannelIndex == 0u)
        {
            g_throttleRaw = result;
            g_adcChannelIndex = 1u;
            ADC_StartChannel(ADC_CHANNEL_JOY_VRX);
        }
        else if (g_adcChannelIndex == 1u)
        {
            g_joyVrxRaw = result;
            g_adcChannelIndex = 2u;
            ADC_StartChannel(ADC_CHANNEL_JOY_VRY);
        }
        else if (g_adcChannelIndex == 2u)
        {
            g_joyVryRaw = result;
            g_adcChannelIndex = 3u;
            ADC_StartChannel(ADC_CHANNEL_IGNITION);
        }
        else
        {
            g_ignitionRaw = result;
            g_adcChannelIndex = 0u;
            ADC_StartChannel(ADC_CHANNEL_THROTTLE);
        }
    }
    else
    {
        /* ไม่มีอะไรต้องทำ */
    }
}

/* ตั้ง flag ให้ main loop ทำงานทุก CONTROL_LOOP_PERIOD_MS - เป็น Timebase
   หลักของทั้งระบบ (ปุ่มกด, กระพริบไฟ, การคำนวณ, การพิมพ์ dashboard) */
void SysTick_Handler(void)
{
    g_systemTickMs = g_systemTickMs + 1u;

    if ((g_systemTickMs % CONTROL_LOOP_PERIOD_MS) == 0u)
    {
        g_controlLoopFlag = 1u;
    }
    else
    {
        /* ยังไม่ถึงจังหวะ */
    }
}

/* ==================================================================== */
/* UART                                                                    */
/* ==================================================================== */

void UART_Init(void)
{
    /* 9600 baud ที่ HSI 16MHz (ค่า default หลัง reset) */
    USART2->BRR = 0x683u;
    USART2->CR1 |= (USART_CR1_UE | USART_CR1_TE);
}

void UART_SendChar(char c)
{
    while ((USART2->SR & USART_SR_TXE) == 0u)
    {
        /* รอ transmit buffer ว่าง (ใช้เฉพาะตอนส่งอักขระ ไม่ได้บล็อก main loop
           ยาวๆ เพราะการพิมพ์เกิดไม่บ่อย และแต่ละตัวอักษรใช้เวลาสั้นมาก) */
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

    scaledValue = (value * 10.0f) + 0.5f;
    tenths = (int32_t)scaledValue;

    wholePart = tenths / 10;
    tenths = tenths % 10;

    UART_SendNumber(wholePart);
    UART_SendChar('.');
    UART_SendNumber(tenths);
}