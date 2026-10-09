/**
 ******************************************************************************
 * @file           : main.c / mighty_x_sim.c
 * @brief          : Toyota Mighty-X (2L) Driving Simulator - STM32F411RE
 ******************************************************************************
 *
 * [ภาพรวมระบบ]
 * 1. Hardware Drivers & Interrupt System:
 *    - TIM2   : ฐานเวลา 10ms สั่ง ADC scan และตั้ง flag ให้ main loop
 *    - ADC1 + DMA2 : อ่าน 5 ช่อง (คันเร่ง, Joystick X/Y, สตาร์ท, LDR) อัตโนมัติ ไม่มี polling
 *    - EXTI   : คลัทช์ PB3, เบรก PB4, ปุ่มเติมน้ำมัน PB5, กุญแจ PA10
 *    - TIM3   : สร้างคลื่นเสียง buzzer (passive) ที่ PC0 โดย toggle ใน interrupt
 *    - USART2 : ส่งข้อมูล Dashboard แบบ Non-blocking (TX Interrupt + Ring Buffer)
 *
 * 2. สถานะระบบขับเคลื่อน (Drivetrain States):
 *    - ENGINE OFF   : ค้างเกียร์ -> แรงฉุดเครื่องยนต์, เกียร์ว่าง/เหยียบคลัทช์ -> ไหลอิสระ
 *    - DISCONNECTED : เหยียบคลัทช์/เกียร์ว่าง -> รอบตามคันเร่ง, รถชะลอตามแรงต้าน
 *    - CLUTCH SLIP  : จังหวะปล่อยคลัทช์ รอบเครื่องกับความเร็วล้อค่อยๆ เข้าหากัน
 *    - IN GEAR      : เร่ง / Engine Brake / เลี้ยงรอบเดินเบา / เบรกจนรอบตก -> ดับ
 *
 * 3. ระบบน้ำมัน + โหมดเติมน้ำมัน (PB5 + LDR) และเสียง buzzer ตามรอบเครื่อง
 *
 * 4. อาการเครื่อง (ไฟเหลือง PA7 กะพริบเตือน): แรงบิดตามรอบ, เครื่องหอบ, กระชากตอนต่อคลัทช์,
 *    ลดเกียร์รอบเกิน (หน่วงแรงแต่ไม่ดับ), เข้าเกียร์ไม่เหยียบคลัทช์ = เฟืองขบ เข้าไม่ได้ (ไม่ดับ)
 *
 * 5. UART CSV ไปหน้าเว็บ: <RPM>,<VELOCITY>,<FUEL_LITERS>,<STALL_REASON>,<WARNING>,<JERK>\r\n
 ******************************************************************************
 */

#include <stdint.h>
#include <math.h>
#define STM32F411xE
#include "stm32f4xx.h"

/* ==================================================================== */
/* 1. CONSTANTS & DEFINITIONS (การกำหนดค่าคงที่)                          */
/* ==================================================================== */

/* --- จังหวะการทำงานระบบ (Timebase Settings) --- */
#define CONTROL_LOOP_PERIOD_MS       10u         /* ลูปหลักทำงานทุกๆ 10ms */
#define TICK_SECONDS                 0.01f       /* 1 tick = 0.01 วินาที */
#define UART_REPORT_PERIOD_TICKS     20u         /* ส่งออก UART ทุกๆ 20 ticks (200ms) */
#define CALIBRATION_DELAY_MS         500u        /* รอ ADC นิ่งก่อนจำตำแหน่งกลาง Joystick */

/* --- ค่าตั้ง Peripheral (Clock HSI 16MHz) --- */
#define TIM2_PRESCALER_1KHZ          (16000u - 1u)   /* 16MHz / 16000 = 1kHz */
#define TIM2_RELOAD_10MS             (CONTROL_LOOP_PERIOD_MS - 1u)
#define TIM3_PRESCALER_1MHZ          (16u - 1u)      /* 16MHz / 16 = 1MHz */
#define TIM3_CLOCK_HZ                1000000.0f
#define USART2_BRR_9600              0x683u          /* 16MHz / 9600 */
#define FPU_CP10_CP11_FULL_ACCESS    (0xFu << 20u)

/* --- ค่าตั้ง GPIO --- */
#define GPIO_MODE_INPUT              0u
#define GPIO_MODE_OUTPUT             1u
#define GPIO_MODE_AF                 2u
#define GPIO_MODE_ANALOG             3u
#define GPIO_MODE_MASK               3u
#define GPIO_PULL_UP                 1u
#define GPIO_PULL_MASK               3u
#define GPIO_FIELD_BITS              2u          /* จำนวนบิตต่อ 1 ขาใน MODER/PUPDR */
#define GPIO_AF_FIELD_BITS           4u
#define GPIO_AF_MASK                 0xFu
#define GPIO_AF7_USART2              7u
#define GPIO_BSRR_RESET_SHIFT        16u         /* ครึ่งบนของ BSRR = สั่งขาเป็น 0 */

/* --- ขาที่ใช้งาน --- */
#define PIN_THROTTLE                 0u          /* PA0  ADC1_IN0  */
#define PIN_LDR                      1u          /* PA1  ADC1_IN1  */
#define PIN_UART_TX                  2u          /* PA2  USART2_TX */
#define PIN_IGNITION                 4u          /* PA4  ADC1_IN4  */
#define PIN_REVERSE_LED              5u          /* PA5  LED ฟ้า (ไฟถอยหลัง) */
#define PIN_BRAKE_LED                6u          /* PA6  LED แดง (ไฟเบรก) */
#define PIN_WARNING_LED              7u          /* PA7  LED เหลือง (ไฟเตือนเครื่องผิดปกติ ใช้แทนสีส้ม) */
#define PIN_SEG_BIT1                 8u          /* PA8  */
#define PIN_SEG_BIT3                 9u          /* PA9  */
#define PIN_KEY_BUTTON               10u         /* PA10 */
#define PIN_CLUTCH                   3u          /* PB3  */
#define PIN_BRAKE                    4u          /* PB4  */
#define PIN_REFUEL_BUTTON            5u          /* PB5  */
#define PIN_STATUS_LED               6u          /* PB6  LED เขียว */
#define PIN_SEG_BIT2                 10u         /* PB10 */
#define PIN_BUZZER                   0u          /* PC0  (ขา A5 บน header) */
#define PIN_JOY_X                    2u          /* PC2  ADC1_IN12 */
#define PIN_JOY_Y                    3u          /* PC3  ADC1_IN13 */
#define PIN_SEG_BIT0                 7u          /* PC7  */

/* --- ADC Scan (ลำดับใน DMA Buffer) --- */
#define ADC_CH_THROTTLE              0u
#define ADC_CH_LDR                   1u
#define ADC_CH_IGNITION              4u
#define ADC_CH_JOY_X                 12u
#define ADC_CH_JOY_Y                 13u
#define ADC_SCAN_LENGTH              5u
#define ADC_IDX_THROTTLE             0u
#define ADC_IDX_JOY_X                1u
#define ADC_IDX_JOY_Y                2u
#define ADC_IDX_IGNITION             3u
#define ADC_IDX_LDR                  4u
#define ADC_MAX_VALUE                4095.0f
#define ADC_MAX_RAW                  4095u       /* ค่าสูงสุดของ ADC 12 บิต (ใช้กลับด้านคันเร่ง) */
#define HALF_ADC_VALUE               2048u

/* --- DMA --- */
#define DMA_PRIORITY_MEDIUM          1u
#define DMA_SIZE_HALFWORD            1u

/* --- EXTI (เลือกพอร์ตใน SYSCFG) --- */
#define EXTI_PORT_A                  0u
#define EXTI_PORT_B                  1u
#define EXTI_LINES_PER_REG           4u
#define EXTI_FIELD_BITS              4u
#define EXTI_FIELD_MASK              0xFu

/* --- กุญแจ และ สตาร์ท --- */
#define BLINK_NORMAL_TICKS           50u         /* กระพริบปกติทุก 500ms */
#define BLINK_STALL_TICKS            10u         /* กระพริบเตือนดับทุก 100ms */
#define KEY_DEBOUNCE_MS              200u        /* หน่วงปุ่มกุญแจ 200ms */
#define BUTTON_DEBOUNCE_TICKS        2u          /* ปุ่มต้องนิ่ง 20ms */

/* --- เครื่องยนต์ --- */
#define IDLE_RPM                     800.0f
#define REDLINE_RPM                  4200.0f
#define STALL_RPM_THRESHOLD          600.0f      /* 75% ของรอบเดินเบา */
#define GEAR2_LAUNCH_RPM_MIN         1200.0f     /* รอบขั้นต่ำในการออกตัวเกียร์ 2 */
#define SLIP_MIN_WHEEL_RPM           300.0f      /* เกียร์ 3-5 ต้องไหลอย่างน้อยเท่านี้ถึงจะเลี้ยงคลัทช์ได้ */
#define CLUTCH_LOCK_TOLERANCE_RPM    50.0f       /* รอบเครื่องกับล้อห่างไม่เกินนี้ = คลัทช์จับสนิท */
#define THROTTLE_IDLE_RAW_MAX        200u        /* ADC คันเร่งต่ำกว่านี้ = ไม่ได้เหยียบ */

/* --- อัตราการเปลี่ยนรอบเครื่อง (rpm ต่อ 10ms tick) --- */
#define CLUTCH_ENGAGE_RATE           100.0f      /* รอบวิ่งเข้าหารอบล้อขณะคลัทช์กำลังจับ */
#define FREE_REV_RISE_RATE           150.0f      /* เร่งเครื่องฟรี (เหยียบคลัทช์/เกียร์ว่าง) */
#define FREE_REV_FALL_RATE           100.0f      /* ถอนคันเร่งแล้วรอบร่วงกลับรอบเดินเบา */

/* --- กราฟแรงบิด (สัดส่วนแรงเร่งตามรอบเครื่อง, เส้นตรงเป็นช่วง) --- */
#define TORQUE_LOW_RPM               800.0f      /* รอบต่ำ: แรงบิดน้อย */
#define TORQUE_LOW_FACTOR            0.35f
#define TORQUE_PEAK_START_RPM        1500.0f     /* ช่วงแรงบิดเต็ม */
#define TORQUE_PEAK_END_RPM          3200.0f
#define TORQUE_PEAK_FACTOR           1.0f
#define TORQUE_REDLINE_FACTOR        0.7f        /* ใกล้ redline แรงบิดตก */

/* --- เครื่องหอบ (รอบต่ำแต่เหยียบคันเร่งหนัก) --- */
#define LUG_RPM_MAX                  1100.0f
#define LUG_THROTTLE_MIN             1638u       /* คันเร่ง ~40% ขึ้นไป */
#define LUG_WOBBLE_RPM               40          /* รอบแกว่ง +-40 */
#define LUG_WOBBLE_STEP_RPM          20          /* แกว่งขึ้น/ลงทีละ 20 rpm ต่อ tick */
#define LUG_WOBBLE_PERIOD_TICKS      8u          /* แกว่งครบรอบทุก 80ms */
#define LUG_WOBBLE_HALF_TICKS        4u

/* --- กระชากตอนต่อคลัทช์ / รอบเกิน --- */
#define JERK_FREE_RPM                500.0f      /* รอบเครื่องกับล้อต่างไม่เกินนี้ = ตบเกียร์ได้จังหวะ */
#define JERK_FULL_RPM                2500.0f     /* ต่างเท่านี้ = กระชากแรงสุด (100) */
#define JERK_FRACTION                0.15f       /* ความเร็วถูกดึงเข้าหาความเร็วตามรอบเครื่อง 15% ของส่วนต่าง */
#define JERK_MAX_KMH                 5.0f        /* กระชากได้ไม่เกิน 5 km/h ในครั้งเดียว */
#define JERK_INTENSITY_MAX           100.0f
#define OVERREV_DECAY                0.25f       /* รอบเกิน: เครื่องฉุดรถแรงมาก ~25 km/h/s จนรอบลงถึง redline */
#define REV_DISPLAY_MAX              6000.0f     /* เข็มรอบสูงสุดบนหน้าปัด */

/* --- เสียงเฟืองขบ (เข้าเกียร์ไม่เหยียบคลัทช์) --- */
#define GRIND_HZ_A                   1800.0f
#define GRIND_HZ_B                   2400.0f     /* สลับสองความถี่ทุก 10ms ให้เสียงครืด */

/* --- คำเตือนเครื่องผิดปกติ (ไฟเหลือง + หน้าเว็บ) --- */
#define WARN_NONE                    0u
#define WARN_LUGGING                 1u          /* เครื่องหอบ */
#define WARN_JERK                    2u          /* กระชาก */
#define WARN_OVERREV                 3u          /* รอบเกิน */
#define WARN_GEAR_GRIND              4u          /* เข้าเกียร์ไม่ได้ เฟืองขบ */
#define WARN_PRIORITY_NONE           0u          /* ลำดับความสำคัญ: รอบเกิน > กระชาก > เฟืองขบ > หอบ */
#define WARN_PRIORITY_LUGGING        1u
#define WARN_PRIORITY_GEAR_GRIND     2u
#define WARN_PRIORITY_JERK           3u
#define WARN_PRIORITY_OVERREV        4u
#define WARNING_HOLD_TICKS           50u         /* ค้างคำเตือน 0.5 วินาที ให้ทันรอบส่ง UART */
#define WARNING_BLINK_TICKS          10u         /* ไฟเหลืองกะพริบทุก 100ms */

/* --- สูตรความเร็ว / อัตราทด --- */
#define FINAL_DRIVE                  4.300f
#define TIRE_FACTOR                  2.6526f
#define KMH_FACTOR                   0.36f
#define GEAR_RATIO_1                 3.928f
#define GEAR_RATIO_2                 2.333f
#define GEAR_RATIO_3                 1.451f
#define GEAR_RATIO_4                 1.000f
#define GEAR_RATIO_5                 0.851f
#define GEAR_RATIO_REVERSE           4.743f

/* --- อัตราเร่ง/ชะลอ (หน่วย: km/h ต่อ 10ms tick) --- */
#define ACCEL_PER_RATIO              0.028f      /* เร่งสูงสุด = ค่านี้ x อัตราทด (G1 ~11 km/h/s, G5 ~2.4 km/h/s) */
#define COAST_DECAY                  0.025f      /* ไหลอิสระ ~2.5 km/h/s */
#define AIR_DRAG_K                   0.000003f   /* แรงต้านอากาศ ~ v^2 (100 km/h เพิ่มอีก ~3 km/h/s) */
#define ENGINE_BRAKE_BASE            0.020f      /* Engine Brake = BASE + PER_RATIO x อัตราทด */
#define ENGINE_BRAKE_PER_RATIO       0.020f
#define ENGINE_DRAG_DECAY            0.080f      /* เครื่องดับค้างเกียร์ (คูณอัตราทด) */
#define BRAKE_DECAY                  0.080f      /* เหยียบเบรก ~8 km/h/s */

/* --- ความเร็วอ้างอิง --- */
#define SPEED_STOPPED_KMH            0.5f        /* ต่ำกว่านี้ถือว่าออกตัวจากจุดหยุดนิ่ง */
#define SPEED_ZERO_KMH               0.01f       /* ต่ำกว่านี้ถือว่ารถหยุดสนิท */

/* --- ทิศทางการเคลื่อนที่ของรถ --- */
#define DIRECTION_STOPPED            0u
#define DIRECTION_FORWARD            1u
#define DIRECTION_REVERSE            2u

/* --- Joystick (ระยะห่างจากจุดกลาง) --- */
#define JOY_LOW_LIMIT                (-2500)
#define JOY_HIGH_LIMIT               120
#define JOY_HYSTERESIS               60          /* อยู่ในเกียร์แล้ว ต้องถอยเลยเส้นอีกเท่านี้ถึงนับว่าออก */
#define GEAR_CONFIRM_TICKS           5u          /* ตำแหน่งเกียร์ใหม่ต้องนิ่ง 50ms กัน ADC แกว่ง */

/* --- สาเหตุเครื่องดับ (ส่งไปหน้าเว็บช่วยหาสาเหตุ, รหัส 1 และ 4 เลิกใช้แล้ว: กลายเป็นคำเตือนแทน) --- */
#define STALL_NONE                   0u
#define STALL_START_IN_GEAR          2u          /* สตาร์ทค้างเกียร์ไม่เหยียบคลัทช์ */
#define STALL_WRONG_DIRECTION        3u          /* เข้าเกียร์สวนทิศทางรถ */
#define STALL_LUGGING                5u          /* ปล่อยคลัทช์รอบต่ำ เครื่องพยุงไม่ไหว */
#define STALL_BRAKE_LUG              6u          /* เบรกไม่เหยียบคลัทช์จนรอบ < 600 */
#define STALL_OUT_OF_FUEL            7u          /* น้ำมันหมด */

/* --- น้ำมัน --- */
#define FUEL_TANK_MAX_L              56.0f
#define FUEL_INITIAL_L               28.0f
#define FUEL_IDLE_LPS                0.005f      /* ลิตรต่อวินาทีที่รอบเดินเบา (ปรับสเกลให้เห็นเข็มขยับตอนเดโม) */
#define FUEL_PER_RPM_LPS             0.00002f    /* ลิตรต่อวินาทีต่อรอบ */
#define REFUEL_MAX_ADD_L             9u          /* เติมได้สูงสุดครั้งละ 9 ลิตร (7-seg หลักเดียว) */
#define REFUEL_ZERO_SHOW_TICKS       25u         /* แสดงเลข 0 ตอนเข้าโหมด 0.25 วินาที */
#define REFUEL_TICKS_PER_LITER       100u        /* แสงค้าง 1 วินาที = 1 ลิตร */

/* --- LDR (สูตรเดียวกับ Lab 4.3) --- */
#define LDR_VREF                     3.3f
#define LDR_RX_OHM                   10000.0f
#define LDR_SLOPE                    (-0.6875f)
#define LDR_OFFSET                   5.1276f
#define LDR_MIN_VOLT                 0.001f      /* กันหารศูนย์/log ของศูนย์ */
#define LUX_BASE                     10.0f
#define LUX_SATURATED                100000.0f
#define FLASHLIGHT_LUX_ON            1000.0f     /* ต้องปรับตามไฟแฟลชจริง: สว่างถึงนี้ = เริ่มเติม */
#define FLASHLIGHT_LUX_OFF           700.0f      /* ต่ำกว่านี้ = หยุดเติม (hysteresis กันค่าแกว่ง) */

/* --- Buzzer (passive) --- */
#define BUZZER_HZ_AT_IDLE            300.0f      /* ความถี่ที่รอบเดินเบา */
#define BUZZER_HZ_AT_REDLINE         2000.0f     /* ความถี่ที่รอบสูงสุด */
#define BUZZER_HZ_MIN                100.0f
#define BUZZER_HZ_MAX                2500.0f
#define BUZZER_START_HZ              150.0f      /* เสียงสตาร์ทเริ่มจากความถี่นี้ แล้วไต่ขึ้นหารอบเดินเบา */
#define BUZZER_START_TICKS           50u         /* เสียงสตาร์ทยาว 0.5 วินาที */
#define TOGGLES_PER_PERIOD           2.0f        /* 1 คาบคลื่น = toggle 2 ครั้ง */

/* --- UART TX Ring Buffer --- */
#define UART_TX_BUFFER_SIZE          128u
#define UART_NUMBER_DIGITS_MAX       12u
#define DECIMAL_BASE                 10u
#define ONE_DECIMAL_SCALE            10.0f
#define ROUND_HALF                   0.5f

/* --- รหัสเกียร์ (Gear Codes) --- */
#define GEAR_NEUTRAL                 0u
#define GEAR_1                       1u
#define GEAR_2                       2u
#define GEAR_3                       3u
#define GEAR_4                       4u
#define GEAR_5                       5u
#define GEAR_REVERSE                 8u

/* --- บิต BCD ของ 7-Segment --- */
#define SEG_BCD_BIT0                 0x01u
#define SEG_BCD_BIT1                 0x02u
#define SEG_BCD_BIT2                 0x04u
#define SEG_BCD_BIT3                 0x08u

/* ==================================================================== */
/* 2. GLOBAL VARIABLES (ตัวแปรโกลบอล)                                     */
/* ==================================================================== */

/* --- ADC Buffer (อัปเดตอัตโนมัติด้วย DMA2) --- */
volatile uint16_t g_adcBuffer[ADC_SCAN_LENGTH] =
{
    0u, 0u, 0u, 0u, 0u
};
uint16_t g_throttleRaw = 0u;   /* CH0  - PA0 (กลับด้านแล้ว: 0 = ไม่เหยียบ, 4095 = เต็ม) */
uint16_t g_joyVrxRaw   = 0u;   /* CH12 - PC2 */
uint16_t g_joyVryRaw   = 0u;   /* CH13 - PC3 */
uint16_t g_ignitionRaw = 0u;   /* CH4  - PA4 */
uint16_t g_ldrRaw      = 0u;   /* CH1  - PA1 */

/* --- ระบบเวลา --- */
volatile uint32_t g_systemTickMs    = 0u;
volatile uint8_t  g_controlLoopFlag = 0u;

/* --- ค่า Center ของ Joystick --- */
int32_t g_centerX = 0;
int32_t g_centerY = 0;

/* --- สถานะระบบกุญแจ และ เครื่องยนต์ --- */
volatile uint8_t  g_keyInserted        = 0u;
volatile uint8_t  g_engineStarted      = 0u;
volatile uint8_t  g_engineStalled      = 0u;
uint8_t g_stallReason = STALL_NONE;          /* สาเหตุการดับล่าสุด ล้างเมื่อสตาร์ทติด */
volatile uint32_t g_lastKeyPressTickMs = 0u;
uint8_t g_previousEngineStarted = 0u;

uint8_t  g_ledBlinkState    = 0u;
uint32_t g_blinkTickCounter = 0u;

/* --- สถานะสวิทช์ (Live จาก EXTI และค่าหลัง Software Debounce) --- */
volatile uint8_t g_clutchPressedLive = 0u;
volatile uint8_t g_brakePressedLive  = 0u;
volatile uint8_t g_refuelButtonLive  = 0u;

uint8_t g_clutchPressed         = 0u;
uint8_t g_brakePressed          = 0u;
uint8_t g_refuelButtonPressed   = 0u;
uint8_t g_previousRefuelButton  = 0u;

/* --- สถานะเกียร์ และ ระบบขับเคลื่อน --- */
uint8_t g_currentGear      = GEAR_NEUTRAL;
uint8_t g_gearGrindActive  = 0u;     /* 1 = กำลังพยายามเข้าเกียร์โดยไม่เหยียบคลัทช์ */
uint8_t g_travelDirection  = DIRECTION_STOPPED;
uint8_t g_drivelineEngaged = 0u;     /* 1 = เกียร์จับกำลังอยู่ (ผ่านจังหวะต่อคลัทช์แล้ว) */
uint8_t g_clutchSlipActive = 0u;     /* 1 = คลัทช์กำลังจับ (รอบเครื่องยังไม่เท่ารอบล้อ) */

float g_currentRpm      = 0.0f;
float g_currentVelocity = 0.0f;

/* --- น้ำมัน และ โหมดเติมน้ำมัน --- */
float    g_fuelLiters          = FUEL_INITIAL_L;
uint8_t  g_refuelActive        = 0u;
uint8_t  g_refuelZeroTicks     = 0u;
uint8_t  g_refuelAddLiters     = 0u;
uint32_t g_refuelLightTicks    = 0u;
uint8_t  g_refuelLightDetected = 0u;

/* --- Buzzer --- */
uint8_t g_buzzerRunning    = 0u;
uint8_t g_buzzerStartTicks = 0u;

/* --- คำเตือนเครื่องผิดปกติ --- */
uint8_t g_warningCode       = WARN_NONE;
uint8_t g_warningHoldTicks  = 0u;
uint8_t g_jerkIntensity     = 0u;      /* 0-100 ความแรงกระชากล่าสุด (ให้หน้าเว็บสั่น) */
uint8_t g_jerkHoldTicks     = 0u;
uint8_t g_warningBlinkTicks = 0u;
uint8_t g_warningLedState   = 0u;
uint8_t g_lugWobbleTick     = 0u;

/* --- ตัวนับเวลาสำหรับ Dashboard UART --- */
uint32_t g_uartReportTickCounter = 0u;

/* --- UART TX FIFO Ring Buffer --- */
typedef struct
{
    volatile uint8_t  data[UART_TX_BUFFER_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
    volatile uint16_t count;
}
UartTxBuffer_t;

/* ตัวแปร global ไม่ต้องใส่ค่าเริ่มต้น: C ตั้งทุกช่องเป็น 0 ให้อัตโนมัติก่อนเข้า main() */
UartTxBuffer_t g_uartTx;

/* ==================================================================== */
/* 3. FUNCTION PROTOTYPES (ประกาศฟังก์ชัน)                                */
/* ==================================================================== */

/* Hardware Drivers Layer */
void Hardware_FPU_Enable(void);
void Hardware_SetPinMode(GPIO_TypeDef *port, uint32_t pin, uint32_t mode);
void Hardware_SetPullUp(GPIO_TypeDef *port, uint32_t pin);
void Hardware_MapExtiLine(uint32_t line, uint32_t portCode);
void Hardware_GPIO_Init(void);
void Hardware_TIM2_Init(void);
void Hardware_TIM3_Init(void);
void Hardware_ADC1_DMA_Init(void);
void Hardware_EXTI_Init(void);
void Hardware_UART2_Init(void);

/* Utility & System Setup */
void System_DelayMs(uint32_t ms);
void System_CalibrateJoystick(void);

/* Application Layer */
void App_ReadAnalogInputs(void);
uint8_t App_DebounceButton(uint8_t rawState, uint8_t stableState, uint8_t *counter);
void App_ProcessInputDebounce(void);
void App_UpdateIgnitionSystem(void);
void App_UpdateStatusLed(void);
void App_UpdateLamps(void);
uint8_t App_ComputeCurrentGear(void);
void App_UpdateGearSelection(void);
void App_UpdateSegmentDisplay(void);
void App_UpdateRefuelMode(void);
void App_CountRefuelLiters(void);
uint8_t App_RefuelHasRoom(void);
void App_UpdateDrivetrain(void);
void App_UpdateEngineOff(uint8_t powerConnected, float gearRatio);
void App_UpdatePowerDisconnected(float throttleRpm);
void App_EngageDriveline(void);
void App_ApplyEngagementJerk(float wheelRpm, float gearRatio);
uint8_t App_WarningPriority(uint8_t code);
void App_RaiseWarning(uint8_t code);
void App_TickWarnings(void);
float App_LugWobbleRpm(void);
uint8_t App_EngineCanHoldSlip(float wheelRpm, float throttleRpm);
void App_UpdateClutchSlip(float gearRatio, float throttleRpm);
void App_UpdateInGear(float gearRatio, float throttleRpm);
void App_ApproachTargetSpeed(float targetVelocity, float gearRatio);
void App_ReduceSpeed(float decay);
void App_TriggerStall(uint8_t reason);
void App_UpdateFuel(void);
void App_UpdateBuzzer(void);
void App_ReportDashboard(void);

uint8_t Driver_ReadClutchRaw(void);
uint8_t Driver_ReadBrakeRaw(void);
uint8_t Driver_ReadRefuelButtonRaw(void);

/* Physics Calculation Helpers */
float Physics_ThrottleToRpm(uint16_t rawValue);
float Physics_GearRatioFor(uint8_t gearDigit);
float Physics_VelocityFromRpm(float rpm, float gearRatio);
float Physics_RpmFromVelocity(float velocity, float gearRatio);
float Physics_AirDrag(float velocity);
float Physics_CoastDecay(float velocity);
float Physics_EngineBrakeDecay(float velocity, float gearRatio);
float Physics_Slew(float current, float target, float riseRate, float fallRate);
float Physics_MaxFloat(float a, float b);
float Physics_MinFloat(float a, float b);
float Physics_TorqueFactor(float rpm);
float Sensor_LuxFromLdr(uint16_t rawValue);
float Sound_FrequencyFromRpm(float rpm);

/* Display & Communication Helpers */
void Driver_SetOutput(GPIO_TypeDef *port, uint32_t pin, uint8_t state);
uint8_t Driver_BitState(uint8_t value, uint8_t mask);
void Driver_SegmentDisplay(uint8_t number);
void Driver_BuzzerOn(float frequencyHz);
void Driver_BuzzerOff(void);
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
    Hardware_TIM3_Init();
    Hardware_TIM2_Init();

    /* 2. Calibrate Sensors */
    System_CalibrateJoystick();

    /* 3. Initialize External Interrupts after calibration */
    Hardware_EXTI_Init();

    /* 4. Priming Initial States */
    App_ReadAnalogInputs();
    g_currentGear = App_ComputeCurrentGear();
    g_clutchPressed = Driver_ReadClutchRaw();
    g_brakePressed = Driver_ReadBrakeRaw();
    g_refuelButtonPressed = Driver_ReadRefuelButtonRaw();
    g_previousRefuelButton = g_refuelButtonPressed;

    /* 5. Main Control Loop (Non-blocking, Timer Driven) */
    for (;;)
    {
        if (g_controlLoopFlag == 1u)
        {
            g_controlLoopFlag = 0u;

            App_ReadAnalogInputs();
            App_ProcessInputDebounce();

            App_TickWarnings();
            App_UpdateIgnitionSystem();
            App_UpdateGearSelection();
            App_UpdateRefuelMode();
            App_UpdateDrivetrain();
            App_UpdateFuel();
            App_UpdateBuzzer();

            App_UpdateStatusLed();
            App_UpdateLamps();
            App_UpdateSegmentDisplay();
            App_ReportDashboard();

            /* บันทึกค่าเพื่อใช้เปรียบเทียบใน Loop ถัดไป */
            g_previousRefuelButton = g_refuelButtonPressed;
            g_previousEngineStarted = g_engineStarted;
        }
        else
        {
            /* No action */
        }
    }
}

/* ==================================================================== */
/* 5. HARDWARE DRIVERS LAYER (ควบคุมอุปกรณ์ฮาร์ดแวร์)                       */
/* ==================================================================== */

void Hardware_FPU_Enable(void)
{
    SCB->CPACR |= FPU_CP10_CP11_FULL_ACCESS;
    __DSB();
    __ISB();
}

void Hardware_SetPinMode(GPIO_TypeDef *port, uint32_t pin, uint32_t mode)
{
    port->MODER &= ~(GPIO_MODE_MASK << (pin * GPIO_FIELD_BITS));
    port->MODER |= (mode << (pin * GPIO_FIELD_BITS));
}

void Hardware_SetPullUp(GPIO_TypeDef *port, uint32_t pin)
{
    port->PUPDR &= ~(GPIO_PULL_MASK << (pin * GPIO_FIELD_BITS));
    port->PUPDR |= (GPIO_PULL_UP << (pin * GPIO_FIELD_BITS));
}

/* เลือกว่า EXTI line นี้รับสัญญาณจากพอร์ตไหน (SYSCFG->EXTICR) */
void Hardware_MapExtiLine(uint32_t line, uint32_t portCode)
{
    uint32_t regIndex = line / EXTI_LINES_PER_REG;
    uint32_t shift = (line % EXTI_LINES_PER_REG) * EXTI_FIELD_BITS;

    SYSCFG->EXTICR[regIndex] &= ~(EXTI_FIELD_MASK << shift);
    SYSCFG->EXTICR[regIndex] |= (portCode << shift);
}

void Hardware_GPIO_Init(void)
{
    RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN);

    /* ปุ่มทั้งหมดเป็น Input Pull-up (Active-Low): กุญแจ PA10, คลัทช์ PB3, เบรก PB4, เติมน้ำมัน PB5 */
    Hardware_SetPinMode(GPIOA, PIN_KEY_BUTTON, GPIO_MODE_INPUT);
    Hardware_SetPullUp(GPIOA, PIN_KEY_BUTTON);
    Hardware_SetPinMode(GPIOB, PIN_CLUTCH, GPIO_MODE_INPUT);
    Hardware_SetPullUp(GPIOB, PIN_CLUTCH);
    Hardware_SetPinMode(GPIOB, PIN_BRAKE, GPIO_MODE_INPUT);
    Hardware_SetPullUp(GPIOB, PIN_BRAKE);
    Hardware_SetPinMode(GPIOB, PIN_REFUEL_BUTTON, GPIO_MODE_INPUT);
    Hardware_SetPullUp(GPIOB, PIN_REFUEL_BUTTON);

    /* LED: เขียว PB6 (สถานะ), ฟ้า PA5 (ถอยหลัง), แดง PA6 (เบรก), เหลือง PA7 (เตือนเครื่องผิดปกติ) */
    Hardware_SetPinMode(GPIOB, PIN_STATUS_LED, GPIO_MODE_OUTPUT);
    Hardware_SetPinMode(GPIOA, PIN_REVERSE_LED, GPIO_MODE_OUTPUT);
    Hardware_SetPinMode(GPIOA, PIN_BRAKE_LED, GPIO_MODE_OUTPUT);
    Hardware_SetPinMode(GPIOA, PIN_WARNING_LED, GPIO_MODE_OUTPUT);

    /* Buzzer PC0 (Output เริ่มต้นเงียบ) */
    Hardware_SetPinMode(GPIOC, PIN_BUZZER, GPIO_MODE_OUTPUT);
    Driver_SetOutput(GPIOC, PIN_BUZZER, 0u);

    /* Analog Inputs: คันเร่ง PA0, LDR PA1, สตาร์ท PA4, Joystick PC2/PC3 */
    Hardware_SetPinMode(GPIOA, PIN_THROTTLE, GPIO_MODE_ANALOG);
    Hardware_SetPinMode(GPIOA, PIN_LDR, GPIO_MODE_ANALOG);
    Hardware_SetPinMode(GPIOA, PIN_IGNITION, GPIO_MODE_ANALOG);
    Hardware_SetPinMode(GPIOC, PIN_JOY_X, GPIO_MODE_ANALOG);
    Hardware_SetPinMode(GPIOC, PIN_JOY_Y, GPIO_MODE_ANALOG);

    /* 7-Segment BCD: PC7(2^0), PA8(2^1), PB10(2^2), PA9(2^3) */
    Hardware_SetPinMode(GPIOC, PIN_SEG_BIT0, GPIO_MODE_OUTPUT);
    Hardware_SetPinMode(GPIOA, PIN_SEG_BIT1, GPIO_MODE_OUTPUT);
    Hardware_SetPinMode(GPIOB, PIN_SEG_BIT2, GPIO_MODE_OUTPUT);
    Hardware_SetPinMode(GPIOA, PIN_SEG_BIT3, GPIO_MODE_OUTPUT);

    /* PA2: USART2 TX (AF7) */
    Hardware_SetPinMode(GPIOA, PIN_UART_TX, GPIO_MODE_AF);
    GPIOA->AFR[0] &= ~(GPIO_AF_MASK << (PIN_UART_TX * GPIO_AF_FIELD_BITS));
    GPIOA->AFR[0] |= (GPIO_AF7_USART2 << (PIN_UART_TX * GPIO_AF_FIELD_BITS));
}

void Hardware_TIM2_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    /* Timer 2: ตั้งจังหวะ 10ms (100Hz) @ HSI 16MHz */
    TIM2->PSC = TIM2_PRESCALER_1KHZ;
    TIM2->ARR = TIM2_RELOAD_10MS;
    TIM2->CNT = 0u;

    TIM2->DIER |= TIM_DIER_UIE;
    NVIC_EnableIRQ(TIM2_IRQn);

    TIM2->CR1 |= TIM_CR1_CEN;
}

/* Timer 3: นับ 1MHz ใช้ toggle ขา buzzer (เปิด/ปิดโดย Driver_BuzzerOn/Off) */
void Hardware_TIM3_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    TIM3->PSC = TIM3_PRESCALER_1MHZ;
    TIM3->CR1 |= TIM_CR1_ARPE;        /* เปลี่ยนความถี่ตอนจบคาบ เสียงไม่สะดุด */
    TIM3->DIER |= TIM_DIER_UIE;
    NVIC_EnableIRQ(TIM3_IRQn);
}

void Hardware_ADC1_DMA_Init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

    /* ตั้ง Sample Time สูงสุดสำหรับทุก Channel */
    ADC1->SMPR2 |= (ADC_SMPR2_SMP0 | ADC_SMPR2_SMP1 | ADC_SMPR2_SMP4);
    ADC1->SMPR1 |= (ADC_SMPR1_SMP12 | ADC_SMPR1_SMP13);

    /* Sequence Length = 5 Conversions (L = 4) */
    ADC1->SQR1 &= ~(ADC_SQR1_L);
    ADC1->SQR1 |= ((ADC_SCAN_LENGTH - 1u) << ADC_SQR1_L_Pos);

    /* ลำดับการอ่าน Scan: SQ1=PA0(0), SQ2=PC2(12), SQ3=PC3(13), SQ4=PA4(4), SQ5=PA1(1) */
    ADC1->SQR3 &= ~(ADC_SQR3_SQ1 | ADC_SQR3_SQ2 | ADC_SQR3_SQ3 | ADC_SQR3_SQ4 | ADC_SQR3_SQ5);
    ADC1->SQR3 |= ((ADC_CH_THROTTLE << ADC_SQR3_SQ1_Pos) |
                   (ADC_CH_JOY_X << ADC_SQR3_SQ2_Pos) |
                   (ADC_CH_JOY_Y << ADC_SQR3_SQ3_Pos) |
                   (ADC_CH_IGNITION << ADC_SQR3_SQ4_Pos) |
                   (ADC_CH_LDR << ADC_SQR3_SQ5_Pos));

    /* เปิดโหมด Scan และ DMA */
    ADC1->CR1 |= ADC_CR1_SCAN;
    ADC1->CR2 |= (ADC_CR2_DMA | ADC_CR2_DDS | ADC_CR2_ADON);

    /* ตั้งค่า DMA2 Stream 0 Channel 0 สำหรับ ADC1 */
    DMA2_Stream0->CR &= ~DMA_SxCR_EN;
    while ((DMA2_Stream0->CR & DMA_SxCR_EN) != 0u)
    {
        /* รอให้ Stream หยุดก่อนตั้งค่า (ครั้งเดียวตอนเริ่มระบบ) */
    }

    DMA2_Stream0->PAR  = (uint32_t)&(ADC1->DR);
    DMA2_Stream0->M0AR = (uint32_t)g_adcBuffer;
    DMA2_Stream0->NDTR = ADC_SCAN_LENGTH;

    DMA2_Stream0->CR = ((DMA_PRIORITY_MEDIUM << DMA_SxCR_PL_Pos) |
                        (DMA_SIZE_HALFWORD << DMA_SxCR_MSIZE_Pos) |
                        (DMA_SIZE_HALFWORD << DMA_SxCR_PSIZE_Pos) |
                        DMA_SxCR_MINC |
                        DMA_SxCR_CIRC);       /* Channel 0, Memory increment, Circular */

    DMA2_Stream0->CR |= DMA_SxCR_EN;
}

void Hardware_EXTI_Init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    /* แมปขาเข้า EXTI Lines: PB3->EXTI3, PB4->EXTI4, PB5->EXTI5, PA10->EXTI10 */
    Hardware_MapExtiLine(PIN_CLUTCH, EXTI_PORT_B);
    Hardware_MapExtiLine(PIN_BRAKE, EXTI_PORT_B);
    Hardware_MapExtiLine(PIN_REFUEL_BUTTON, EXTI_PORT_B);
    Hardware_MapExtiLine(PIN_KEY_BUTTON, EXTI_PORT_A);

    /* Trigger: PA10 (Falling Edge), PB3 / PB4 / PB5 (Both Edges) */
    EXTI->FTSR |= (EXTI_FTSR_TR10 | EXTI_FTSR_TR3 | EXTI_FTSR_TR4 | EXTI_FTSR_TR5);
    EXTI->RTSR |= (EXTI_RTSR_TR3 | EXTI_RTSR_TR4 | EXTI_RTSR_TR5);

    /* อ่านค่าแรกเริ่ม */
    g_clutchPressedLive = Driver_ReadClutchRaw();
    g_brakePressedLive  = Driver_ReadBrakeRaw();
    g_refuelButtonLive  = Driver_ReadRefuelButtonRaw();

    EXTI->PR = (EXTI_PR_PR3 | EXTI_PR_PR4 | EXTI_PR_PR5 | EXTI_PR_PR10);
    EXTI->IMR |= (EXTI_IMR_MR3 | EXTI_IMR_MR4 | EXTI_IMR_MR5 | EXTI_IMR_MR10);

    NVIC_EnableIRQ(EXTI3_IRQn);
    NVIC_EnableIRQ(EXTI4_IRQn);
    NVIC_EnableIRQ(EXTI9_5_IRQn);
    NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void Hardware_UART2_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    /* Baud Rate 9600 @ 16MHz */
    USART2->BRR = USART2_BRR_9600;
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

        g_systemTickMs += CONTROL_LOOP_PERIOD_MS;
        g_controlLoopFlag = 1u;

        /* สั่ง ADC1 Scan ทั้ง 5 ช่อง ผลลัพธ์ไหลเข้า g_adcBuffer ผ่าน DMA */
        ADC1->CR2 |= ADC_CR2_SWSTART;
    }
    else
    {
        /* No action */
    }
}

/* TIM3 Interrupt (ทุกครึ่งคาบเสียง) -> กลับสถานะขา buzzer ได้คลื่นสี่เหลี่ยม */
void TIM3_IRQHandler(void)
{
    if ((TIM3->SR & TIM_SR_UIF) != 0u)
    {
        TIM3->SR = ~TIM_SR_UIF;
        GPIOC->ODR ^= (1u << PIN_BUZZER);
    }
    else
    {
        /* No action */
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
    else
    {
        /* No action */
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
    else
    {
        /* No action */
    }
}

/* EXTI9_5 ISR (ปุ่มเติมน้ำมัน PB5) */
void EXTI9_5_IRQHandler(void)
{
    if ((EXTI->PR & EXTI_PR_PR5) != 0u)
    {
        EXTI->PR = EXTI_PR_PR5;
        g_refuelButtonLive = Driver_ReadRefuelButtonRaw();
    }
    else
    {
        /* No action */
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

            /* ถอดกุญแจได้เฉพาะตอนเครื่องดับ และไม่อยู่ในสถานะดับผิดปกติ */
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
                else
                {
                    /* No action: เครื่องติดอยู่ ห้ามถอดกุญแจ */
                }
            }
            else
            {
                /* No action */
            }
        }
        else
        {
            /* No action */
        }
    }
    else
    {
        /* No action */
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
    else
    {
        /* No action */
    }
}

/* ==================================================================== */
/* 7. APPLICATION LAYER & LOGIC (ระบบฟิสิกส์และการควบคุม)                    */
/* ==================================================================== */

/* ดึงค่า ADC ล่าสุดจาก DMA Buffer */
void App_ReadAnalogInputs(void)
{
    /* คันเร่งกลับด้าน: หมุน pot ไปฝั่งค่า ADC สูง = ถอนคันเร่ง, ฝั่งค่าต่ำ = เหยียบเต็ม */
    g_throttleRaw = (uint16_t)(ADC_MAX_RAW - (uint32_t)g_adcBuffer[ADC_IDX_THROTTLE]);
    g_joyVrxRaw   = g_adcBuffer[ADC_IDX_JOY_X];
    g_joyVryRaw   = g_adcBuffer[ADC_IDX_JOY_Y];
    g_ignitionRaw = g_adcBuffer[ADC_IDX_IGNITION];
    g_ldrRaw      = g_adcBuffer[ADC_IDX_LDR];
}

/* Software Debounce: ค่าดิบต้องต่างจากค่าเดิมต่อเนื่อง BUTTON_DEBOUNCE_TICKS รอบจึงยอมเปลี่ยน */
uint8_t App_DebounceButton(uint8_t rawState, uint8_t stableState, uint8_t *counter)
{
    uint8_t newState = stableState;

    if (rawState != stableState)
    {
        (*counter)++;
        if (*counter >= BUTTON_DEBOUNCE_TICKS)
        {
            newState = rawState;
            *counter = 0u;
        }
        else
        {
            /* No action */
        }
    }
    else
    {
        *counter = 0u;
    }

    return newState;
}

void App_ProcessInputDebounce(void)
{
    static uint8_t clutchDebounceCount = 0u;
    static uint8_t brakeDebounceCount  = 0u;
    static uint8_t refuelDebounceCount = 0u;

    g_clutchPressed = App_DebounceButton(g_clutchPressedLive, g_clutchPressed, &clutchDebounceCount);
    g_brakePressed = App_DebounceButton(g_brakePressedLive, g_brakePressed, &brakeDebounceCount);
    g_refuelButtonPressed = App_DebounceButton(g_refuelButtonLive, g_refuelButtonPressed, &refuelDebounceCount);
}

void App_UpdateIgnitionSystem(void)
{
    if (g_engineStalled == 1u)
    {
        /* ดับผิดปกติ: ต้องหมุน pot สตาร์ทกลับตำแหน่งดับก่อนถึงจะปลดล็อก */
        if (g_ignitionRaw < HALF_ADC_VALUE)
        {
            g_engineStalled = 0u;
        }
        else
        {
            /* No action */
        }
    }
    else if ((g_keyInserted == 1u) && (g_ignitionRaw >= HALF_ADC_VALUE) && (g_fuelLiters > 0.0f))
    {
        g_engineStarted = 1u;
        g_stallReason = STALL_NONE;
    }
    else
    {
        /* ไม่มีกุญแจ / pot ตำแหน่งดับ / น้ำมันหมด -> สตาร์ทไม่ติด */
        g_engineStarted = 0u;
    }
}

void App_UpdateStatusLed(void)
{
    uint32_t blinkTicks = BLINK_NORMAL_TICKS;

    if (g_engineStalled == 1u)
    {
        blinkTicks = BLINK_STALL_TICKS;
    }
    else
    {
        /* No action */
    }

    if ((g_engineStalled == 1u) || ((g_keyInserted == 1u) && (g_engineStarted == 0u)))
    {
        g_blinkTickCounter++;
        if (g_blinkTickCounter >= blinkTicks)
        {
            g_blinkTickCounter = 0u;
            g_ledBlinkState ^= 1u;
            Driver_SetOutput(GPIOB, PIN_STATUS_LED, g_ledBlinkState);
        }
        else
        {
            /* No action */
        }
    }
    else if (g_engineStarted == 1u)
    {
        Driver_SetOutput(GPIOB, PIN_STATUS_LED, 1u);
        g_blinkTickCounter = 0u;
    }
    else
    {
        Driver_SetOutput(GPIOB, PIN_STATUS_LED, 0u);
        g_blinkTickCounter = 0u;
    }
}

/* ไฟเบรก (แดง PA6) ตามปุ่มเบรก, ไฟถอยหลัง (ฟ้า PA5) ตามเกียร์ R, ไฟเตือน (เหลือง PA7) กะพริบเมื่อมีคำเตือน */
void App_UpdateLamps(void)
{
    Driver_SetOutput(GPIOA, PIN_BRAKE_LED, g_brakePressed);

    if (g_currentGear == GEAR_REVERSE)
    {
        Driver_SetOutput(GPIOA, PIN_REVERSE_LED, 1u);
    }
    else
    {
        Driver_SetOutput(GPIOA, PIN_REVERSE_LED, 0u);
    }

    if (g_warningCode != WARN_NONE)
    {
        g_warningBlinkTicks++;
        if (g_warningBlinkTicks >= WARNING_BLINK_TICKS)
        {
            g_warningBlinkTicks = 0u;
            g_warningLedState ^= 1u;
        }
        else
        {
            /* No action */
        }
    }
    else
    {
        g_warningBlinkTicks = 0u;
        g_warningLedState = 0u;
    }

    Driver_SetOutput(GPIOA, PIN_WARNING_LED, g_warningLedState);
}

/*
 * แปลงตำแหน่ง Joystick เป็นเกียร์ (แกน Y กลาง = เกียร์ว่าง)
 * Hysteresis: ถ้าเกียร์ปัจจุบันอยู่ฝั่งไหนของเส้นแบ่งแล้ว เส้นนั้นจะถูกขยับออกไป JOY_HYSTERESIS
 * ทำให้ต้องโยกกลับชัดเจนจริงถึงจะหลุดเกียร์ (แถวล่าง 2/4/R มีระยะจากเส้นแค่ 120 จึงหลุดง่าย)
 */
uint8_t App_ComputeCurrentGear(void)
{
    int32_t offsetX = (int32_t)g_joyVrxRaw - g_centerX;
    int32_t offsetY = (int32_t)g_joyVryRaw - g_centerY;
    int32_t lowLimitX = JOY_LOW_LIMIT;
    int32_t highLimitX = JOY_HIGH_LIMIT;
    int32_t lowLimitY = JOY_LOW_LIMIT;
    int32_t highLimitY = JOY_HIGH_LIMIT;
    uint8_t gear = GEAR_NEUTRAL;

    if ((g_currentGear == GEAR_1) || (g_currentGear == GEAR_3) || (g_currentGear == GEAR_5))
    {
        lowLimitY = JOY_LOW_LIMIT + JOY_HYSTERESIS;
    }
    else if ((g_currentGear == GEAR_2) || (g_currentGear == GEAR_4) || (g_currentGear == GEAR_REVERSE))
    {
        highLimitY = JOY_HIGH_LIMIT - JOY_HYSTERESIS;
    }
    else
    {
        /* No action: เกียร์ว่างใช้เส้นปกติ */
    }

    if ((g_currentGear == GEAR_1) || (g_currentGear == GEAR_2))
    {
        lowLimitX = JOY_LOW_LIMIT + JOY_HYSTERESIS;
    }
    else if ((g_currentGear == GEAR_5) || (g_currentGear == GEAR_REVERSE))
    {
        highLimitX = JOY_HIGH_LIMIT - JOY_HYSTERESIS;
    }
    else
    {
        /* No action: คอลัมน์กลางใช้เส้นปกติ */
    }

    if (offsetY < lowLimitY)
    {
        /* แถวบน: เกียร์ 1 / 3 / 5 */
        if (offsetX < lowLimitX)
        {
            gear = GEAR_1;
        }
        else if (offsetX <= highLimitX)
        {
            gear = GEAR_3;
        }
        else
        {
            gear = GEAR_5;
        }
    }
    else if (offsetY > highLimitY)
    {
        /* แถวล่าง: เกียร์ 2 / 4 / R */
        if (offsetX < lowLimitX)
        {
            gear = GEAR_2;
        }
        else if (offsetX <= highLimitX)
        {
            gear = GEAR_4;
        }
        else
        {
            gear = GEAR_REVERSE;
        }
    }
    else
    {
        gear = GEAR_NEUTRAL;
    }

    return gear;
}

/*
 * ยอมเปลี่ยนเกียร์เมื่อตำแหน่งใหม่อ่านได้เหมือนเดิมต่อเนื่อง GEAR_CONFIRM_TICKS รอบ
 * เครื่องติดอยู่และไม่เหยียบคลัทช์: โยกเข้าเกียร์ไม่ได้ (เฟืองขบ ไม่ดับ), โยกออกเกียร์ว่างได้
 */
void App_UpdateGearSelection(void)
{
    static uint8_t candidateGear = GEAR_NEUTRAL;
    static uint8_t candidateTicks = 0u;
    uint8_t rawGear = GEAR_NEUTRAL;

    g_gearGrindActive = 0u;

    if (g_refuelActive == 1u)
    {
        /* โหมดเติมน้ำมัน: ล็อกเกียร์ว่าง ไม่สนตำแหน่ง Joystick */
        g_currentGear = GEAR_NEUTRAL;
        candidateTicks = 0u;
    }
    else
    {
        rawGear = App_ComputeCurrentGear();

        if (rawGear == g_currentGear)
        {
            candidateTicks = 0u;
        }
        else if (rawGear == candidateGear)
        {
            if (candidateTicks < GEAR_CONFIRM_TICKS)
            {
                candidateTicks++;
            }
            else
            {
                /* No action */
            }

            if (candidateTicks >= GEAR_CONFIRM_TICKS)
            {
                if ((g_engineStarted == 1u) && (g_clutchPressed == 0u) && (rawGear != GEAR_NEUTRAL))
                {
                    /* เข้าเกียร์ไม่เหยียบคลัทช์: เฟืองขบ เกียร์ไม่เปลี่ยน รอจนเหยียบคลัทช์หรือโยกกลับ */
                    g_gearGrindActive = 1u;
                    App_RaiseWarning(WARN_GEAR_GRIND);
                }
                else
                {
                    g_currentGear = rawGear;
                    candidateTicks = 0u;
                }
            }
            else
            {
                /* No action */
            }
        }
        else
        {
            candidateGear = rawGear;
            candidateTicks = 1u;
        }
    }
}

void App_UpdateSegmentDisplay(void)
{
    if (g_refuelActive == 1u)
    {
        /* ช่วงแรกแสดง 0 ครึ่งวินาที แล้วแสดงลิตรที่กำลังจะเติม (0-9) */
        if (g_refuelZeroTicks > 0u)
        {
            Driver_SegmentDisplay(0u);
        }
        else
        {
            Driver_SegmentDisplay(g_refuelAddLiters);
        }
    }
    else
    {
        Driver_SegmentDisplay(g_currentGear);
    }
}

/* ------------------------------------------------------------------ */
/* โหมดเติมน้ำมัน (PB5 + LDR)                                           */
/* ------------------------------------------------------------------ */
void App_UpdateRefuelMode(void)
{
    uint8_t buttonPressedEdge = 0u;
    float newFuel = 0.0f;

    if ((g_refuelButtonPressed == 1u) && (g_previousRefuelButton == 0u))
    {
        buttonPressedEdge = 1u;
    }
    else
    {
        /* No action */
    }

    if (g_refuelActive == 0u)
    {
        /* เข้าโหมดได้เฉพาะเกียร์ว่างและรถหยุดสนิท (เครื่องติดหรือดับก็ได้) */
        if ((buttonPressedEdge == 1u) &&
            (g_currentGear == GEAR_NEUTRAL) &&
            (g_currentVelocity < SPEED_ZERO_KMH))
        {
            g_refuelActive = 1u;
            g_refuelZeroTicks = REFUEL_ZERO_SHOW_TICKS;
            g_refuelAddLiters = 0u;
            g_refuelLightTicks = 0u;
            g_refuelLightDetected = 0u;
            g_currentGear = GEAR_NEUTRAL;
        }
        else
        {
            /* No action */
        }
    }
    else if (buttonPressedEdge == 1u)
    {
        /* กดซ้ำ = ยืนยัน: น้ำมันใหม่ = น้ำมันเก่า + ลิตรที่เติม แล้วกลับโหมดขับ */
        newFuel = g_fuelLiters + (float)g_refuelAddLiters;
        g_fuelLiters = Physics_MinFloat(newFuel, FUEL_TANK_MAX_L);
        g_refuelActive = 0u;
    }
    else
    {
        App_CountRefuelLiters();
    }
}

/* นับลิตรจากแสง: แสงแฟลชค้าง 1 วินาที = 1 ลิตร */
void App_CountRefuelLiters(void)
{
    float lux = 0.0f;

    if (g_refuelZeroTicks > 0u)
    {
        g_refuelZeroTicks--;
    }
    else
    {
        lux = Sensor_LuxFromLdr(g_ldrRaw);

        if ((g_refuelLightDetected == 0u) && (lux >= FLASHLIGHT_LUX_ON))
        {
            g_refuelLightDetected = 1u;
        }
        else if ((g_refuelLightDetected == 1u) && (lux < FLASHLIGHT_LUX_OFF))
        {
            /* แสงหลุด: เริ่มนับวินาทีใหม่ (ลิตรที่นับแล้วไม่หาย) */
            g_refuelLightDetected = 0u;
            g_refuelLightTicks = 0u;
        }
        else
        {
            /* No action */
        }

        if ((g_refuelLightDetected == 1u) && (App_RefuelHasRoom() == 1u))
        {
            g_refuelLightTicks++;
            if (g_refuelLightTicks >= REFUEL_TICKS_PER_LITER)
            {
                g_refuelLightTicks = 0u;
                g_refuelAddLiters++;
            }
            else
            {
                /* No action */
            }
        }
        else
        {
            /* No action */
        }
    }
}

/* ยังเติมเพิ่มได้ไหม: ไม่เกิน 9 ลิตรต่อครั้ง และถังยังไม่เต็ม */
uint8_t App_RefuelHasRoom(void)
{
    uint8_t hasRoom = 0u;

    if ((g_refuelAddLiters < REFUEL_MAX_ADD_L) &&
        ((g_fuelLiters + (float)g_refuelAddLiters) < FUEL_TANK_MAX_L))
    {
        hasRoom = 1u;
    }
    else
    {
        /* No action */
    }

    return hasRoom;
}

/* ------------------------------------------------------------------ */
/* ระบบขับเคลื่อน (Drivetrain)                                         */
/* ------------------------------------------------------------------ */
void App_UpdateDrivetrain(void)
{
    float gearRatio = Physics_GearRatioFor(g_currentGear);
    float throttleRpm = Physics_ThrottleToRpm(g_throttleRaw);
    uint8_t powerConnected = 0u;

    /* กำลังส่งถึงล้อได้เมื่อ อยู่ในเกียร์ และ ไม่ได้เหยียบคลัทช์ */
    if ((g_currentGear != GEAR_NEUTRAL) && (g_clutchPressed == 0u))
    {
        powerConnected = 1u;
    }
    else
    {
        /* No action */
    }

    if (g_engineStarted == 0u)
    {
        App_UpdateEngineOff(powerConnected, gearRatio);
    }
    else if (powerConnected == 0u)
    {
        App_UpdatePowerDisconnected(throttleRpm);
    }
    else
    {
        /* จังหวะแรกที่เกียร์จับกำลัง: ตรวจเงื่อนไขดับก่อนเริ่มต่อคลัทช์ */
        if (g_drivelineEngaged == 0u)
        {
            App_EngageDriveline();
        }
        else
        {
            /* No action */
        }

        if (g_engineStarted == 1u)
        {
            if (g_clutchSlipActive == 1u)
            {
                App_UpdateClutchSlip(gearRatio, throttleRpm);
            }
            else
            {
                App_UpdateInGear(gearRatio, throttleRpm);
            }
        }
        else
        {
            /* No action */
        }
    }
}

/* เครื่องดับ: ค้างเกียร์ -> เครื่องฉุดรถ (แรงตามอัตราทด), ไม่งั้นไหลอิสระ; เบรกใช้ได้เสมอ */
void App_UpdateEngineOff(uint8_t powerConnected, float gearRatio)
{
    float decay = Physics_CoastDecay(g_currentVelocity);

    g_currentRpm = 0.0f;
    g_drivelineEngaged = 0u;
    g_clutchSlipActive = 0u;

    if (powerConnected == 1u)
    {
        decay = (ENGINE_DRAG_DECAY * gearRatio) + Physics_AirDrag(g_currentVelocity);
    }
    else
    {
        /* No action */
    }

    if (g_brakePressed == 1u)
    {
        decay += BRAKE_DECAY;
    }
    else
    {
        /* No action */
    }

    App_ReduceSpeed(decay);
}

/* ตัดกำลัง (เหยียบคลัทช์/เกียร์ว่าง): รอบตามคันเร่ง รถชะลอตามแรงต้าน (+เบรก) */
void App_UpdatePowerDisconnected(float throttleRpm)
{
    float decay = Physics_CoastDecay(g_currentVelocity);

    g_drivelineEngaged = 0u;
    g_clutchSlipActive = 0u;
    g_currentRpm = Physics_Slew(g_currentRpm, throttleRpm, FREE_REV_RISE_RATE, FREE_REV_FALL_RATE);

    if (g_brakePressed == 1u)
    {
        decay += BRAKE_DECAY;
    }
    else
    {
        /* No action */
    }

    App_ReduceSpeed(decay);
}

/* ตรวจเงื่อนไขตอนเกียร์เริ่มจับกำลัง (ปล่อยคลัทช์ หรือ สตาร์ทค้างเกียร์) */
void App_EngageDriveline(void)
{
    float wheelRpm = Physics_RpmFromVelocity(g_currentVelocity, Physics_GearRatioFor(g_currentGear));
    uint8_t wrongDirection = 0u;

    if (g_currentVelocity >= SPEED_STOPPED_KMH)
    {
        /* รถยังเคลื่อนที่: เกียร์ต้องตรงกับทิศทางที่รถกำลังไป */
        if ((g_currentGear == GEAR_REVERSE) && (g_travelDirection == DIRECTION_FORWARD))
        {
            wrongDirection = 1u;
        }
        else if ((g_currentGear != GEAR_REVERSE) && (g_travelDirection == DIRECTION_REVERSE))
        {
            wrongDirection = 1u;
        }
        else
        {
            /* No action */
        }
    }
    else if (g_currentGear == GEAR_REVERSE)
    {
        g_travelDirection = DIRECTION_REVERSE;
    }
    else
    {
        g_travelDirection = DIRECTION_FORWARD;
    }

    if (g_previousEngineStarted == 0u)
    {
        /* สตาร์ทเครื่องทั้งที่ค้างเกียร์และไม่เหยียบคลัทช์ -> ดับ */
        App_TriggerStall(STALL_START_IN_GEAR);
    }
    else if (wrongDirection == 1u)
    {
        /* เข้าเกียร์สวนทิศทางที่รถกำลังวิ่ง -> ดับ */
        App_TriggerStall(STALL_WRONG_DIRECTION);
    }
    else
    {
        g_drivelineEngaged = 1u;
        g_clutchSlipActive = 1u;

        if (g_currentVelocity >= SPEED_STOPPED_KMH)
        {
            App_ApplyEngagementJerk(wheelRpm, Physics_GearRatioFor(g_currentGear));
        }
        else
        {
            /* No action */
        }
    }
}

/*
 * กระชากตอนปล่อยคลัทช์ขณะรถไหล: ยิ่งรอบเครื่องกับรอบล้อต่างกันมาก ยิ่งกระชากแรง
 * เครื่องเร็วกว่าล้อ = รถพุ่ง, เครื่องช้ากว่าล้อ (ลดเกียร์ไม่ตบรอบ) = รถหน่วง
 * รอบล้อเกิน redline (ลดเกียร์ต่ำเกิน) = กระชากแรงสุด + เตือนรอบเกิน (ไม่ดับ)
 */
void App_ApplyEngagementJerk(float wheelRpm, float gearRatio)
{
    float mismatch = fabsf(g_currentRpm - wheelRpm);
    float engineVelocity = Physics_VelocityFromRpm(g_currentRpm, gearRatio);
    float kick = (engineVelocity - g_currentVelocity) * JERK_FRACTION;
    float intensity = 0.0f;

    if ((mismatch > JERK_FREE_RPM) || (wheelRpm > REDLINE_RPM))
    {
        kick = Physics_MaxFloat(kick, -JERK_MAX_KMH);
        kick = Physics_MinFloat(kick, JERK_MAX_KMH);
        g_currentVelocity = Physics_MaxFloat(g_currentVelocity + kick, 0.0f);

        if (wheelRpm > REDLINE_RPM)
        {
            intensity = JERK_INTENSITY_MAX;
            App_RaiseWarning(WARN_OVERREV);
        }
        else
        {
            intensity = Physics_MinFloat((mismatch / JERK_FULL_RPM) * JERK_INTENSITY_MAX, JERK_INTENSITY_MAX);
            App_RaiseWarning(WARN_JERK);
        }

        g_jerkIntensity = (uint8_t)intensity;
        g_jerkHoldTicks = WARNING_HOLD_TICKS;
    }
    else
    {
        /* No action */
    }
}

/*
 * เครื่องยนต์พยุงรถไว้ได้ไหมขณะคลัทช์กำลังจับ
 * - รอบล้อ >= 600 : ได้เสมอ
 * - เกียร์ 1 / R  : ได้เสมอ (ออกตัวด้วยรอบเดินเบาได้)
 * - เกียร์ 2      : ต้องเร่งเกิน 1200 rpm
 * - เกียร์ 3-5    : ต้องเหยียบคันเร่งและรถต้องไหลอยู่แล้ว (ออกตัวจากหยุดนิ่งไม่ได้)
 */
uint8_t App_EngineCanHoldSlip(float wheelRpm, float throttleRpm)
{
    uint8_t canHold = 0u;

    if (wheelRpm >= STALL_RPM_THRESHOLD)
    {
        canHold = 1u;
    }
    else if ((g_currentGear == GEAR_1) || (g_currentGear == GEAR_REVERSE))
    {
        canHold = 1u;
    }
    else if (g_currentGear == GEAR_2)
    {
        if (throttleRpm > GEAR2_LAUNCH_RPM_MIN)
        {
            canHold = 1u;
        }
        else
        {
            /* No action */
        }
    }
    else
    {
        if ((g_throttleRaw > THROTTLE_IDLE_RAW_MAX) && (wheelRpm >= SLIP_MIN_WHEEL_RPM))
        {
            canHold = 1u;
        }
        else
        {
            /* No action */
        }
    }

    return canHold;
}

/* คลัทช์กำลังจับ: รถเร่ง/ชะลอเข้าหาความเร็วเป้าหมาย รอบเครื่องค่อยๆ วิ่งเข้าหารอบล้อ */
void App_UpdateClutchSlip(float gearRatio, float throttleRpm)
{
    float wheelRpm = Physics_RpmFromVelocity(g_currentVelocity, gearRatio);
    float engineTarget = 0.0f;
    float targetVelocity = 0.0f;

    if (wheelRpm > REDLINE_RPM)
    {
        /* รอบเกิน: ล้อปั่นเครื่องเกิน redline เครื่องฉุดรถหน่วงแรงจนรอบลงมาถึง redline */
        App_ReduceSpeed(OVERREV_DECAY);
        g_currentRpm = Physics_MinFloat(Physics_RpmFromVelocity(g_currentVelocity, gearRatio), REV_DISPLAY_MAX);
        App_RaiseWarning(WARN_OVERREV);
    }
    else if (App_EngineCanHoldSlip(wheelRpm, throttleRpm) == 0u)
    {
        App_TriggerStall(STALL_LUGGING);
    }
    else if ((g_brakePressed == 1u) && (wheelRpm < STALL_RPM_THRESHOLD))
    {
        /* เบรกค้างขณะปล่อยคลัทช์ รอบถูกกดต่ำ -> ดับ */
        App_TriggerStall(STALL_BRAKE_LUG);
    }
    else
    {
        if (g_brakePressed == 1u)
        {
            App_ReduceSpeed(BRAKE_DECAY + Physics_CoastDecay(g_currentVelocity));
        }
        else
        {
            targetVelocity = Physics_VelocityFromRpm(Physics_MaxFloat(throttleRpm, IDLE_RPM), gearRatio);
            App_ApproachTargetSpeed(targetVelocity, gearRatio);
        }

        wheelRpm = Physics_RpmFromVelocity(g_currentVelocity, gearRatio);

        /* ล้อยังช้ากว่ารอบเดินเบา: เครื่องถูกพยุงด้วยคันเร่ง/รอบเดินเบา, ไม่งั้นถูกดึงเข้าหารอบล้อ */
        if (wheelRpm >= IDLE_RPM)
        {
            engineTarget = wheelRpm;
        }
        else
        {
            engineTarget = Physics_MaxFloat(throttleRpm, IDLE_RPM);
        }

        g_currentRpm = Physics_Slew(g_currentRpm, engineTarget, CLUTCH_ENGAGE_RATE, CLUTCH_ENGAGE_RATE);

        /* รอบเครื่องเท่ารอบล้อแล้ว -> คลัทช์จับสนิท */
        if ((wheelRpm >= (IDLE_RPM - CLUTCH_LOCK_TOLERANCE_RPM)) &&
            (fabsf(g_currentRpm - wheelRpm) <= CLUTCH_LOCK_TOLERANCE_RPM))
        {
            g_clutchSlipActive = 0u;
            g_currentRpm = wheelRpm;
        }
        else
        {
            /* No action */
        }
    }
}

/* เกียร์จับสนิท: รอบเครื่อง = รอบล้อ */
void App_UpdateInGear(float gearRatio, float throttleRpm)
{
    float targetVelocity = 0.0f;

    if (g_brakePressed == 1u)
    {
        /* เบรกโดยไม่เหยียบคลัทช์: รอบตกตามความเร็ว ต่ำกว่า 600 rpm -> ดับ */
        App_ReduceSpeed(BRAKE_DECAY + Physics_EngineBrakeDecay(g_currentVelocity, gearRatio));
        g_currentRpm = Physics_RpmFromVelocity(g_currentVelocity, gearRatio);

        if (g_currentRpm < STALL_RPM_THRESHOLD)
        {
            App_TriggerStall(STALL_BRAKE_LUG);
        }
        else
        {
            /* No action */
        }
    }
    else
    {
        /* เป้าหมายคือความเร็วตามรอบคันเร่ง (ไม่ต่ำกว่ารอบเดินเบา = Idle Creep) */
        targetVelocity = Physics_VelocityFromRpm(Physics_MaxFloat(throttleRpm, IDLE_RPM), gearRatio);
        App_ApproachTargetSpeed(targetVelocity, gearRatio);
        g_currentRpm = Physics_RpmFromVelocity(g_currentVelocity, gearRatio);

        /* เครื่องหอบ: รอบต่ำแต่เหยียบคันเร่งหนัก -> รอบแกว่ง (buzzer สั่นตาม) เร่งช้าจากกราฟแรงบิด */
        if ((g_throttleRaw >= LUG_THROTTLE_MIN) && (g_currentRpm < LUG_RPM_MAX))
        {
            g_currentRpm += App_LugWobbleRpm();
            App_RaiseWarning(WARN_LUGGING);
        }
        else
        {
            /* No action */
        }
    }
}

/* ช้ากว่าเป้า -> เร่งด้วยอัตราจำกัดตามเกียร์และแรงบิดที่รอบนั้น, เร็วกว่าเป้า -> Engine Brake แต่ไม่ต่ำกว่าเป้า */
void App_ApproachTargetSpeed(float targetVelocity, float gearRatio)
{
    float step = 0.0f;

    if (g_currentVelocity < targetVelocity)
    {
        step = Physics_MinFloat(ACCEL_PER_RATIO * gearRatio * Physics_TorqueFactor(g_currentRpm),
                                targetVelocity - g_currentVelocity);
        g_currentVelocity += step;
    }
    else
    {
        step = Physics_MinFloat(Physics_EngineBrakeDecay(g_currentVelocity, gearRatio),
                                g_currentVelocity - targetVelocity);
        g_currentVelocity -= step;
    }
}

/* ลำดับความสำคัญของคำเตือน (ตัวสำคัญกว่าแทนที่ตัวที่น้อยกว่าได้) */
uint8_t App_WarningPriority(uint8_t code)
{
    uint8_t priority = WARN_PRIORITY_NONE;

    switch (code)
    {
        case WARN_LUGGING:
            priority = WARN_PRIORITY_LUGGING;
            break;
        case WARN_GEAR_GRIND:
            priority = WARN_PRIORITY_GEAR_GRIND;
            break;
        case WARN_JERK:
            priority = WARN_PRIORITY_JERK;
            break;
        case WARN_OVERREV:
            priority = WARN_PRIORITY_OVERREV;
            break;
        default:
            priority = WARN_PRIORITY_NONE;
            break;
    }

    return priority;
}

/* ตั้งคำเตือนค้างไว้ WARNING_HOLD_TICKS (ถ้าสำคัญเท่าหรือมากกว่าตัวที่แสดงอยู่) */
void App_RaiseWarning(uint8_t code)
{
    if (App_WarningPriority(code) >= App_WarningPriority(g_warningCode))
    {
        g_warningCode = code;
        g_warningHoldTicks = WARNING_HOLD_TICKS;
    }
    else
    {
        /* No action */
    }
}

/* นับเวลาค้างคำเตือน/ความแรงกระชาก ทุก 10ms หมดเวลาแล้วล้าง */
void App_TickWarnings(void)
{
    if (g_warningHoldTicks > 0u)
    {
        g_warningHoldTicks--;
    }
    else
    {
        g_warningCode = WARN_NONE;
    }

    if (g_jerkHoldTicks > 0u)
    {
        g_jerkHoldTicks--;
    }
    else
    {
        g_jerkIntensity = 0u;
    }
}

/* รอบแกว่งแบบสามเหลี่ยม -40 ... +40 rpm คาบ 80ms ตอนเครื่องหอบ */
float App_LugWobbleRpm(void)
{
    int32_t phase = 0;
    int32_t offset = 0;

    g_lugWobbleTick = (uint8_t)((g_lugWobbleTick + 1u) % LUG_WOBBLE_PERIOD_TICKS);
    phase = (int32_t)g_lugWobbleTick;

    if (phase > (int32_t)LUG_WOBBLE_HALF_TICKS)
    {
        phase = (int32_t)LUG_WOBBLE_PERIOD_TICKS - phase;
    }
    else
    {
        /* No action */
    }

    offset = (phase * LUG_WOBBLE_STEP_RPM) - LUG_WOBBLE_RPM;

    return (float)offset;
}

/* ลดความเร็ว ไม่ให้ติดลบ หยุดสนิทแล้วล้างทิศทาง */
void App_ReduceSpeed(float decay)
{
    if (g_currentVelocity > decay)
    {
        g_currentVelocity -= decay;
    }
    else
    {
        g_currentVelocity = 0.0f;
        g_travelDirection = DIRECTION_STOPPED;
    }
}

void App_TriggerStall(uint8_t reason)
{
    g_stallReason = reason;
    g_engineStalled = 1u;
    g_engineStarted = 0u;
    g_currentRpm = 0.0f;
    g_drivelineEngaged = 0u;
    g_clutchSlipActive = 0u;
}

/* ------------------------------------------------------------------ */
/* น้ำมัน และ Buzzer                                                    */
/* ------------------------------------------------------------------ */

/* เครื่องติด -> กินน้ำมันตามรอบ, หมดถัง -> เครื่องดับ */
void App_UpdateFuel(void)
{
    float burn = 0.0f;

    if (g_engineStarted == 1u)
    {
        burn = (FUEL_IDLE_LPS + (g_currentRpm * FUEL_PER_RPM_LPS)) * TICK_SECONDS;

        if (g_fuelLiters > burn)
        {
            g_fuelLiters -= burn;
        }
        else
        {
            g_fuelLiters = 0.0f;
            App_TriggerStall(STALL_OUT_OF_FUEL);
        }
    }
    else
    {
        /* No action */
    }
}

/* ดังตอนสตาร์ท (ไต่ความถี่ขึ้น) แล้วดังตามรอบเครื่องตลอดเวลาที่เครื่องติด, เฟืองขบ = เสียงครืด */
void App_UpdateBuzzer(void)
{
    static uint8_t grindToggle = 0u;
    float frequency = 0.0f;
    float progress = 0.0f;

    if (g_engineStarted == 0u)
    {
        g_buzzerStartTicks = 0u;
        Driver_BuzzerOff();
    }
    else
    {
        if (g_previousEngineStarted == 0u)
        {
            g_buzzerStartTicks = BUZZER_START_TICKS;
        }
        else
        {
            /* No action */
        }

        if (g_buzzerStartTicks > 0u)
        {
            progress = (float)(BUZZER_START_TICKS - g_buzzerStartTicks) / (float)BUZZER_START_TICKS;
            frequency = BUZZER_START_HZ + ((BUZZER_HZ_AT_IDLE - BUZZER_START_HZ) * progress);
            g_buzzerStartTicks--;
        }
        else if (g_gearGrindActive == 1u)
        {
            /* เฟืองขบ: สลับสองความถี่ทุก tick ให้เสียงครืด */
            grindToggle ^= 1u;
            if (grindToggle == 1u)
            {
                frequency = GRIND_HZ_A;
            }
            else
            {
                frequency = GRIND_HZ_B;
            }
        }
        else
        {
            frequency = Sound_FrequencyFromRpm(g_currentRpm);
        }

        Driver_BuzzerOn(frequency);
    }
}

void App_ReportDashboard(void)
{
    g_uartReportTickCounter++;

    if (g_uartReportTickCounter >= UART_REPORT_PERIOD_TICKS)
    {
        g_uartReportTickCounter = 0u;

        /* ส่งข้อมูล CSV: <RPM>,<VELOCITY>,<FUEL_LITERS>,<STALL_REASON>,<WARNING>,<JERK>\r\n */
        Driver_UART_SendFloatOneDecimal(g_currentRpm);
        Driver_UART_SendChar(',');
        Driver_UART_SendFloatOneDecimal(g_currentVelocity);
        Driver_UART_SendChar(',');
        Driver_UART_SendFloatOneDecimal(g_fuelLiters);
        Driver_UART_SendChar(',');
        Driver_UART_SendNumber((int32_t)g_stallReason);
        Driver_UART_SendChar(',');
        Driver_UART_SendNumber((int32_t)g_warningCode);
        Driver_UART_SendChar(',');
        Driver_UART_SendNumber((int32_t)g_jerkIntensity);
        Driver_UART_SendString("\r\n");
    }
    else
    {
        /* No action */
    }
}

/* ==================================================================== */
/* 8. HELPERS & UTILITIES (สูตรคำนวณและฟังก์ชันช่วย)                          */
/* ==================================================================== */

uint8_t Driver_ReadClutchRaw(void)
{
    uint8_t pressed = 0u;

    if ((GPIOB->IDR & GPIO_IDR_ID3) == 0u)
    {
        pressed = 1u;
    }
    else
    {
        /* No action */
    }

    return pressed;
}

uint8_t Driver_ReadBrakeRaw(void)
{
    uint8_t pressed = 0u;

    if ((GPIOB->IDR & GPIO_IDR_ID4) == 0u)
    {
        pressed = 1u;
    }
    else
    {
        /* No action */
    }

    return pressed;
}

uint8_t Driver_ReadRefuelButtonRaw(void)
{
    uint8_t pressed = 0u;

    if ((GPIOB->IDR & GPIO_IDR_ID5) == 0u)
    {
        pressed = 1u;
    }
    else
    {
        /* No action */
    }

    return pressed;
}

float Physics_ThrottleToRpm(uint16_t rawValue)
{
    float ratio = (float)rawValue / ADC_MAX_VALUE;

    return (IDLE_RPM + (ratio * (REDLINE_RPM - IDLE_RPM)));
}

float Physics_GearRatioFor(uint8_t gearDigit)
{
    float ratio = GEAR_RATIO_4;

    switch (gearDigit)
    {
        case GEAR_1:
            ratio = GEAR_RATIO_1;
            break;
        case GEAR_2:
            ratio = GEAR_RATIO_2;
            break;
        case GEAR_3:
            ratio = GEAR_RATIO_3;
            break;
        case GEAR_4:
            ratio = GEAR_RATIO_4;
            break;
        case GEAR_5:
            ratio = GEAR_RATIO_5;
            break;
        case GEAR_REVERSE:
            ratio = GEAR_RATIO_REVERSE;
            break;
        default:
            ratio = GEAR_RATIO_4;
            break;
    }

    return ratio;
}

float Physics_VelocityFromRpm(float rpm, float gearRatio)
{
    return ((rpm * KMH_FACTOR) / (TIRE_FACTOR * gearRatio * FINAL_DRIVE));
}

float Physics_RpmFromVelocity(float velocity, float gearRatio)
{
    return ((velocity * TIRE_FACTOR * gearRatio * FINAL_DRIVE) / KMH_FACTOR);
}

float Physics_AirDrag(float velocity)
{
    return (AIR_DRAG_K * velocity * velocity);
}

float Physics_CoastDecay(float velocity)
{
    return (COAST_DECAY + Physics_AirDrag(velocity));
}

float Physics_EngineBrakeDecay(float velocity, float gearRatio)
{
    return (ENGINE_BRAKE_BASE + (ENGINE_BRAKE_PER_RATIO * gearRatio) + Physics_AirDrag(velocity));
}

/* ขยับค่าเข้าหาเป้าหมายด้วยอัตราจำกัด (ขาขึ้น/ขาลงแยกกัน) */
float Physics_Slew(float current, float target, float riseRate, float fallRate)
{
    float result = target;

    if (current < (target - riseRate))
    {
        result = current + riseRate;
    }
    else if (current > (target + fallRate))
    {
        result = current - fallRate;
    }
    else
    {
        result = target;
    }

    return result;
}

float Physics_MaxFloat(float a, float b)
{
    float result = b;

    if (a > b)
    {
        result = a;
    }
    else
    {
        /* No action */
    }

    return result;
}

float Physics_MinFloat(float a, float b)
{
    float result = b;

    if (a < b)
    {
        result = a;
    }
    else
    {
        /* No action */
    }

    return result;
}

/* กราฟแรงบิด: รอบต่ำแรงน้อย -> เต็มช่วงกลาง -> ตกใกล้ redline (เส้นตรงเป็นช่วง) */
float Physics_TorqueFactor(float rpm)
{
    float factor = TORQUE_PEAK_FACTOR;
    float position = 0.0f;

    if (rpm <= TORQUE_LOW_RPM)
    {
        factor = TORQUE_LOW_FACTOR;
    }
    else if (rpm < TORQUE_PEAK_START_RPM)
    {
        position = (rpm - TORQUE_LOW_RPM) / (TORQUE_PEAK_START_RPM - TORQUE_LOW_RPM);
        factor = TORQUE_LOW_FACTOR + ((TORQUE_PEAK_FACTOR - TORQUE_LOW_FACTOR) * position);
    }
    else if (rpm <= TORQUE_PEAK_END_RPM)
    {
        factor = TORQUE_PEAK_FACTOR;
    }
    else if (rpm < REDLINE_RPM)
    {
        position = (rpm - TORQUE_PEAK_END_RPM) / (REDLINE_RPM - TORQUE_PEAK_END_RPM);
        factor = TORQUE_PEAK_FACTOR + ((TORQUE_REDLINE_FACTOR - TORQUE_PEAK_FACTOR) * position);
    }
    else
    {
        factor = TORQUE_REDLINE_FACTOR;
    }

    return factor;
}

/* แปลงค่า ADC ของ LDR เป็นความสว่าง (Lux) ด้วยสูตรเดียวกับ Lab 4.3 */
float Sensor_LuxFromLdr(uint16_t rawValue)
{
    float voltage = ((float)rawValue * LDR_VREF) / ADC_MAX_VALUE;
    float resistance = 0.0f;
    float lux = 0.0f;

    if (voltage <= LDR_MIN_VOLT)
    {
        lux = LUX_SATURATED;
    }
    else if (voltage >= (LDR_VREF - LDR_MIN_VOLT))
    {
        lux = 0.0f;
    }
    else
    {
        resistance = (LDR_RX_OHM * voltage) / (LDR_VREF - voltage);
        lux = powf(LUX_BASE, ((LDR_SLOPE * log10f(resistance)) + LDR_OFFSET));
    }

    return lux;
}

/* ความถี่เสียงแปรผันเชิงเส้นตามรอบ: 800 rpm = 300 Hz, 4200 rpm = 2000 Hz */
float Sound_FrequencyFromRpm(float rpm)
{
    float hzPerRpm = (BUZZER_HZ_AT_REDLINE - BUZZER_HZ_AT_IDLE) / (REDLINE_RPM - IDLE_RPM);
    float frequency = BUZZER_HZ_AT_IDLE + ((rpm - IDLE_RPM) * hzPerRpm);

    frequency = Physics_MaxFloat(frequency, BUZZER_HZ_MIN);
    frequency = Physics_MinFloat(frequency, BUZZER_HZ_MAX);

    return frequency;
}

void Driver_SetOutput(GPIO_TypeDef *port, uint32_t pin, uint8_t state)
{
    if (state == 1u)
    {
        port->BSRR = (1u << pin);
    }
    else
    {
        port->BSRR = (1u << (pin + GPIO_BSRR_RESET_SHIFT));
    }
}

/* คืน 1u ถ้าบิตตาม mask ใน value เป็น 1 ไม่งั้นคืน 0u (เลี่ยงการแปลงผลเปรียบเทียบเป็นตัวเลขตรงๆ ตามกฎข้อ 13) */
uint8_t Driver_BitState(uint8_t value, uint8_t mask)
{
    uint8_t state = 0u;

    if ((value & mask) != 0u)
    {
        state = 1u;
    }
    else
    {
        /* No action */
    }

    return state;
}

void Driver_SegmentDisplay(uint8_t number)
{
    Driver_SetOutput(GPIOC, PIN_SEG_BIT0, Driver_BitState(number, SEG_BCD_BIT0));
    Driver_SetOutput(GPIOA, PIN_SEG_BIT1, Driver_BitState(number, SEG_BCD_BIT1));
    Driver_SetOutput(GPIOB, PIN_SEG_BIT2, Driver_BitState(number, SEG_BCD_BIT2));
    Driver_SetOutput(GPIOA, PIN_SEG_BIT3, Driver_BitState(number, SEG_BCD_BIT3));
}

/* ตั้งความถี่ TIM3 (toggle 2 ครั้งต่อคาบ) แล้วเปิดนับถ้ายังไม่ได้เปิด */
void Driver_BuzzerOn(float frequencyHz)
{
    uint32_t halfPeriodTicks = (uint32_t)(TIM3_CLOCK_HZ / (TOGGLES_PER_PERIOD * frequencyHz));

    TIM3->ARR = halfPeriodTicks - 1u;

    if (g_buzzerRunning == 0u)
    {
        TIM3->CNT = 0u;
        TIM3->EGR = TIM_EGR_UG;          /* โหลด ARR ใหม่ทันที */
        TIM3->CR1 |= TIM_CR1_CEN;
        g_buzzerRunning = 1u;
    }
    else
    {
        /* No action */
    }
}

/* หยุด TIM3 แล้วดึงขา buzzer ลง 0 (ไม่ค้างกระแสที่ลำโพง) */
void Driver_BuzzerOff(void)
{
    if (g_buzzerRunning == 1u)
    {
        TIM3->CR1 &= ~TIM_CR1_CEN;
        TIM3->SR = ~TIM_SR_UIF;
        NVIC_ClearPendingIRQ(TIM3_IRQn);
        g_buzzerRunning = 0u;
    }
    else
    {
        /* No action */
    }

    Driver_SetOutput(GPIOC, PIN_BUZZER, 0u);
}

/* รอแบบ blocking ใช้เฉพาะตอนเริ่มระบบ ก่อนเข้า main loop */
void System_DelayMs(uint32_t ms)
{
    uint32_t startTick = g_systemTickMs;

    while ((g_systemTickMs - startTick) < ms)
    {
        /* รอ TIM2 นับเวลา */
    }
}

void System_CalibrateJoystick(void)
{
    System_DelayMs(CALIBRATION_DELAY_MS);
    g_centerX = (int32_t)g_adcBuffer[ADC_IDX_JOY_X];
    g_centerY = (int32_t)g_adcBuffer[ADC_IDX_JOY_Y];
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
    else
    {
        /* No action */
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
    char digits[UART_NUMBER_DIGITS_MAX];
    uint32_t magnitude = 0u;
    uint8_t digitCount = 0u;

    if (value < 0)
    {
        Driver_UART_SendChar('-');
        magnitude = (uint32_t)(-value);
    }
    else
    {
        magnitude = (uint32_t)value;
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
            digits[digitCount] = (char)('0' + (magnitude % DECIMAL_BASE));
            magnitude /= DECIMAL_BASE;
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
    float magnitude = value;
    int32_t tenths = 0;
    int32_t wholePart = 0;

    if (value < 0.0f)
    {
        Driver_UART_SendChar('-');
        magnitude = -value;
    }
    else
    {
        /* No action */
    }

    tenths = (int32_t)((magnitude * ONE_DECIMAL_SCALE) + ROUND_HALF);
    wholePart = tenths / (int32_t)DECIMAL_BASE;
    tenths = tenths % (int32_t)DECIMAL_BASE;

    Driver_UART_SendNumber(wholePart);
    Driver_UART_SendChar('.');
    Driver_UART_SendNumber(tenths);
}
