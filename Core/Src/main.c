/* Smart Lock System
 * ----------------------------------------------------------------
 *  - 0.96" SSD1306 OLED  (I2C1, PB8 SCL / PB9 SDA)
 *  - SG90 servo          (TIM3_CH3 PWM on PB0)
 *  - HC-SR04 ultrasonic  (TRIG = PA9, ECHO = PA8, TIM1 1us-tick)
 *  - AS608 fingerprint   (USART1 PB6 TX / PB7 RX, 57600 8N1)
 *  - 4x4 matrix keypad   (rows PA4..PA7 out, cols PA0..PA3 in pull-down)
 *
 *  Behaviour:
 *      Idle: OLED is OFF.
 *      When ultrasonic measures <= 5 cm: show welcome screen, then menu.
 *      User can unlock/lock via fingerprint OR password (keypad).
 *      Fingerprint is read ONLY when the user selects option 1 from the menu.
 *      Enrollment auto-advances to the next free slot; previous prints
 *        are never overwritten.
 *      No detection for 10 s -> OLED is turned OFF until the next approach.
 * ---------------------------------------------------------------- */

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "fonts.h"
#include "ssd1306.h"
#include "fingerprint.h"
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* ----- Ultrasonic / proximity ----- */
#define TRIG_GPIO_PORT          GPIOA
#define TRIG_GPIO_PIN           GPIO_PIN_9
#define ECHO_GPIO_PORT          GPIOA
#define ECHO_GPIO_PIN           GPIO_PIN_8

#define PROXIMITY_THRESHOLD_CM  5       /* trigger welcome below this distance  */
#define IDLE_TIMEOUT_MS         10000U  /* turn screen OFF after 10 s no activity*/
#define WELCOME_DURATION_MS     2000U   /* how long to hold the welcome screen   */
#define ULTRASONIC_NO_ECHO      9999U   /* sentinel for "no measurement"         */

/* ----- Servo (TIM3_CH3 on PB0) ----- */
#define SERVO_LOCKED_PULSE      250     /* ~0.5 ms  -> 0 deg                     */
#define SERVO_UNLOCKED_PULSE    1250    /* ~2.5 ms  -> 180 deg                   */

/* ----- Fingerprint config --------- */
#define FINGER_MIN_SCORE        50      /* below this we treat as no match       */
#define AS608_MAX_FINGER_SLOTS  127U    /* maximum slots the AS608 supports      */

/* ----- Built-in LED (Blue Pill PC13, active LOW) ----- */
#define LED_GPIO_PORT           GPIOC
#define LED_GPIO_PIN            GPIO_PIN_13

/* ----- Buzzer (active-HIGH on PB1) ----- */
#define BUZZER_GPIO_PORT        GPIOB
#define BUZZER_GPIO_PIN         GPIO_PIN_1

/* ----- Feedback durations ----- */
#define FEEDBACK_HOLD_MS        2000U   /* LED solid + buzzer + message hold     */
#define BLINK_PERIOD_MS         200U    /* enrollment-success blink half-period  */
#define BLINK_TOTAL_MS          2000U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef  hi2c1;
TIM_HandleTypeDef  htim1;
TIM_HandleTypeDef  htim3;
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
static uint8_t  screen_is_on      = 0;  /* OLED state                            */
static uint32_t last_active_ms    = 0;  /* last time we saw the user             */
static uint8_t  fingerprint_ready = 0;  /* set if AS608 responded at boot        */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */
static char     getPressedKey(void);
static void     firstScreen(void);
static void     secondScreen(void);

static void     OLED_TurnOn(void);
static void     OLED_TurnOff(void);

static void     delay_us(uint16_t us);
static uint16_t HCSR04_ReadDistanceCM(void);

static void     Servo_Lock(void);
static void     Servo_Unlock(void);

static void     show_message(const char *line1, const char *line2,
                             uint32_t hold_ms);
static void     fingerprint_status_cb(const char *status);

static int      prompt_for_password(const char *header);
static void     handle_password_flow(int *lock);
static void     handle_enroll_flow(void);
static void     handle_fingerprint_flow(int *lock);

static void     LED_On(void);
static void     LED_Off(void);
static void     LED_Toggle(void);
static void     Buzzer_On(void);
static void     Buzzer_Off(void);
static void     toggle_lock_with_feedback(int *lock, const char *header);
static void     blink_led_2s(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ===================================================================== */
/*                          OLED display helpers                         */
/* ===================================================================== */

static void OLED_TurnOn(void)
{
    if (!screen_is_on) {
        ssd1306_I2C_Write(SSD1306_I2C_ADDR, 0x00, 0xAF); /* DISPLAY ON  */
        screen_is_on = 1;
    }
}

static void OLED_TurnOff(void)
{
    if (screen_is_on) {
        ssd1306_I2C_Write(SSD1306_I2C_ADDR, 0x00, 0xAE); /* DISPLAY OFF */
        screen_is_on = 0;
    }
}

/* Show two short lines on the OLED for `hold_ms` milliseconds.         */
static void show_message(const char *line1, const char *line2,
                         uint32_t hold_ms)
{
    OLED_TurnOn();
    SSD1306_Fill(0);
    SSD1306_GotoXY(0, 0);
    if (line1) SSD1306_Puts((char *)line1, &Font_7x10, 1);
    if (line2) {
        SSD1306_GotoXY(0, 16);
        SSD1306_Puts((char *)line2, &Font_7x10, 1);
    }
    SSD1306_UpdateScreen();
    if (hold_ms) HAL_Delay(hold_ms);
}

/* ===================================================================== */
/*                        Welcome / menu screens                         */
/* ===================================================================== */

static void firstScreen(void)
{
    OLED_TurnOn();
    SSD1306_Fill(0);
    SSD1306_GotoXY(0, 0);
    SSD1306_Puts("Welcome to",     &Font_11x18, 1);
    SSD1306_GotoXY(0, 20);
    SSD1306_Puts("The Smart",      &Font_11x18, 1);
    SSD1306_GotoXY(0, 40);
    SSD1306_Puts("Lock System",    &Font_11x18, 1);
    SSD1306_UpdateScreen();
}

/* Menu — fingerprint is now on-demand only (no passive polling hint).  */
static void secondScreen(void)
{
    OLED_TurnOn();
    SSD1306_Fill(0);
    SSD1306_GotoXY(0, 0);
    SSD1306_Puts("Select option:",    &Font_7x10, 1);
    SSD1306_GotoXY(0, 14);
    SSD1306_Puts("1.Use Fingerprint", &Font_7x10, 1);
    SSD1306_GotoXY(0, 26);
    SSD1306_Puts("2.Enroll Finger",   &Font_7x10, 1);
    SSD1306_GotoXY(0, 38);
    SSD1306_Puts("3.Type Password",   &Font_7x10, 1);
    SSD1306_GotoXY(0, 52);
    SSD1306_Puts("D.Turn off display",&Font_7x10, 1);
    SSD1306_UpdateScreen();
}

/* ===================================================================== */
/*                             Keypad scan                               */
/* ===================================================================== */

static char getPressedKey(void)
{
    char key = 0;

    GPIO_TypeDef *portRow = GPIOA;
    GPIO_TypeDef *portCol = GPIOA;

    uint16_t rowPins[4] = {GPIO_PIN_7, GPIO_PIN_6, GPIO_PIN_5, GPIO_PIN_4};
    uint16_t colPins[4] = {GPIO_PIN_3, GPIO_PIN_2, GPIO_PIN_1, GPIO_PIN_0};

    /* ROW 1 */
    HAL_GPIO_WritePin(portRow, rowPins[0], GPIO_PIN_SET);
    HAL_GPIO_WritePin(portRow, rowPins[1], GPIO_PIN_RESET);
    HAL_GPIO_WritePin(portRow, rowPins[2], GPIO_PIN_RESET);
    HAL_GPIO_WritePin(portRow, rowPins[3], GPIO_PIN_RESET);
    if      (HAL_GPIO_ReadPin(portCol, colPins[0]) == GPIO_PIN_SET) key = '1';
    else if (HAL_GPIO_ReadPin(portCol, colPins[1]) == GPIO_PIN_SET) key = '2';
    else if (HAL_GPIO_ReadPin(portCol, colPins[2]) == GPIO_PIN_SET) key = '3';
    else if (HAL_GPIO_ReadPin(portCol, colPins[3]) == GPIO_PIN_SET) key = 'A';

    /* ROW 2 */
    HAL_GPIO_WritePin(portRow, rowPins[0], GPIO_PIN_RESET);
    HAL_GPIO_WritePin(portRow, rowPins[1], GPIO_PIN_SET);
    if      (HAL_GPIO_ReadPin(portCol, colPins[0]) == GPIO_PIN_SET) key = '4';
    else if (HAL_GPIO_ReadPin(portCol, colPins[1]) == GPIO_PIN_SET) key = '5';
    else if (HAL_GPIO_ReadPin(portCol, colPins[2]) == GPIO_PIN_SET) key = '6';
    else if (HAL_GPIO_ReadPin(portCol, colPins[3]) == GPIO_PIN_SET) key = 'B';

    /* ROW 3 */
    HAL_GPIO_WritePin(portRow, rowPins[1], GPIO_PIN_RESET);
    HAL_GPIO_WritePin(portRow, rowPins[2], GPIO_PIN_SET);
    if      (HAL_GPIO_ReadPin(portCol, colPins[0]) == GPIO_PIN_SET) key = '7';
    else if (HAL_GPIO_ReadPin(portCol, colPins[1]) == GPIO_PIN_SET) key = '8';
    else if (HAL_GPIO_ReadPin(portCol, colPins[2]) == GPIO_PIN_SET) key = '9';
    else if (HAL_GPIO_ReadPin(portCol, colPins[3]) == GPIO_PIN_SET) key = 'C';

    /* ROW 4 */
    HAL_GPIO_WritePin(portRow, rowPins[2], GPIO_PIN_RESET);
    HAL_GPIO_WritePin(portRow, rowPins[3], GPIO_PIN_SET);
    if      (HAL_GPIO_ReadPin(portCol, colPins[0]) == GPIO_PIN_SET) key = '*';
    else if (HAL_GPIO_ReadPin(portCol, colPins[1]) == GPIO_PIN_SET) key = '0';
    else if (HAL_GPIO_ReadPin(portCol, colPins[2]) == GPIO_PIN_SET) key = '#';
    else if (HAL_GPIO_ReadPin(portCol, colPins[3]) == GPIO_PIN_SET) key = 'D';

    if (key != 0) {
        HAL_Delay(20); /* debounce */
        return key;
    }
    return 0;
}

/* ===================================================================== */
/*                   Microsecond delay + HC-SR04                         */
/* ===================================================================== */

static void delay_us(uint16_t us)
{
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    HAL_TIM_Base_Start(&htim1);
    while (__HAL_TIM_GET_COUNTER(&htim1) < us) { /* spin */ }
    HAL_TIM_Base_Stop(&htim1);
}

/* Returns distance in cm, or ULTRASONIC_NO_ECHO on timeout.            */
static uint16_t HCSR04_ReadDistanceCM(void)
{
    uint32_t timeout;

    /* 10 us trigger pulse */
    HAL_GPIO_WritePin(TRIG_GPIO_PORT, TRIG_GPIO_PIN, GPIO_PIN_RESET);
    delay_us(2);
    HAL_GPIO_WritePin(TRIG_GPIO_PORT, TRIG_GPIO_PIN, GPIO_PIN_SET);
    delay_us(10);
    HAL_GPIO_WritePin(TRIG_GPIO_PORT, TRIG_GPIO_PIN, GPIO_PIN_RESET);

    /* Wait for echo line to go HIGH (start of return pulse). */
    timeout = 30000;
    while (HAL_GPIO_ReadPin(ECHO_GPIO_PORT, ECHO_GPIO_PIN) == GPIO_PIN_RESET) {
        if (--timeout == 0) return ULTRASONIC_NO_ECHO;
    }

    /* Time the HIGH width with TIM1 (1 us per tick). */
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    HAL_TIM_Base_Start(&htim1);
    timeout = 30000;
    while (HAL_GPIO_ReadPin(ECHO_GPIO_PORT, ECHO_GPIO_PIN) == GPIO_PIN_SET) {
        if (--timeout == 0) {
            HAL_TIM_Base_Stop(&htim1);
            return ULTRASONIC_NO_ECHO;
        }
    }
    uint32_t pulse_us = __HAL_TIM_GET_COUNTER(&htim1);
    HAL_TIM_Base_Stop(&htim1);

    /* HC-SR04: distance(cm) = pulse_width(us) / 58 */
    return (uint16_t)(pulse_us / 58U);
}

/* ===================================================================== */
/*                               Servo                                   */
/* ===================================================================== */

static void Servo_Lock(void)
{
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, SERVO_LOCKED_PULSE);
}

static void Servo_Unlock(void)
{
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, SERVO_UNLOCKED_PULSE);
}

/* ===================================================================== */
/*                         LED + buzzer helpers                          */
/* ===================================================================== */

/* PC13 is active-LOW on the Blue Pill. */
static void LED_On(void)
{
    HAL_GPIO_WritePin(LED_GPIO_PORT, LED_GPIO_PIN, GPIO_PIN_RESET);
}
static void LED_Off(void)
{
    HAL_GPIO_WritePin(LED_GPIO_PORT, LED_GPIO_PIN, GPIO_PIN_SET);
}
static void LED_Toggle(void)
{
    HAL_GPIO_TogglePin(LED_GPIO_PORT, LED_GPIO_PIN);
}

/* Active buzzer on PB1: HIGH = beep, LOW = silent. */
static void Buzzer_On(void)
{
    HAL_GPIO_WritePin(BUZZER_GPIO_PORT, BUZZER_GPIO_PIN, GPIO_PIN_SET);
}
static void Buzzer_Off(void)
{
    HAL_GPIO_WritePin(BUZZER_GPIO_PORT, BUZZER_GPIO_PIN, GPIO_PIN_RESET);
}

/* Common lock-change feedback: move servo, hold LED + buzzer for 2 s. */
static void toggle_lock_with_feedback(int *lock, const char *header)
{
    LED_On();
    Buzzer_On();

    if (*lock == 1) {
        Servo_Unlock();
        show_message(header, "Unlocked", FEEDBACK_HOLD_MS);
        *lock = 0;
    } else {
        Servo_Lock();
        show_message(header, "Locked", FEEDBACK_HOLD_MS);
        *lock = 1;
    }

    LED_Off();
    Buzzer_Off();
    last_active_ms = HAL_GetTick();
}

/* Blink the built-in LED at ~5 Hz for 2 seconds after enrolment.     */
static void blink_led_2s(void)
{
    uint32_t toggles = BLINK_TOTAL_MS / BLINK_PERIOD_MS; /* 10 toggles */
    LED_Off();
    for (uint32_t i = 0; i < toggles; i++) {
        LED_Toggle();
        HAL_Delay(BLINK_PERIOD_MS);
    }
    LED_Off();
}

/* ===================================================================== */
/*                       Fingerprint UI helpers                          */
/* ===================================================================== */

/* Push a single status line to the OLED during enrollment.            */
static void fingerprint_status_cb(const char *status)
{
    OLED_TurnOn();
    SSD1306_Fill(0);
    SSD1306_GotoXY(0, 0);
    SSD1306_Puts("Fingerprint:", &Font_7x10, 1);
    SSD1306_GotoXY(0, 16);
    SSD1306_Puts((char *)status, &Font_7x10, 1);
    SSD1306_UpdateScreen();
    last_active_ms = HAL_GetTick();
}

/* ===================================================================== */
/*                        Authentication flows                           */
/* ===================================================================== */

/* Read 4 keystrokes and return 1 if they match the stored password.   */
static int prompt_for_password(const char *header)
{
    static const char password[4] = {'5', '8', '6', '9'};
    char getPassword[4];
    int  flag = 1;

    /* Wait for any in-flight key release before reading entry. */
    while (getPressedKey() != 0) { /* spin */ }

    OLED_TurnOn();
    SSD1306_Fill(0);
    SSD1306_GotoXY(0, 0);
    SSD1306_Puts((char *)header, &Font_7x10, 1);
    SSD1306_GotoXY(0, 20);
    SSD1306_UpdateScreen();

    int      i           = 0;
    uint32_t entry_start = HAL_GetTick();

    while (i < 4) {
        /* Abort if the user walks away mid-entry. */
        if ((HAL_GetTick() - entry_start) > 15000U) {
            show_message("Timeout", NULL, 1000);
            return 0;
        }

        char k = getPressedKey();
        if (k != 0) {
            /* Show '*' instead of the digit for privacy. */
            SSD1306_Putc('*', &Font_7x10, 1);
            SSD1306_UpdateScreen();
            getPassword[i] = k;
            if (getPassword[i] != password[i]) flag = 0;
            i++;
            while (getPressedKey() != 0) { /* wait release */ }
            last_active_ms = HAL_GetTick();
        }
    }
    return flag;
}

/* --- Password unlock/lock --- */
static void handle_password_flow(int *lock)
{
    if (!prompt_for_password("Enter Password:")) {
        show_message("Wrong password", "Access Denied", 1500);
        return;
    }
    toggle_lock_with_feedback(lock, "Correct password");
}

/* --- Enrolment (admin only, multi-slot) ---
 *
 *  `next_slot` starts at 1 and increments after every SUCCESSFUL enrolment.
 *  A failed attempt does NOT advance the counter, so the admin can retry.
 *  Previously enrolled fingerprints are never overwritten because we never
 *  reuse a slot that has already been committed.
 *
 *  Note: next_slot resets to 1 on every power-cycle.  The AS608 keeps its
 *  flash templates across resets, so existing IDs remain valid; only the
 *  slot counter is lost.  To survive resets, persist next_slot in the STM32
 *  backup registers or internal flash.
 */
static void handle_enroll_flow(void)
{
    static uint8_t next_slot = 1; /* advances only on success */

    if (!fingerprint_ready) {
        show_message("Sensor not", "available", 1500);
        return;
    }

    if (!prompt_for_password("Admin password:")) {
        show_message("Wrong password", "Access Denied", 1500);
        return;
    }

    /* Guard: cannot exceed the sensor's capacity. */
    if (next_slot > AS608_MAX_FINGER_SLOTS) {
        show_message("Memory full!", "Max slots used", 1500);
        return;
    }

    /* Show the admin which slot will be written. */
    char slot_line[20];
    snprintf(slot_line, sizeof(slot_line), "slot %u of %u",
             (unsigned)next_slot, (unsigned)AS608_MAX_FINGER_SLOTS);
    show_message("Enrolling at", slot_line, 800);

    uint8_t r = AS608_EnrollFinger(next_slot, fingerprint_status_cb);

    if (r == AS608_OK) {
        /* Build a confirmation string BEFORE incrementing so it shows the
         * ID that was just saved. */
        char ok_line[20];
        snprintf(ok_line, sizeof(ok_line), "Saved (ID %u)", (unsigned)next_slot);

        next_slot++; /* advance for the next enrolment call */

        OLED_TurnOn();
        SSD1306_Fill(0);
        SSD1306_GotoXY(0, 0);
        SSD1306_Puts("Enrollment OK", &Font_7x10, 1);
        SSD1306_GotoXY(0, 16);
        SSD1306_Puts(ok_line, &Font_7x10, 1);
        SSD1306_UpdateScreen();
        blink_led_2s(); /* 2-second visual confirmation (blocks here) */
    } else {
        char buf[16];
        snprintf(buf, sizeof(buf), "Err 0x%02X", r);
        show_message("Enroll failed", buf, 1500);
        /* next_slot NOT incremented — admin can retry the same slot */
    }

    last_active_ms = HAL_GetTick();
}

/* --- Fingerprint unlock/lock (explicit, triggered by key '1') ---
 *
 *  Announces "Place finger", waits up to 5 s, then runs the full
 *  identify pipeline.  Fingerprint is read ONLY from this function;
 *  there is no background / passive polling.
 */
static void handle_fingerprint_flow(int *lock)
{
    if (!fingerprint_ready) {
        show_message("Sensor not", "available", 1500);
        return;
    }

    show_message("Place finger", "on sensor...", 0);

    AS608_SearchResult_t hit;
    uint8_t r = AS608_IdentifyFinger(&hit, 5000);

    if (r == AS608_OK && hit.matchScore >= FINGER_MIN_SCORE) {
        char buf[20];
        snprintf(buf, sizeof(buf), "ID:%u  S:%u",
                 (unsigned)hit.pageID, (unsigned)hit.matchScore);
        toggle_lock_with_feedback(lock, "Match");
        show_message("Welcome", buf, 800);
    } else if (r == AS608_NO_FINGER) {
        show_message("Timed out", "No finger", 1500);
    } else if (r == AS608_NOT_FOUND) {
        show_message("Not recognized", "Access Denied", 1500);
    } else {
        char buf[16];
        snprintf(buf, sizeof(buf), "Err 0x%02X", r);
        show_message("Sensor error", buf, 1500);
    }

    last_active_ms = HAL_GetTick();
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  */
int main(void)
{
    /* USER CODE BEGIN 1 */
    /* USER CODE END 1 */

    HAL_Init();

    /* USER CODE BEGIN Init */
    /* USER CODE END Init */

    SystemClock_Config();

    /* USER CODE BEGIN SysInit */
    /* USER CODE END SysInit */

    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_TIM3_Init();
    MX_TIM1_Init();
    MX_USART1_UART_Init();

    /* USER CODE BEGIN 2 */
    SSD1306_Init();
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

    /* Start in the LOCKED state and show the welcome screen. */
    int lock = 1;
    Servo_Lock();
    screen_is_on = 1;
    firstScreen();
    HAL_Delay(2500);

    /* Try to talk to the AS608 fingerprint sensor. */
    fingerprint_ready = (AS608_Init(&huart1) == AS608_OK) ? 1 : 0;

    /* Show the menu and enter the main loop. */
    secondScreen();
    last_active_ms = HAL_GetTick();
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        /* 1) Sample the ultrasonic sensor. */
        uint16_t dist = HCSR04_ReadDistanceCM();
        uint8_t  near = (dist != ULTRASONIC_NO_ECHO &&
                         dist <= PROXIMITY_THRESHOLD_CM);

        /* 2) Wake-up: object appeared while screen was off. */
        if (near && !screen_is_on) {
            firstScreen();
            HAL_Delay(WELCOME_DURATION_MS);
            secondScreen();
            last_active_ms = HAL_GetTick();
        }
        /* 3) Object still close — keep the wake-up timer fresh. */
        else if (near) {
            last_active_ms = HAL_GetTick();
        }

        /* 4) Accept user input only while the screen is on. */
        if (screen_is_on) {

            char k = getPressedKey();
            if (k != 0) {
                while (getPressedKey() != 0) { /* wait for key release */ }
                last_active_ms = HAL_GetTick();

                switch (k) {
                    case '1':   /* Fingerprint unlock/lock (explicit, on-demand) */
                        handle_fingerprint_flow(&lock);
                        secondScreen();
                        break;

                    case '2':   /* Enrol a new fingerprint (password-protected)  */
                        handle_enroll_flow();
                        secondScreen();
                        break;

                    case '3':   /* Password unlock/lock                           */
                        handle_password_flow(&lock);
                        secondScreen();
                        break;

                    case 'D':   /* Manual display off                             */
                        OLED_TurnOff();
                        break;

                    default:
                        /* Ignore other keys; keep the menu visible. */
                        break;
                }
            }

            /* Auto-sleep after IDLE_TIMEOUT_MS of no activity. */
            if ((HAL_GetTick() - last_active_ms) >= IDLE_TIMEOUT_MS) {
                OLED_TurnOff();
            }
        }

        /* 5) Light pacing — no need to ping the sonar at full speed. */
        HAL_Delay(60);
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  *
  *  NOTE: this build runs on the HSI (8 MHz) without PLL.
  *        TIM1 prescaler is set accordingly to give a 1 us tick.
  *        If you switch to HSE 8 MHz x PLL9 -> 72 MHz, also update:
  *           - TIM1 prescaler  (71 instead of  7)
  *           - TIM3 prescaler  (143 instead of 15)
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK  | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
        Error_Handler();
    }
}

/**
  * @brief I2C1 Initialization (SSD1306 OLED, 400 kHz fast-mode)
  */
static void MX_I2C1_Init(void)
{
    hi2c1.Instance             = I2C1;
    hi2c1.Init.ClockSpeed      = 400000;
    hi2c1.Init.DutyCycle       = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1     = 0;
    hi2c1.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2     = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
        Error_Handler();
    }
}

/**
  * @brief TIM1 Initialization (microsecond timebase for HC-SR04)
  *
  *  HSI 8 MHz / (prescaler 7 + 1) = 1 MHz  ->  1 tick = 1 us.
  *  ARR = 0xFFFF gives ~65 ms range, enough for any echo pulse.
  */
static void MX_TIM1_Init(void)
{
    TIM_ClockConfigTypeDef  sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig      = {0};

    htim1.Instance               = TIM1;
    htim1.Init.Prescaler         = 7;
    htim1.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim1.Init.Period            = 0xFFFF;
    htim1.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
}

/**
  * @brief TIM3 Initialization (50 Hz PWM for SG90 servo on TIM3_CH3 / PB0)
  *
  *  HSI 8 MHz / (prescaler 15 + 1) = 500 kHz  ->  1 tick = 2 us.
  *  Period 9999 -> 10 000 ticks = 20 ms = 50 Hz.
  *  Pulse 250  ->  500 us (~0.5 ms, locked  /  0 deg).
  *  Pulse 1250 -> 2500 us (~2.5 ms, unlocked/ 180 deg).
  */
static void MX_TIM3_Init(void)
{
    TIM_ClockConfigTypeDef  sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig      = {0};
    TIM_OC_InitTypeDef      sConfigOC          = {0};

    htim3.Instance               = TIM3;
    htim3.Init.Prescaler         = 15;
    htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim3.Init.Period            = 9999;
    htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
    sConfigOC.OCMode       = TIM_OCMODE_PWM1;
    sConfigOC.Pulse        = 0;
    sConfigOC.OCPolarity   = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode   = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) {
        Error_Handler();
    }
    HAL_TIM_MspPostInit(&htim3);
}

/**
  * @brief USART1 Initialization (AS608 fingerprint sensor, 57600 8N1)
  */
static void MX_USART1_UART_Init(void)
{
    huart1.Instance          = USART1;
    huart1.Init.BaudRate     = 57600;
    huart1.Init.WordLength   = UART_WORDLENGTH_8B;
    huart1.Init.StopBits     = UART_STOPBITS_1;
    huart1.Init.Parity       = UART_PARITY_NONE;
    huart1.Init.Mode         = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK) {
        Error_Handler();
    }
}

/**
  * @brief GPIO Initialization
  *
  *  Pin map:
  *      PA0..PA3  Input pull-down   (keypad columns)
  *      PA4..PA7  Output PP         (keypad rows)
  *      PA8       Input             (HC-SR04 ECHO, 5V-tolerant on F1)
  *      PA9       Output PP         (HC-SR04 TRIG)
  *      PB1       Output PP         (Buzzer, active HIGH)
  *      PC13      Output PP         (Built-in LED, active LOW)
  */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* Initial output levels */
    HAL_GPIO_WritePin(GPIOA,
                      GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7,
                      GPIO_PIN_RESET);
    HAL_GPIO_WritePin(TRIG_GPIO_PORT,   TRIG_GPIO_PIN,   GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BUZZER_GPIO_PORT, BUZZER_GPIO_PIN, GPIO_PIN_RESET); /* off */
    HAL_GPIO_WritePin(LED_GPIO_PORT,    LED_GPIO_PIN,    GPIO_PIN_SET);   /* off */

    /* PA0..PA3 — keypad columns (input, pull-down) */
    GPIO_InitStruct.Pin  = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* PA4..PA7 — keypad rows (output PP) */
    GPIO_InitStruct.Pin   = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* PA9 — ultrasonic TRIG (output PP) */
    GPIO_InitStruct.Pin   = TRIG_GPIO_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(TRIG_GPIO_PORT, &GPIO_InitStruct);

    /* PA8 — ultrasonic ECHO (input, no pull) */
    GPIO_InitStruct.Pin  = ECHO_GPIO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(ECHO_GPIO_PORT, &GPIO_InitStruct);

    /* PB1 — buzzer (output PP, active HIGH) */
    GPIO_InitStruct.Pin   = BUZZER_GPIO_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(BUZZER_GPIO_PORT, &GPIO_InitStruct);

    /* PC13 — built-in LED (output PP, active LOW) */
    GPIO_InitStruct.Pin   = LED_GPIO_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_GPIO_PORT, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  */
void Error_Handler(void)
{
    __disable_irq();
    while (1) { /* hang */ }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    /* User can add reporting here. */
}
#endif /* USE_FULL_ASSERT */
