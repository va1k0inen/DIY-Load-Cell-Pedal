/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Педаль на тензодатчике: NAU7802 -> USB HID (одна 16-бит ось X)
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
//#include "ssd1306.h"
#include "usbd_customhid.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* ======================= НАСТРОЙКИ ПЕДАЛИ ================================= */
#define ADC_SPAN_COUNTS   150000.0f  // Изменение raw от нуля до полного хода/усилия.
                                     // Подберите: нажмите педаль до упора и
                                     // посмотрите число на экране (строка AXIS RAW).
#define INVERT_AXIS       0          // 1, если при нажатии raw УМЕНЬШАЕТСЯ
#define FILTER_ALPHA      0.25f      // EMA 0..1: меньше = плавнее, но медленнее
#define DEADBAND          2200.0f     // Мёртвая зона у нуля (в отсчётах АЦП)
#define ZERO_TRACK        0.0005f    // Скорость подтягивания нуля (на один отсчёт)
#define TARE_SAMPLES      32         // Сколько отсчётов усреднять при старте
#define DRDY_TIMEOUT_MS   100        // Если фронт DRDY пропущен, читаем принудительно

#define OLED_ENABLED      1          // 0 = экран полностью отключен
#define OLED_UPDATE_MS    500        // Период обновления экрана (он блокирует I2C ~25 мс)

#define SIGN              (INVERT_AXIS ? -1.0f : 1.0f)

#define ZERO_TRACK        0.0005f    // Скорость подтягивания нуля (на один отсчёт)
#define WARMUP_MS         60000      // «прогрев»: первая минута после включения
#define WARMUP_WINDOW     5000.0f    // окно подстройки нуля в прогреве (отсчёты)
#define WARMUP_TRACK      0.003f     // скорость подстройки в прогреве
#define TARE_SAMPLES      32

#define OUT_SMOOTH   0.35f   // 0..1: меньше = плавнее, но больше задержка (~3 мс при 0.35)
/* ======================= NAU7802 ========================================== */
#define NAU7802_ADDR (0x2A << 1)

// Карта регистров NAU7802
#define NAU_REG_PU_CTRL  0x00
#define NAU_REG_CTRL1    0x01
#define NAU_REG_CTRL2    0x02
#define NAU_REG_ADCO_B2  0x12
#define NAU_REG_ADCO_B1  0x13
#define NAU_REG_ADCO_B0  0x14

// Биты регистра PU_CTRL (0x00)
#define NAU_PU_RR     (1 << 0)  // Register Reset
#define NAU_PU_PUD    (1 << 1)  // Power Up Digital
#define NAU_PU_PUA    (1 << 2)  // Power Up Analog
#define NAU_PU_PUR    (1 << 3)  // Power Up Ready (только чтение)
#define NAU_PU_CS     (1 << 4)  // Cycle Start (запуск конвертаций)
#define NAU_PU_CR     (1 << 5)  // Cycle Ready (данные готовы, только чтение)
#define NAU_PU_OSCS   (1 << 6)  // System clock source
#define NAU_PU_AVDDS  (1 << 7)  // AVDD source: 1 = внутренний LDO

// Биты регистра CTRL2 (0x02)
#define NAU_CTRL2_CALS     (1 << 2)  // Старт калибровки (сбрасывается сам по окончании)
#define NAU_CTRL2_CAL_ERR  (1 << 3)  // Ошибка калибровки

#define NAU_REG_ADC      0x15
#define NAU_REG_PGA_PWR  0x1C

// CTRL1: VLDO [5:3], GAIN [2:0]
//   0x2E = VLDO 3.0 В (101), GAIN x64 (110)
//   0x26 = VLDO 3.3 В (100), GAIN x64 (110)
#define NAU_CTRL1_VALUE  0x2E

// CTRL2: CRS [6:4] - частота выборок
//   0 = 10 SPS, 1 = 20, 2 = 40, 3 = 80, 7 = 320 SPS
#define NAU_CRS_VALUE    0x07
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */
extern USBD_HandleTypeDef hUsbDeviceFS;

volatile int32_t weight_raw = 0;       // последний сырой отсчёт (для отладчика)
volatile uint16_t axis_value = 0;      // то, что уходит на ПК
volatile uint8_t  drdy = 0;            // флаг от прерывания DRDY (PA0)

//static uint32_t last_oled_update = 0;
static uint32_t last_data_tick   = 0;
//static uint8_t  oled_ok    = 0;
static uint8_t  sensor_ok  = 0;

static float    filt   = 0.0f;         // отфильтрованное значение raw
static float    tare_f = 0.0f;         // текущий «ноль» (подстраивается)
static int32_t  last_hid = -1;         // последнее успешно отправленное значение

static float    out_f = 0.0f;
static uint32_t last_out_tick = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
void NAU7802_WriteReg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t value);
uint8_t NAU7802_ReadReg(I2C_HandleTypeDef *hi2c, uint8_t reg);
uint8_t NAU7802_Init(I2C_HandleTypeDef *hi2c);
int32_t NAU7802_ReadADC(I2C_HandleTypeDef *hi2c);
static int32_t NAU7802_ReadRaw(I2C_HandleTypeDef *hi2c, int32_t last);
static int32_t Tare_Measure(void);
static uint16_t Raw_To_U16(float d);
static void HID_SendAxis(uint16_t v);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void NAU7802_WriteReg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t value) {
    HAL_I2C_Mem_Write(hi2c, NAU7802_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, 100);
}

uint8_t NAU7802_ReadReg(I2C_HandleTypeDef *hi2c, uint8_t reg) {
    uint8_t value = 0;
    HAL_I2C_Mem_Read(hi2c, NAU7802_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, 100);
    return value;
}

uint8_t NAU7802_Init(I2C_HandleTypeDef *hi2c) {
    // 1. Программный сброс (RR = 1, затем RR = 0)
    NAU7802_WriteReg(hi2c, NAU_REG_PU_CTRL, NAU_PU_RR);
    HAL_Delay(10);
    NAU7802_WriteReg(hi2c, NAU_REG_PU_CTRL, 0x00);
    HAL_Delay(10);

    // 2. Включаем цифровую (PUD) и аналоговую (PUA) части
    NAU7802_WriteReg(hi2c, NAU_REG_PU_CTRL, NAU_PU_PUD | NAU_PU_PUA);   // = 0x06
    HAL_Delay(200);

    // 3. Ждем готовности питания (PUR = 1)
    uint32_t timeout = 1000;
    while (!(NAU7802_ReadReg(hi2c, NAU_REG_PU_CTRL) & NAU_PU_PUR)) {
        HAL_Delay(1);
        if (--timeout == 0) return 0;
    }

    // 4. Внутренний LDO (AVDDS = 1) и запуск циклов конвертации (CS = 1)
    uint8_t pu_ctrl = NAU7802_ReadReg(hi2c, NAU_REG_PU_CTRL);
    pu_ctrl |= NAU_PU_AVDDS | NAU_PU_CS;
    NAU7802_WriteReg(hi2c, NAU_REG_PU_CTRL, pu_ctrl);
    HAL_Delay(50);

    // 5. CTRL1: напряжение LDO и усиление PGA
    NAU7802_WriteReg(hi2c, NAU_REG_CTRL1, NAU_CTRL1_VALUE);
    HAL_Delay(10);

    // 6. CTRL2: частота выборок CRS [6:4]
    uint8_t ctrl2 = NAU7802_ReadReg(hi2c, NAU_REG_CTRL2);
    ctrl2 &= ~0x70;
    ctrl2 |= (NAU_CRS_VALUE << 4);
    NAU7802_WriteReg(hi2c, NAU_REG_CTRL2, ctrl2);
    HAL_Delay(10);

    // 6a. Отключаем CLK_CHP (рекомендация даташита, раздел 9.1)
    NAU7802_WriteReg(hi2c, NAU_REG_ADC, 0x30);

    // 6b. Включаем встроенный конденсатор 330 пФ на выходе PGA (бит 7)
    uint8_t pga_pwr = NAU7802_ReadReg(hi2c, NAU_REG_PGA_PWR);
    pga_pwr |= (1 << 7);
    NAU7802_WriteReg(hi2c, NAU_REG_PGA_PWR, pga_pwr);

    // 7. Внутренняя калибровка (CALS = 1), ждем сброса бита
    ctrl2 = NAU7802_ReadReg(hi2c, NAU_REG_CTRL2);
    ctrl2 |= NAU_CTRL2_CALS;
    NAU7802_WriteReg(hi2c, NAU_REG_CTRL2, ctrl2);

    timeout = 1000;
    while (NAU7802_ReadReg(hi2c, NAU_REG_CTRL2) & NAU_CTRL2_CALS) {
        HAL_Delay(10);
        if (--timeout == 0) return 0;
    }

    // Проверка ошибки калибровки
    if (NAU7802_ReadReg(hi2c, NAU_REG_CTRL2) & NAU_CTRL2_CAL_ERR) {
        return 0;
    }

    return 1;
}

/* Чтение с ожиданием готовности по биту CR (опрос по I2C).
 * Используется только при старте (тарирование). В основном цикле
 * данные читаются по прерыванию DRDY через NAU7802_ReadRaw(). */
int32_t NAU7802_ReadADC(I2C_HandleTypeDef *hi2c) {
    static int32_t last = 0;

    uint32_t timeout = 300;
    while (!(NAU7802_ReadReg(hi2c, NAU_REG_PU_CTRL) & NAU_PU_CR)) {
        HAL_Delay(1);
        if (--timeout == 0) return last;   // данных нет - отдаем прошлое значение
    }

    last = NAU7802_ReadRaw(hi2c, last);
    return last;
}

/* Чтение 3 байт результата без ожидания (burst с адреса 0x12). */
static int32_t NAU7802_ReadRaw(I2C_HandleTypeDef *hi2c, int32_t last)
{
    uint8_t rx[3];
    if (HAL_I2C_Mem_Read(hi2c, NAU7802_ADDR, NAU_REG_ADCO_B2, I2C_MEMADD_SIZE_8BIT,
                         rx, 3, 100) != HAL_OK) {
        return last;
    }

    int32_t raw = ((int32_t)rx[0] << 16) | ((int32_t)rx[1] << 8) | rx[2];

    // Знаковое расширение 24-бит до 32-бит
    if (raw & 0x800000) {
        raw |= 0xFF000000;
    }
    return raw;
}

/* Усреднённое значение для начального «нуля». */
static int32_t Tare_Measure(void)
{
    int64_t sum = 0;
    for (int i = 0; i < TARE_SAMPLES; i++) {
        sum += NAU7802_ReadADC(&hi2c1);
    }
    return (int32_t)(sum / TARE_SAMPLES);
}

/* Смещение от нуля (в отсчётах, уже со знаком оси) -> 0..65535. */
static uint16_t Raw_To_U16(float d)
{
    d -= DEADBAND;
    if (d < 0.0f) d = 0.0f;
    if (d > ADC_SPAN_COUNTS) d = ADC_SPAN_COUNTS;
    return (uint16_t)(d * 65535.0f / ADC_SPAN_COUNTS);
}

/* Отправка отчёта HID (2 байта, little-endian, без Report ID).
 * Если эндпоинт занят - last_hid не обновляется, и отправка повторится
 * в следующей итерации цикла. */
static void HID_SendAxis(uint16_t v)
{
    uint8_t rep[2] = { (uint8_t)(v & 0xFF), (uint8_t)(v >> 8) };

    if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED) {
        if (USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, rep, 2) == USBD_OK) {
            last_hid = v;
        }
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  // USB уже поднят: даже если датчик или экран не ответят, устройство
  // будет видно на ПК (ось просто останется в нуле).

//#if OLED_ENABLED
//  // Экран необязателен: при ошибке просто работаем без него
//  oled_ok = SSD1306_Init(&hi2c1);
//  if (oled_ok) {
//      SSD1306_Fill(SSD1306_BLACK);
//      SSD1306_WriteString(0, 0, "AXIS RAW", 1);
//      SSD1306_UpdateScreen();
//  }
//#endif

  // Датчик: проверка связи + инициализация
  if (HAL_I2C_IsDeviceReady(&hi2c1, NAU7802_ADDR, 3, 100) == HAL_OK) {
      sensor_ok = NAU7802_Init(&hi2c1);
  }

  if (sensor_ok) {
      // Начальный ноль (педаль должна быть свободна; если нет - ноль сам
      // сдвинется вниз, как только педаль отпустят)
      tare_f = (float)Tare_Measure();
      filt   = tare_f;
  }
//  else if (oled_ok) {
//      SSD1306_Fill(SSD1306_BLACK);
//      SSD1306_WriteString(0, 0, "NAU ERROR", 2);
//      SSD1306_UpdateScreen();
//  }

  drdy = 0;
  last_data_tick = HAL_GetTick();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      if (sensor_ok &&
          (drdy || (HAL_GetTick() - last_data_tick > DRDY_TIMEOUT_MS)))
      {
          drdy = 0;
          last_data_tick = HAL_GetTick();

          int32_t raw = NAU7802_ReadRaw(&hi2c1, weight_raw);
          weight_raw = raw;

          // Экспоненциальный фильтр
          filt += FILTER_ALPHA * ((float)raw - filt);

          // Смещение от нуля с учётом направления оси
          // Смещение от нуля с учётом направления оси
          float d = (filt - tare_f) * SIGN;

          // Первую минуту ноль подстраивается быстрее и в более широком окне
          uint32_t now = HAL_GetTick();
          float window = (now < WARMUP_MS) ? WARMUP_WINDOW : DEADBAND;
          float k      = (now < WARMUP_MS) ? WARMUP_TRACK  : ZERO_TRACK;

          if (d < 0.0f) {
              // Ушли ниже нуля: сразу переносим ноль
              tare_f = filt;
              d = 0.0f;
          } else if (d < window) {
              // В зоне покоя: ноль медленно следует за дрейфом
              tare_f += (filt - tare_f) * k;
          }

          axis_value = Raw_To_U16(d);
      }

      // Каждую миллисекунду плавно подтягиваем выход к целевому значению
      uint32_t t = HAL_GetTick();
      if (t != last_out_tick) {
          last_out_tick = t;
          out_f += OUT_SMOOTH * ((float)axis_value - out_f);
          uint16_t v = (uint16_t)(out_f + 0.5f);
          if ((int32_t)v != last_hid) {
              HID_SendAxis(v);
          }
      }

//#if OLED_ENABLED
//      if (oled_ok && sensor_ok &&
//          (HAL_GetTick() - last_oled_update >= OLED_UPDATE_MS)) {
//          last_oled_update = HAL_GetTick();
//
//          char buf[16];
//          snprintf(buf, sizeof(buf), "%8ld", (long)((filt - tare_f) * SIGN));
//
//          char hbuf[16];
//          snprintf(hbuf, sizeof(hbuf), "HID: %5u", (unsigned)axis_value);
//
//          SSD1306_Fill(SSD1306_BLACK);
//          SSD1306_WriteString(0, 0, "AXIS RAW", 1);
//          SSD1306_WriteString(0, 24, buf, 2);
//          SSD1306_WriteString(0, 52, hbuf, 1);
//          SSD1306_UpdateScreen();
//      }
//#endif

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* Прерывание DRDY (PA0): только ставим флаг, чтение - в основном цикле */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_0) {
        drdy = 1;
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
