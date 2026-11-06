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
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "ssd1306.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
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
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */
volatile int32_t weight_raw = 0;
uint32_t last_oled_update = 0;
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

    // 5. CTRL1: VLDO [5:3] = 100 (3.3 В), GAIN [2:0] = 110 (x64) -> 0x26
    NAU7802_WriteReg(hi2c, NAU_REG_CTRL1, 0x2E); // was 0x26 stands for 3V3
    HAL_Delay(10);

    // 6. CTRL2: частота выборок CRS [6:4] = 011 (80 SPS)
    uint8_t ctrl2 = NAU7802_ReadReg(hi2c, NAU_REG_CTRL2);
    ctrl2 &= ~0x70;
    ctrl2 |= (0x03 << 4);
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

int32_t NAU7802_ReadADC(I2C_HandleTypeDef *hi2c) {
    static int32_t last = 0;

    // Ожидание готовности данных: бит CR (5) в PU_CTRL (0x00)
    uint32_t timeout = 300;
    while (!(NAU7802_ReadReg(hi2c, NAU_REG_PU_CTRL) & NAU_PU_CR)) {
        HAL_Delay(1);
        if (--timeout == 0) return last;   // данных нет — отдаем прошлое значение
    }

    // Читаем 3 байта результата одним burst-чтением с адреса 0x12
    uint8_t rx_data[3];
    if (HAL_I2C_Mem_Read(hi2c, NAU7802_ADDR, NAU_REG_ADCO_B2, I2C_MEMADD_SIZE_8BIT,
                         rx_data, 3, 100) != HAL_OK) {
        return last;
    }

    int32_t raw = ((int32_t)rx_data[0] << 16) | ((int32_t)rx_data[1] << 8) | rx_data[2];

    // Знаковое расширение 24-бит до 32-бит
    if (raw & 0x800000) {
        raw |= 0xFF000000;
    }

    last = raw;
    return raw;
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
  /* USER CODE BEGIN 2 */

  // Проверка связи с датчиком по I2C
  if (HAL_I2C_IsDeviceReady(&hi2c1, NAU7802_ADDR, 3, 100) != HAL_OK) {
      Error_Handler();
  }

  // Инициализация датчика
  if (NAU7802_Init(&hi2c1) == 0) {
      Error_Handler();
  }

  // Инициализация экрана SSD1306 (тот же I2C1, адрес 0x3C)
  if (!SSD1306_Init(&hi2c1)) {
      Error_Handler();
  }
  SSD1306_Fill(SSD1306_BLACK);
  SSD1306_WriteString(0, 0, "WEIGHT RAW", 1);
  SSD1306_UpdateScreen();

  // Калибровка нуля при старте (усреднение 30 точек)
//  int32_t init_sum = 0;
//  for (int i = 0; i < 20; i++) {
//      init_sum += NAU7802_ReadADC(&hi2c1);
//      HAL_Delay(15);
//  }
//  int32_t zero_offset = init_sum / 20;

  // Инициализация фильтра первым отсчетом (избегаем скачка к 0)
//  float filtered_weight = 0.0f;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
        // ReadADC сам ждет готовности данных (80 SPS ~ 12.5 мс), лишний HAL_Delay не нужен
        int32_t raw = NAU7802_ReadADC(&hi2c1);

//        // Убираем шумы и отрицательные значения около нуля
//        if (raw > -1200 && raw < 1200) {
//            raw = 0;
//        } else if (raw < 0) {
//            raw = 0;
//        }

        weight_raw = raw;

        // Обновляем экран 5 раз в секунду, чтобы не тормозить опрос АЦП
        if (HAL_GetTick() - last_oled_update >= 200) {
            last_oled_update = HAL_GetTick();

            char buf[16];
            snprintf(buf, sizeof(buf), "%8ld", (long)weight_raw);

            SSD1306_Fill(SSD1306_BLACK);
            SSD1306_WriteString(0, 0, "WEIGHT RAW", 1);
            SSD1306_WriteString(0, 24, buf, 2);
            SSD1306_UpdateScreen();
        }

        // Медианный + экспоненциальный фильтры (пока отключены)
//        int32_t samples[5];
//        for (int i = 0; i < 5; i++) {
//            samples[i] = NAU7802_ReadADC(&hi2c1) - zero_offset;
//            HAL_Delay(2);
//        }
//        for (int i = 0; i < 4; i++) {
//            for (int j = i + 1; j < 5; j++) {
//                if (samples[i] > samples[j]) {
//                    int32_t temp = samples[i];
//                    samples[i] = samples[j];
//                    samples[j] = temp;
//                }
//            }
//        }
//        int32_t raw_median = samples[2];
//        filtered_weight = (filtered_weight * 0.98f) + ((float)raw_median * 0.02f);
//        weight_raw = (int32_t)filtered_weight;

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

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;   // было 100000; в CubeMX: I2C Speed Mode = Fast Mode
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
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif
