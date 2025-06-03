/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
#define SECTOR0_RAW      0
#define SECTOR1_RAW      40
#define SECTOR2_RAW      32
#define SECTOR3_RAW      24
#define SECTOR4_RAW      16
#define SECTOR5_RAW      8
#define RAW_TOLERANCE    1
#define MAX_RAW          47
#define MAX_VECTOR_LENGTH 9
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
volatile uint8_t received_char;
uint8_t received_byte;
uint8_t g_RxBuff[1];

uint32_t rawCounter = 0, rawCounter2 = 0, rawCounter3 = 0;
uint32_t rawCounter4 = 0, rawCounter5 = 0;

int currentSector = 0, newSector = 0;
uint8_t currentVectorLength = 0, sectorUpdated = 0;
uint8_t ispaused = 0;
uint8_t twoMotorActive[5] = {0};
volatile uint8_t directionDone = 0;

uint32_t speed[] = {3150, 3300, 3500, 3700, 3900};
uint8_t modeFlag = 0; // 0: 정지, 1: 메인, 2: 서브1, 3: 서브2

const uint32_t sub2_sequence[6] = {0, 8, 16, 24, 32, 40};
uint8_t initialSectorSet = 0;
uint8_t currentTargetSector = 0;
uint8_t zeroAlignDone = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
void stopMotor(int motorCh);
void stopAllMotors(void);
void changeSector(int newSector);
void changeSpeed(void);
void runSubMode1(void);
void runSubMode2(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USB_DEVICE_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_TIM9_Init();
  MX_TIM12_Init();
  MX_NVIC_Init();

  HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim5,  TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim5,  TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim5,  TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim9,  TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim9,  TIM_CHANNEL_2);

  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);

  __HAL_TIM_SET_COUNTER(&htim1, 0);
  __HAL_TIM_SET_COUNTER(&htim2, 0);
  __HAL_TIM_SET_COUNTER(&htim3, 0);
  __HAL_TIM_SET_COUNTER(&htim4, 0);
  __HAL_TIM_SET_COUNTER(&htim8, 0);

  while (1)
  {
    rawCounter  = __HAL_TIM_GET_COUNTER(&htim1);
    rawCounter2 = __HAL_TIM_GET_COUNTER(&htim2);
    rawCounter3 = __HAL_TIM_GET_COUNTER(&htim3);
    rawCounter4 = __HAL_TIM_GET_COUNTER(&htim4);
    rawCounter5 = __HAL_TIM_GET_COUNTER(&htim8);

      if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2) == GPIO_PIN_RESET) {
      ispaused = 0;
      __HAL_TIM_SET_COUNTER(&htim1, 0);
      __HAL_TIM_SET_COUNTER(&htim2, 0);
      __HAL_TIM_SET_COUNTER(&htim3, 0);
      __HAL_TIM_SET_COUNTER(&htim4, 0);
      __HAL_TIM_SET_COUNTER(&htim8, 0);
    }
    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1) == GPIO_PIN_RESET) {
      ispaused = 1;
    }
    if (ispaused) {
      stopAllMotors();
      // 1열~5열 버튼(PE13,14,15, PB10, PB11)
      __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2,
      HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_13)==GPIO_PIN_RESET ? 3700 : 0);
      __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3,
      HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_14)==GPIO_PIN_RESET ? 3700 : 0);
      __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4,
      HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_15)==GPIO_PIN_RESET ? 3700 : 0);
      __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1,
      HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10)==GPIO_PIN_RESET ? 3700 : 0);
      __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2,
      HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11)==GPIO_PIN_RESET ? 3700 : 0);
      continue;
    }

    switch (modeFlag) {
    case 0:
      stopAllMotors();

      if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2) == GPIO_PIN_RESET) {
        __HAL_TIM_SET_COUNTER(&htim1, 0);
        __HAL_TIM_SET_COUNTER(&htim2, 0);
        __HAL_TIM_SET_COUNTER(&htim3, 0);
        __HAL_TIM_SET_COUNTER(&htim4, 0);
        __HAL_TIM_SET_COUNTER(&htim8, 0);
        zeroAlignDone = 1;
      }
      else if (!zeroAlignDone) {
        TIM_HandleTypeDef* htimList[5] = {&htim5, &htim5, &htim5, &htim9, &htim9};
        uint32_t chList[5] = {TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4, TIM_CHANNEL_1, TIM_CHANNEL_2};
        uint32_t counters[5] = {rawCounter, rawCounter2, rawCounter3, rawCounter4, rawCounter5};

        for (int i = 0; i < 5; i++) {
          if (counters[i] <= RAW_TOLERANCE || counters[i] >= (MAX_RAW - RAW_TOLERANCE)) {
            __HAL_TIM_SET_COMPARE(htimList[i], chList[i], 0);
          } else {
            __HAL_TIM_SET_COMPARE(htimList[i], chList[i], 2650);
          }
        }
      }
      break;
      case 1:
    	zeroAlignDone = 0;
        if (sectorUpdated) {
          stopMotor(1);
          changeSector(newSector);
        } else if (directionDone) {
          changeSpeed();
        }
        break;
      case 2:
    	zeroAlignDone = 0;
    	runSubMode1();
        break;
      case 3:
    	zeroAlignDone = 0;
        runSubMode2();
        break;
    }
  }
}

void stopMotor(int motorCh) {
  if (motorCh == 1) {
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 0);
  } else {
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, 0);
    __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, 0);
  }
}

void stopAllMotors(void) {
  stopMotor(1);
  stopMotor(2);
}

void changeSector(int newSector) {
  static const uint32_t sectorRaw[6] = {SECTOR0_RAW, SECTOR1_RAW, SECTOR2_RAW, SECTOR3_RAW, SECTOR4_RAW, SECTOR5_RAW};
  int32_t forwardDiff = (newSector - currentSector + 6) % 6;
  int32_t reverseDiff = (currentSector - newSector + 6) % 6;
  bool useReverse = (forwardDiff >= reverseDiff);
  TIM_HandleTypeDef* htimList[5] = {&htim5, &htim5, &htim5, &htim9, &htim9};
  uint32_t chList[5] = {TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4, TIM_CHANNEL_1, TIM_CHANNEL_2};
  uint32_t counters[5] = {rawCounter, rawCounter2, rawCounter3, rawCounter4, rawCounter5};
  uint16_t pwmVal = useReverse ? 3650 : 2650;
  for (int i = 0; i < 5; i++) {
    if (twoMotorActive[i])
      __HAL_TIM_SET_COMPARE(htimList[i], chList[i], pwmVal);
  }
  if (newSector == 0) {
    for (int i = 0; i < 5; i++) {
      if (twoMotorActive[i] && (counters[i] <= RAW_TOLERANCE || counters[i] >= (MAX_RAW - RAW_TOLERANCE))) {
        __HAL_TIM_SET_COMPARE(htimList[i], chList[i], 0);
        twoMotorActive[i] = 0;
      }
    }
  } else {
    for (int i = 0; i < 5; i++) {
      if (twoMotorActive[i] && abs((int32_t)counters[i] - (int32_t)sectorRaw[newSector]) <= RAW_TOLERANCE) {
        __HAL_TIM_SET_COMPARE(htimList[i], chList[i], 0);
        twoMotorActive[i] = 0;
      }
    }
  }
  bool any = false;
  for (int i = 0; i < 5; i++) {
    if (twoMotorActive[i]) { any = true; break; }
  }
  if (!any) {
    currentSector = newSector;
    sectorUpdated = 0;
    directionDone = 1;
  }
}

void changeSpeed(void) {
  uint32_t pwm_value = 3150;
  int Vector = currentVectorLength;
  if (currentVectorLength > 1) {
    pwm_value = speed[Vector];
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, pwm_value);
  } else {
    stopMotor(1);
  }
}

void runSubMode1(void) {
/*  static const uint32_t sectorRaw[6] = {
    SECTOR0_RAW, SECTOR1_RAW, SECTOR2_RAW,
    SECTOR3_RAW, SECTOR4_RAW, SECTOR5_RAW
  };*/

  uint32_t counters[5] = {rawCounter, rawCounter2, rawCounter3, rawCounter4, rawCounter5};
  TIM_HandleTypeDef* htimList[5] = {&htim5, &htim5, &htim5, &htim9, &htim9};
  uint32_t chList[5] = {TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4, TIM_CHANNEL_1, TIM_CHANNEL_2};
  bool allReached = true;

  for (int i = 0; i < 5; i++) {
    uint32_t target = (i < 3) ? 35 : 11;  // 3개는 32, 2개는 8
    int32_t diff = (int32_t)counters[i] - (int32_t)target;

    if (abs(diff) <= RAW_TOLERANCE) {
      __HAL_TIM_SET_COMPARE(htimList[i], chList[i], 0);  // 도달하면 정지
    } else {
      __HAL_TIM_SET_COMPARE(htimList[i], chList[i], (diff > 0) ? 2800 : 3500);  // 방향 판단해서 회전
      allReached = false;
    }
  }

  if (allReached) {
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 4000);  // 1층 모터 동작
  }
}


void runSubMode2(void) {
    static bool motorAligned = false;
    static uint32_t targetRaw = 0;

    uint32_t counters[5] = {rawCounter, rawCounter2, rawCounter3, rawCounter4, rawCounter5};
    TIM_HandleTypeDef* htimList[5] = {&htim5, &htim5, &htim5, &htim9, &htim9};
    uint32_t chList[5] = {TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4, TIM_CHANNEL_1, TIM_CHANNEL_2};

    // 1층 모터는 항상 천천히 회전
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 3780);

    if (!motorAligned) {
        targetRaw = sub2_sequence[(currentTargetSector + 5) % 6];
        bool allReached = true;
        for (int i = 0; i < 5; i++) {
            int32_t diff = (int32_t)targetRaw - (int32_t)counters[i];
            if (diff > 24) diff -= 48;
            else if (diff < -24) diff += 48;

            if (abs(diff) <= RAW_TOLERANCE) {
                __HAL_TIM_SET_COMPARE(htimList[i], chList[i], 0);
            } else {
                __HAL_TIM_SET_COMPARE(htimList[i], chList[i], (diff > 0) ? 3600 : 2700);
                allReached = false;
            }
        }
        if (allReached) {
            motorAligned = true;
        }
    } else {
        // 섹터 변경 및 다음 정렬 준비
        currentTargetSector = (currentTargetSector + 1) % 6;
        motorAligned = false;
    }
}




void CDC_ReceiveCallback(uint8_t *buf, uint32_t len)
{
  if (len > 0) {
    received_byte = buf[0];
    if (received_byte == 'r') modeFlag = 0;
    else if (received_byte == 's') modeFlag = 1;
    else if (received_byte == 'c') modeFlag = 2;
    else if (received_byte == 't') {
          modeFlag = 3;
          initialSectorSet = 0;
    }
    else if (modeFlag == 3 && !initialSectorSet) {
                currentTargetSector = received_byte & 0x0F;
                initialSectorSet = 1;
            }



    if (modeFlag == 1 && (received_byte != 's')) {
      newSector = (received_byte >> 4) & 0x0F;
      currentVectorLength = received_byte & 0x0F;
      for (int i = 0; i < 5; i++) twoMotorActive[i] = 1;
      sectorUpdated = 1;
      directionDone = 0;
    }
  }
}

void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 168;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK |
	RCC_CLOCKTYPE_SYSCLK |
	RCC_CLOCKTYPE_PCLK1 |
	RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	  }
}
static void MX_NVIC_Init(void) {
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
	HAL_NVIC_SetPriority(TIM1_BRK_TIM9_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(TIM1_BRK_TIM9_IRQn);
	HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(TIM2_IRQn);
	HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(TIM3_IRQn);
	HAL_NVIC_SetPriority(TIM4_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(TIM4_IRQn);
	HAL_NVIC_SetPriority(TIM8_BRK_TIM12_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(TIM8_BRK_TIM12_IRQn);
}
void Error_Handler(void) { __disable_irq(); while (1) {} }
#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {}
#endif
