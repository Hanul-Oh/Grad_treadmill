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
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
//#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SECTOR0_RAW 0
#define SECTOR1_RAW 8
#define SECTOR2_RAW 16
#define SECTOR3_RAW 24
#define SECTOR4_RAW 32
#define SECTOR5_RAW 40
#define RAW_TOLERANCE 2
#define MAX_VECTOR_LENGTH 9  // 최대 벡터 길이 (0-9)

// 빙빙 모드 관련 상수
#define CIRCLE_MODE_LAYER1_PWM 3500
#define CIRCLE_MODE_LAYER2_PWM 3500
#define CIRCLE_MODE_TRANSITION_DELAY 1000  // 1초
#define CIRCLE_MODE_SECTOR_ANGLE 45  // 섹터당 45도 회전
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t g_RxBuff[1];
uint32_t rawCounter = 0;
int currentSector = 0;
int newSector = 0;
uint8_t currentVectorLength = 0;  // 현재 벡터 길이 (0-9)
uint8_t sectorUpdated = 0;
uint8_t g_circleModeEnabled = 0;  // 빙빙 모드 활성화 여부
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void changeSector(int newSector);
void changeSpeed();
void enterCircleMode(void);
void exitCircleMode(void);
void updateCircleMode(void);
void initializeCircleMode(void);
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
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // 1층
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); // 2층
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);

//  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 3500);
//  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 3500);
//  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 4200);
  HAL_UART_Receive_IT(&huart2, g_RxBuff, 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

     rawCounter = __HAL_TIM_GET_COUNTER(&htim1);

     if(sectorUpdated){
        changeSector(newSector);
     }
     
     changeSpeed();

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
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* USART2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
}

/* USER CODE BEGIN 4 */

// 모터 정지
void stopMotor(int motorCh) {
   if(motorCh == 1) // 1층
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);  // PWM 출력을 0으로 설정하여 모터 정지
   if(motorCh == 2) // 2층
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);  // PWM 출력을 0으로 설정하여 모터 정지
}

// 정방향 회전(2층)
void rotateForward(void) {
   __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 3500);  // 정방향 회전을 위한 PWM 값
}

// 역방향 회전(2층)
void rotateReverse(void) {
   __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 2800);  // 역방향 회전을 위한 PWM 값
}

// 구역 변경 함수
void changeSector(int newSector) {
    if(currentSector != newSector) {
        // 정방향으로 회전할 때의 각도 차이 계산
        int32_t forwardDiff = (newSector - currentSector + 6) % 6;
        // 역방향으로 회전할 때의 각도 차이 계산
        int32_t reverseDiff = (currentSector - newSector + 6) % 6;

        // 정방향이 더 빠른 경우
        if(forwardDiff <= reverseDiff) {
            rotateForward();

            switch(newSector) {
                case 0:
                    if(abs(rawCounter - SECTOR0_RAW) <= RAW_TOLERANCE) {  // 0구역
                        stopMotor(2);
                        currentSector = newSector;
                        sectorUpdated = 0;
                    }
                    break;
                case 1:
                    if(abs(rawCounter - SECTOR1_RAW) <= RAW_TOLERANCE) {  // 1구역
                        stopMotor(2);
                        currentSector = newSector;
                        sectorUpdated = 0;
                    }
                    break;
                case 2:
                    if(abs(rawCounter - SECTOR2_RAW) <= RAW_TOLERANCE) {  // 2구역
                        stopMotor(2);
                        currentSector = newSector;
                        sectorUpdated = 0;
                    }
                    break;
                case 3:
                    if(abs(rawCounter - SECTOR3_RAW) <= RAW_TOLERANCE) {  // 3구역
                        stopMotor(2);
                        currentSector = newSector;
                        sectorUpdated = 0;
                    }
                    break;
                case 4:
                    if(abs(rawCounter - SECTOR4_RAW) <= RAW_TOLERANCE) {  // 4구역
                        stopMotor(2);
                        currentSector = newSector;
                        sectorUpdated = 0;
                    }
                    break;
                case 5:
                    if(abs(rawCounter - SECTOR5_RAW) <= RAW_TOLERANCE) {  // 5구역
                        stopMotor(2);
                        currentSector = newSector;
                        sectorUpdated = 0;
                    }
                    break;
            }
        }
        // 역방향이 더 빠른 경우
        else {
            rotateReverse();

            switch(newSector) {
               case 0:
               if(abs(rawCounter - SECTOR0_RAW) <= RAW_TOLERANCE) {  // 0구역
                  stopMotor(2);
                  currentSector = newSector;
                  sectorUpdated = 0;
               }
               break;
            case 1:
               if(abs(rawCounter - SECTOR1_RAW) <= RAW_TOLERANCE) {  // 1구역
                  stopMotor(2);
                  currentSector = newSector;
                  sectorUpdated = 0;
               }
               break;
            case 2:
               if(abs(rawCounter - SECTOR2_RAW) <= RAW_TOLERANCE) {  // 2구역
                  stopMotor(2);
                  currentSector = newSector;
                  sectorUpdated = 0;
               }
               break;
            case 3:
               if(abs(rawCounter - SECTOR3_RAW) <= RAW_TOLERANCE) {  // 3구역
                  stopMotor(2);
                  currentSector = newSector;
                  sectorUpdated = 0;
               }
               break;
            case 4:
               if(abs(rawCounter - SECTOR4_RAW) <= RAW_TOLERANCE) {  // 4구역
                  stopMotor(2);
                  currentSector = newSector;
                  sectorUpdated = 0;
               }
               break;
            case 5:
               if(abs(rawCounter - SECTOR5_RAW) <= RAW_TOLERANCE) {  // 5구역
                  stopMotor(2);
                  currentSector = newSector;
                  sectorUpdated = 0;
               }
               break;
            }
        }
    }
    sectorUpdated = 0;
}

// 이동 속도 조절 함수
void changeSpeed(){
    uint32_t pwm_value = 3150;  // 기본 PWM 값 (정방향 최소값)
    if(currentVectorLength > 3) {
           // 3150-4200 범위로 PWM 값 조정
           pwm_value = 3150 + (currentVectorLength * 117);  // (4200-3150)/9 ≈ 117
          __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwm_value);
    }
    else{
       stopMotor(1);
    }

}

// 빙빙 모드 초기화 함수
void initializeCircleMode(void) {
    // 2층 모터를 초기 위치로 설정 (rawCounter = 0)
    while (rawCounter != 0) {
        if (rawCounter > 24) {
            rotateReverse();
        } else {
            rotateForward();
        }
    }
    stopMotor(2);
}

// 빙빙 모드 진입 함수
void enterCircleMode(void) {
    // 1. 1층 모터 정지
    stopMotor(1);
    HAL_Delay(CIRCLE_MODE_TRANSITION_DELAY);
    
    // 2. 2층 모터 초기화
    initializeCircleMode();
    
    // 3. 1층 모터 회전 시작
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, CIRCLE_MODE_LAYER1_PWM);
    
    // 4. 빙빙 모드 활성화
    g_circleModeEnabled = 1;
}

// 빙빙 모드 종료 함수
void exitCircleMode(void) {
    // 1. 1층 모터 정지
    stopMotor(1);
    HAL_Delay(CIRCLE_MODE_TRANSITION_DELAY);
    
    // 2. 2층 모터 각도 조정 (중앙 모드로)
    changeSector(currentSector);
    
    // 3. 빙빙 모드 비활성화
    g_circleModeEnabled = 0;
}

// 빙빙 모드 업데이트 함수
void updateCircleMode(void) {
    if (!g_circleModeEnabled) return;
    
    // 현재 섹터 확인
    int targetSector = currentSector;
    
    // 2층 모터 각도 계산 및 조정
    int targetRaw = targetSector * 8;  // 각 섹터는 8씩 증가
    
    // 현재 위치와 목표 위치의 차이 계산
    int diff = targetRaw - rawCounter;
    
    // 회전 방향 결정 및 실행
    if (abs(diff) > RAW_TOLERANCE) {
        if (diff > 0) {
            rotateForward();
        } else {
            rotateReverse();
        }
    } else {
        stopMotor(2);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
   if (huart->Instance == USART2) {
           // 수신된 바이트에서 구역 번호와 벡터 길이 추출
           uint8_t received_byte = g_RxBuff[0];
           newSector = (received_byte >> 4) & 0x0F;  // 상위 4비트: 구역 번호
           currentVectorLength = received_byte & 0x0F;  // 하위 4비트: 벡터 길이
           
           // 빙빙 모드가 활성화되어 있으면 updateCircleMode 호출
           if (g_circleModeEnabled) {
               updateCircleMode();
           } else {
               sectorUpdated = 1;
           }
           
           HAL_UART_Receive_IT(&huart2, g_RxBuff, 1);  // 다시 인터럽트 활성화
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

#ifdef  USE_FULL_ASSERT
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
