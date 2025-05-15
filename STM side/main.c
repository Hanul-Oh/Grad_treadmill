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
#define CIRCLE_MODE_SECTOR_ANGLE 60  // 섹터당 60도 회전 (360도/6)
#define CIRCLE_MODE_RADIUS_TIME 1000  // 회전 반지름 이동 시간 (ms)
#define CIRCLE_MODE_90_DEGREE_RAW 12  // 90도 회전에 해당하는 raw counter 값
#define CIRCLE_MODE_60_DEGREE_RAW 8   // 60도 회전에 해당하는 raw counter 값

// UART 명령어 관련 상수
#define CMD_MODE_CENTER 'c'  // 중앙 모드 전환 명령
#define CMD_MODE_CIRCLE 'r'  // 빙빙 모드 전환 명령
#define CMD_MODE_CUSTOM 'm'  // 커스텀 모드 전환 명령
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

// 운영 모드 정의
typedef enum {
    MODE_CENTER = 0,    // 중앙으로 보내는 모드
    MODE_CIRCLE = 1,    // 빙빙 도는 모드
    MODE_CUSTOM = 2     // 추가 예정인 커스텀 모드
} OperationMode_t;

OperationMode_t g_currentMode = MODE_CENTER;  // 기본 모드: 중앙 모드

// 빙빙 모드 상태 관리
typedef enum {
    CIRCLE_STATE_MOVING_UP,      // 위로 이동 중
    CIRCLE_STATE_ROTATING_90,    // 90도 회전 중
    CIRCLE_STATE_CIRCLE_MOTION   // 원 운동 중
} CircleState_t;

CircleState_t g_circleState = CIRCLE_STATE_MOVING_UP;
uint32_t g_circleStartTime = 0;
int g_previousSector = -1;  // 이전 sector 저장용
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
void rotateForward(void);
void rotateReverse(void);
void stopMotor(int motorCh);

// 모드 관련 함수 선언
void changeMode(OperationMode_t newMode);
void initCenterMode(void);
void updateCenterMode(void);

void initCircleMode(void);
void updateCircleMode(void);
void exitCircleMode(void);

void initCustomMode(void);
void updateCustomMode(void);
void exitCustomMode(void);

// UART 명령 처리 함수
void processCommand(uint8_t cmd);
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
    // 엔코더 값 읽기
    rawCounter = __HAL_TIM_GET_COUNTER(&htim1);

    // 현재 모드에 따른 업데이트 처리
    switch (g_currentMode) {
        case MODE_CENTER:
            updateCenterMode();
            break;
            
        case MODE_CIRCLE:
            // 빙빙 모드는 UART 인터럽트에서 처리됨
            break;
            
        case MODE_CUSTOM:
            updateCustomMode();
            break;
    }

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

// 모드 전환 함수
void changeMode(OperationMode_t newMode) {
    // 현재 모드와 동일하면 무시
    if (g_currentMode == newMode) return;
    
    // 현재 모드 종료 처리
    switch (g_currentMode) {
        case MODE_CENTER:
            // 중앙 모드는 특별한 종료 처리 필요 없음
            stopMotor(1);
            stopMotor(2);
            break;
        case MODE_CIRCLE:
            exitCircleMode();
            break;
        case MODE_CUSTOM:
            exitCustomMode();
            break;
    }
    
    // 새 모드 시작 처리
    switch (newMode) {
        case MODE_CENTER:
            initCenterMode();
            break;
        case MODE_CIRCLE:
            initCircleMode();
            break;
        case MODE_CUSTOM:
            initCustomMode();
            break;
    }
    
    // 현재 모드 업데이트
    g_currentMode = newMode;
}

// 중앙 모드 초기화
void initCenterMode(void) {
    // 중앙 모드는 기본 설정으로 초기화
    sectorUpdated = 0;
}

// 중앙 모드 업데이트
void updateCenterMode(void) {
    // 기존 중앙 모드 로직
    if(sectorUpdated){
        changeSector(newSector);
    }
    changeSpeed();
}

// 빙빙 모드 초기화 함수
void initCircleMode(void) {
    // 1. 1층 모터 정지
    stopMotor(1);
    HAL_Delay(CIRCLE_MODE_TRANSITION_DELAY);
    
    // 2. 2층 모터 초기화
    // 2층 모터를 초기 위치로 설정 (rawCounter = 0)
    while (rawCounter != 0) {
        if (rawCounter > 24) {
            rotateReverse();
        } else {
            rotateForward();
        }
    }
    stopMotor(2);
    
    // 3. 상태 초기화
    g_circleState = CIRCLE_STATE_MOVING_UP;
    g_circleStartTime = HAL_GetTick();
    g_previousSector = currentSector;
    
    // 4. 1층 모터 회전 시작 (1초간 위로 이동)
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, CIRCLE_MODE_LAYER1_PWM);
}

// 빙빙 모드 종료 함수
void exitCircleMode(void) {
    // 1. 1층 모터 정지
    stopMotor(1);
    stopMotor(2);
    HAL_Delay(CIRCLE_MODE_TRANSITION_DELAY);
    
    // 2. 2층 모터 각도 조정 (중앙 모드로)
    changeSector(currentSector);
}

// 빙빙 모드 업데이트 함수
void updateCircleMode(void) {
    uint32_t currentTime = HAL_GetTick();
    
    switch(g_circleState) {
        case CIRCLE_STATE_MOVING_UP:
            // 1단계: 1초 동안 위로 이동
            if ((currentTime - g_circleStartTime) >= CIRCLE_MODE_RADIUS_TIME) {
                // 1초 완료, 1층 모터 정지
                stopMotor(1);
                
                // 2단계로 전환
                g_circleState = CIRCLE_STATE_ROTATING_90; // 90도 회전 중이라는 플래그
                
                // 2층 모터 90도 회전 시작
                rotateForward();
            }
            break;
            
        case CIRCLE_STATE_ROTATING_90:
            // 2단계: 90도 회전
            // 목표 위치: 초기 위치(0) + 90도(12)
            // 음수 처리를 포함한 비교
            int diff = abs((rawCounter + 48) % 48 - CIRCLE_MODE_90_DEGREE_RAW);
            if (diff <= RAW_TOLERANCE) {
                // 90도 회전 완료
                stopMotor(2);
                
                // 3단계로 전환
                g_circleState = CIRCLE_STATE_CIRCLE_MOTION;
                
                // 1층 모터 다시 시작 (원 운동)
                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, CIRCLE_MODE_LAYER1_PWM);
            }
            break;
            
        case CIRCLE_STATE_CIRCLE_MOTION:
            // 3단계: 원 운동
            // sector가 변경되었는지 확인
            if (currentSector != g_previousSector) {
                // sector 변경 시 2층 모터 60도 회전
                int sectorDiff = (currentSector - g_previousSector + 6) % 6;
                int targetRaw = (rawCounter + (sectorDiff * CIRCLE_MODE_60_DEGREE_RAW)) % 48;
                
                // 목표 위치로 회전
                if (rawCounter != targetRaw) {
                    int diff = targetRaw - rawCounter;
                    
                    // 최단 경로 계산 (48로 나눈 나머지 고려)
                    if (diff > 24) diff -= 48;
                    if (diff < -24) diff += 48;
                    
                    if (abs(diff) > RAW_TOLERANCE) {
                        if (diff > 0) {
                            rotateForward();
                        } else {
                            rotateReverse();
                        }
                    } else {
                        stopMotor(2);
                        g_previousSector = currentSector;
                    }
                }
            }
            break;
    }
}

// 커스텀 모드 초기화 함수 (나중에 구현)
void initCustomMode(void) {
    // 향후 구현
    stopMotor(1);
    stopMotor(2);
}

// 커스텀 모드 업데이트 함수 (나중에 구현)
void updateCustomMode(void) {
    // 향후 구현
}

// 커스텀 모드 종료 함수 (나중에 구현)
void exitCustomMode(void) {
    // 향후 구현
    stopMotor(1);
    stopMotor(2);
}

// UART 명령어 처리 함수
void processCommand(uint8_t cmd) {
    switch(cmd) {
        case CMD_MODE_CENTER:
            changeMode(MODE_CENTER);
            break;
            
        case CMD_MODE_CIRCLE:
            changeMode(MODE_CIRCLE);
            break;
            
        case CMD_MODE_CUSTOM:
            changeMode(MODE_CUSTOM);
            break;
            
        // 향후 더 많은 명령어 추가 가능
        
        default:
            // 알 수 없는 명령은 무시
            break;
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        // 수신된 바이트 처리
        uint8_t received_byte = g_RxBuff[0];
        
        // 모드 변경 명령인지 확인
        if (received_byte == CMD_MODE_CENTER || 
            received_byte == CMD_MODE_CIRCLE || 
            received_byte == CMD_MODE_CUSTOM) {
            
            // 모드 변경 명령 처리
            processCommand(received_byte);
        } 
        else {
            // 일반 데이터 처리 (기존 로직)
            newSector = (received_byte >> 4) & 0x0F;  // 상위 4비트: 구역 번호
            currentVectorLength = received_byte & 0x0F;  // 하위 4비트: 벡터 길이
            
            // 현재 모드에 따른 처리
            switch (g_currentMode) {
                case MODE_CENTER:
                    sectorUpdated = 1;
                    break;
                    
                case MODE_CIRCLE:
                    updateCircleMode();
                    break;
                    
                case MODE_CUSTOM:
                    // 향후 구현
                    break;
            }
        }
        
        // 다시 인터럽트 활성화
        HAL_UART_Receive_IT(&huart2, g_RxBuff, 1);
    }
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
                // ... 다른 case 문들 ...
            }
        }
        // 역방향이 더 빠른 경우
        else {
            rotateReverse();
            
            switch(newSector) {
                // ... case 문들 ...
            }
        }
    }
    sectorUpdated = 0;
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

