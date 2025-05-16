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
// 섹터 관련 상수
#define SECTOR_COUNT 6
#define SECTOR_ANGLE_RAW 8  // 60도에 해당하는 raw counter 값
#define RAW_TOLERANCE 4

// 모드 관련 상수
#define MODE_CENTER 1
#define MODE_CIRCLE 2
#define MODE_CUSTOM 3

// UART 명령어 관련 상수
#define CMD_MODE_CENTER 'c'
#define CMD_MODE_CIRCLE 'r'
#define CMD_MODE_CUSTOM 'm'

// 모터 제어 관련 상수
#define MOTOR_PWM_VALUE 3500
#define MOTOR_REVERSE_VALUE 2800
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
// UART 관련 변수
uint8_t g_RxBuff[1];
uint8_t received_byte;

// 섹터 관련 변수
uint32_t rawCounter = 0;
int currentSector = 0;
int newSector = 0;
int previousSector = -1;

// 모드 관련 변수
uint8_t g_currentMode = MODE_CENTER;
uint8_t targetMode;

// 벡터 관련 변수
uint8_t currentVectorLength = 0;
uint8_t sectorUpdated = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
void initMotors(void);
void stopMotors(void);
void rotateMotor(int motorCh, int direction);
void updateSector(int targetSector);
void updateSpeed(void);
void changeMode(uint8_t cmd);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// 모터 초기화
void initMotors(void) {
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // 1층
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); // 2층
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
}

// 모든 모터 정지
void stopMotors(void) {
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
}

// 모터 회전 (direction: 1=정방향, -1=역방향)
void rotateMotor(int motorCh, int direction) {
    uint32_t pwmValue = (direction > 0) ? MOTOR_PWM_VALUE : MOTOR_REVERSE_VALUE;
    
    switch(motorCh) {
        case 1:
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwmValue);
            break;
        case 2:
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pwmValue);
            break;
        case 3:
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, pwmValue);
            break;
        case 4:
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, pwmValue);
            break;
        default:
            // 잘못된 채널 번호 처리
            break;
    }
}

// 섹터 업데이트
void updateSector(int targetSector) {
    if(currentSector == targetSector) return;

    // 정방향으로 회전할 때의 각도 차이 계산
    int32_t forwardDiff = (targetSector - currentSector + SECTOR_COUNT) % SECTOR_COUNT;
    // 역방향으로 회전할 때의 각도 차이 계산
    int32_t reverseDiff = (currentSector - targetSector + SECTOR_COUNT) % SECTOR_COUNT;

    // 최단 경로로 회전
    int direction = (forwardDiff <= reverseDiff) ? 1 : -1;
    rotateMotor(2, direction);

    // 목표 위치 도달 확인
    int targetRaw = targetSector * SECTOR_ANGLE_RAW;
    if(abs(rawCounter - targetRaw) <= RAW_TOLERANCE) {
        stopMotors();
        currentSector = targetSector;
        sectorUpdated = 0;
    }
}

// 속도 업데이트
void updateSpeed(void) {
    if(currentVectorLength > 3) {
        uint32_t pwmValue = 3150 + (currentVectorLength * 117);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwmValue);
    } else {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
    }
}

// 모드 전환
void changeMode(uint8_t cmd) {
    printf("changeMode()\r\n");
    
    // 명령어에 따른 목표 모드 설정
    switch(cmd) {
        case CMD_MODE_CENTER:
            printf("cmd = %d (CMD_MODE_CENTER)\r\n", cmd);
            targetMode = MODE_CENTER;
            break;
        case CMD_MODE_CIRCLE:
            printf("cmd = %d (CMD_MODE_CIRCLE)\r\n", cmd);
            targetMode = MODE_CIRCLE;
            break;
        case CMD_MODE_CUSTOM:
            printf("cmd = %d (CMD_MODE_CUSTOM)\r\n", cmd);
            targetMode = MODE_CUSTOM;
            break;
        default:
            printf("cmd = %d (Unknown command)\r\n", cmd);
            return;
    }
    
    if (g_currentMode == targetMode) {
        printf("이미 해당 모드입니다.\r\n");
        return;
    }
    
    printf("모드 전환: %d -> %d\r\n", g_currentMode, targetMode);
    
    // 현재 모드 종료
    stopMotors();
    
    // 새 모드 시작
    switch (targetMode) {
        case MODE_CENTER:
            printf("center mode init\r\n");
            sectorUpdated = 0;
            break;
        case MODE_CIRCLE:
            printf("circle mode init\r\n");
            rotateMotor(1, 1);  // 1층 모터 정방향 회전
            previousSector = currentSector;
            break;
        case MODE_CUSTOM:
            printf("custom mode init\r\n");
            break;
    }
    
    g_currentMode = targetMode;
    printf("모드 전환 완료\r\n");
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    received_byte = g_RxBuff[0];

    // 모드 변경 명령 처리
    if (received_byte == CMD_MODE_CENTER || 
        received_byte == CMD_MODE_CIRCLE || 
        received_byte == CMD_MODE_CUSTOM) {
        changeMode(received_byte);
    } else {
        // 섹터 및 벡터 정보 업데이트
        newSector = (received_byte >> 4) & 0x0F;
        currentVectorLength = received_byte & 0x0F;

        if (g_currentMode == MODE_CENTER) {
            sectorUpdated = 1;
        }
    }
    
    // Circle 모드가 아닐 때만 다음 수신 대기
    if (g_currentMode != MODE_CIRCLE) {
        HAL_UART_Receive_IT(&huart2, g_RxBuff, 1);
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
    MX_USART2_UART_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();

    /* Initialize interrupts */
    MX_NVIC_Init();

    /* USER CODE BEGIN 2 */
    initMotors();
    HAL_UART_Receive_IT(&huart2, g_RxBuff, 1);
    printf("main init\r\n");
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        rawCounter = __HAL_TIM_GET_COUNTER(&htim1);

        switch (g_currentMode) {
            case MODE_CENTER:
                if(sectorUpdated) {
                    updateSector(newSector);
                }
                updateSpeed();
                break;
                
            case MODE_CIRCLE:
                if (newSector != previousSector) {
                    int targetRaw = (rawCounter + SECTOR_ANGLE_RAW) % (SECTOR_COUNT * SECTOR_ANGLE_RAW);
                    if (abs(rawCounter - targetRaw) > RAW_TOLERANCE) {
                        rotateMotor(2, 1);
                    } else {
                        stopMotors();
                        previousSector = newSector;
                    }
                }
                break;
                
            case MODE_CUSTOM:
                // 향후 구현
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
    /* USER CODE BEGIN SystemClock_Config */
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
    /* USER CODE END SystemClock_Config */
}

/**
 * @brief NVIC Configuration.
 * @retval None
 */
static void MX_NVIC_Init(void)
{
    /* USER CODE BEGIN NVIC_Init */
    /* USART2_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
    /* USER CODE END NVIC_Init */
}

/* USER CODE BEGIN 4 */
int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, 100);
    return 0;
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
