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
#define MOTOR_NEUTRAL 3150
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
void stopMotor(int motorCh);
void rotateForward(void);
void rotateReverse(void);
void rotateMotor(int motorCh, int direction);
void updateSectorCenterMode(int targetSector);
void updateSpeed(void);
void changeMode(uint8_t cmd);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* === 초기화 함수 === */
void initMotors(void) {
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // 1층
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); // 2층
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
}

/* === 모터 제어 === */
void stopMotors(void) {
    stopMotor(1);
    stopMotor(2);
}

void stopMotor(int motorCh) {
    if (motorCh == 1)
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, MOTOR_NEUTRAL);
    else if (motorCh == 2)
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, MOTOR_NEUTRAL);
}

void rotateForward(void) {
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, MOTOR_PWM_VALUE);
}

void rotateReverse(void) {
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, MOTOR_REVERSE_VALUE);
}

void rotateMotor(int motorCh, int direction) {
    uint32_t pwmValue = (direction > 0) ? MOTOR_PWM_VALUE : MOTOR_REVERSE_VALUE;

    if (motorCh == 1)
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwmValue);
    else if (motorCh == 2)
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pwmValue);
}

/* === 섹터 회전 === */
void updateSectorCenterMode(int targetSector) {
    if (currentSector == targetSector) return;

    int forwardDiff = (targetSector - currentSector + SECTOR_COUNT) % SECTOR_COUNT;
    int reverseDiff = (currentSector - targetSector + SECTOR_COUNT) % SECTOR_COUNT;

    int direction = (forwardDiff <= reverseDiff) ? 1 : -1;
    if (direction == 1) rotateForward();
    else rotateReverse();

    int targetRaw = targetSector * SECTOR_ANGLE_RAW;
    if (abs((int)rawCounter - targetRaw) <= RAW_TOLERANCE) {
        stopMotor(2);
        currentSector = targetSector;
        sectorUpdated = 0;
    }
}

/* === 속도 제어 === */
void updateSpeed(void) {
    if (currentVectorLength > 3) {
        uint32_t pwm = 3150 + (currentVectorLength * 117); // 3150~4200
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwm);
    } else {
        stopMotor(1);
    }
}

/* === 모드 전환 === */
void changeMode(uint8_t cmd) {
    printf("changeMode()\r\n");
    
    switch (cmd) {
        case CMD_MODE_CENTER:
            printf("CMD_MODE_CENTER\r\n");
            targetMode = MODE_CENTER;
            break;
        case CMD_MODE_CIRCLE:
            printf("CMD_MODE_CIRCLE\r\n");
            targetMode = MODE_CIRCLE;
            break;
        case CMD_MODE_CUSTOM:
            printf("CMD_MODE_CUSTOM\r\n");
            targetMode = MODE_CUSTOM;
            break;
        default:
            printf("Unknown Command: %d\r\n", cmd);
            return;
    }
    
    if (g_currentMode == targetMode) {
        printf("이미 해당 모드입니다.\r\n");
        return;
    }
    
    printf("모드 전환: %d -> %d\r\n", g_currentMode, targetMode);

    stopMotors();

    switch (targetMode) {
        case MODE_CENTER:
            printf("CENTER 모드 시작\r\n");
            sectorUpdated = 0;
            break;
        case MODE_CIRCLE:
            printf("CIRCLE 모드 시작\r\n");
            rotateMotor(1, 1);
            previousSector = currentSector;
            break;
        case MODE_CUSTOM:
            printf("CUSTOM 모드 시작\r\n");
            break;
    }
    
    g_currentMode = targetMode;
    printf("모드 전환 완료\r\n");
}

/* === UART 수신 콜백 === */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        received_byte = g_RxBuff[0];

        if (received_byte == CMD_MODE_CENTER ||
            received_byte == CMD_MODE_CIRCLE ||
            received_byte == CMD_MODE_CUSTOM) {
            changeMode(received_byte);
        } else {
            newSector = (received_byte >> 4) & 0x0F;
            currentVectorLength = received_byte & 0x0F;
            if (g_currentMode == MODE_CENTER) {
                sectorUpdated = 1;
            }
        }

        HAL_UART_Receive_IT(&huart2, g_RxBuff, 1); // 항상 다시 활성화
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
                    updateSectorCenterMode(newSector);
                }
                updateSpeed();
                break;

            case MODE_CIRCLE:
                if (newSector != previousSector) {
                    int targetRaw = (rawCounter + SECTOR_ANGLE_RAW) % (SECTOR_COUNT * SECTOR_ANGLE_RAW); // 일단 순방향으로만 증가 가능(0구역 -> 1구역 은 되지만, 1구역 -> 0구역 은 안됨)
                    if (abs(rawCounter - targetRaw) > RAW_TOLERANCE) {
                        // 이거 순방향으로 돌아야하는지 역방향으로 돌아야하는지 모름. 1층 모터와 2층 모터의 회전 방향에 따른 물체의 이동을 재확인하자 
                        rotateMotor(2, 1);
                        printf("rotateMotor(2, 1)\r\n");
                    } else {
                        stopMotor(2);  // 2층 모터만 정지
                        previousSector = newSector;
                        printf("섹터별 회전각 조정 완료, 현재 섹터: %d\r\n", previousSector);
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
