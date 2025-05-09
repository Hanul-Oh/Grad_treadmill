/*
STM32에서 구역 변경 함수 -> main.c에 삽입할 예정인 코드이다.

rawCounter 변수는 현재 모터의 각도를 저장하는 변수 -> 0부터 29까지 존재함
예를 들어 0°이면 -> rawCounter = 0
30°이면 -> rawCounter = 4
60°이면 -> rawCounter = 8
...
360°이면 -> rawCounter = 29

0구역이면 -> rawCounter는 0으로 설정되게 할 것
1구역이면 -> rawCounter는 5로 설정되게 할 것
2구역이면 -> rawCounter는 10으로 설정되게 할 것
5구역이면 -> rawCounter는 25로 설정되게 할 것
0구역부터 5구역까지만 존재한다. 


현재 몇 구역에 속하는지는 @tredmill_cameraSide.py 파일에서 시리얼 통신으로 STM32에 전달해줌
예를 들어 0구역이면 -> 시리얼 통신으로 0을 전달해줌
1구역이면 -> 시리얼 통신으로 1을 전달해줌
...
5구역이면 -> 시리얼 통신으로 5를 전달해줌
*/


// 구역별 rawCounter 값 정의
#define SECTOR0_RAW 0    // 0구역
#define SECTOR1_RAW 5    // 1구역
#define SECTOR2_RAW 10   // 2구역
#define SECTOR3_RAW 15   // 3구역
#define SECTOR4_RAW 20   // 4구역
#define SECTOR5_RAW 25   // 5구역

// 모터 제어 함수들
void stopMotor(void) {
    HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 0);  // PWM 출력을 0으로 설정하여 모터 정지
}

void rotateForward(void) {
    HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 1000);  // 정방향 회전을 위한 PWM 값
}

void rotateReverse(void) {
    HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 1000);  // 역방향 회전을 위한 PWM 값
}

// 구역 변경 함수
void changeSector(uint8_t newSector) {
    static uint8_t currentSector = 0;  // 현재 구역 저장
    
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
                    if(rawCounter == SECTOR0_RAW) {  // 0구역
                        stopMotor();
                        currentSector = newSector;
                    }
                    break;
                case 1:
                    if(rawCounter == SECTOR1_RAW) {  // 1구역
                        stopMotor();
                        currentSector = newSector;
                    }
                    break;
                case 2:
                    if(rawCounter == SECTOR2_RAW) {  // 2구역
                        stopMotor();
                        currentSector = newSector;
                    }
                    break;
                case 3:
                    if(rawCounter == SECTOR3_RAW) {  // 3구역
                        stopMotor();
                        currentSector = newSector;
                    }
                    break;
                case 4:
                    if(rawCounter == SECTOR4_RAW) {  // 4구역
                        stopMotor();
                        currentSector = newSector;
                    }
                    break;
                case 5:
                    if(rawCounter == SECTOR5_RAW) {  // 5구역
                        stopMotor();
                        currentSector = newSector;
                    }
                    break;
            }
        }
        // 역방향이 더 빠른 경우
        else {
            rotateReverse();
            
            switch(newSector) {
                case 0:
                    if(rawCounter == SECTOR0_RAW) {  // 0구역
                        stopMotor();
                        currentSector = newSector;
                    }
                    break;
                case 1:
                    if(rawCounter == SECTOR1_RAW) {  // 1구역
                        stopMotor();
                        currentSector = newSector;
                    }
                    break;
                case 2:
                    if(rawCounter == SECTOR2_RAW) {  // 2구역
                        stopMotor();
                        currentSector = newSector;
                    }
                    break;
                case 3:
                    if(rawCounter == SECTOR3_RAW) {  // 3구역
                        stopMotor();
                        currentSector = newSector;
                    }
                    break;
                case 4:
                    if(rawCounter == SECTOR4_RAW) {  // 4구역
                        stopMotor();
                        currentSector = newSector;
                    }
                    break;
                case 5:
                    if(rawCounter == SECTOR5_RAW) {  // 5구역
                        stopMotor();
                        currentSector = newSector;
                    }
                    break;
            }
        }
    }
} 