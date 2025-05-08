import serial
import time

# 시리얼 포트 설정
ser = serial.Serial(
    port='COM3',  # Windows 기준임
    baudrate=115200,  # STM32 코드와 동일한 baudrate 설정
    bytesize=serial.EIGHTBITS, # serial library에 기본
    parity=serial.PARITY_NONE, # serial library에 기본
    stopbits=serial.STOPBITS_ONE, # serial library에 기본
    timeout=1 # serial library에 기₩
)
def send_message(message):
    # 메시지 전송
    ser.write(message.encode()) # 실질적으로 메세지 전송하는 함수
    time.sleep(0.1)  # 잠시 대기
def receive_message():
    # 수신된 데이터가 있으면 읽기
    if ser.in_waiting > 0: # 시리얼 포트에 읽을 수 있는 데이터가 하나 이상 있다면
        return ser.readline().decode().strip() # 디코드 해서 출력
    return None
try:
    while True:
        # 메시지 전송
        send_message('a') # 일단 a 한 글자만 보내보기

        # 수신 대기 및 출력
        received = receive_message() # 버퍼에 데이터가 있으면 received 변수에 대입
        if received: # 해당 변수에 데이터가 대입되어 있다면
            print(f"수신된 데이터: {received}") # 출력
            # 나중에 우리는 방향제어를 해야하므로, 신호를 한 번 줬으면 received 변수를 초기화해줘야 할 것 같다. 
            # received = 0 이렇게 변수를 쓴 다음에는 0으로 초기화 한 번 해줘야할 듯

        time.sleep(1) # 1초마다 실행

except KeyboardInterrupt:
    print("프로그램 종료")
    ser.close()