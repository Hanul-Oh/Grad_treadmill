import cv2
import numpy as np
import torch
from ultralytics import YOLO
import time
from filterpy.kalman import KalmanFilter
import math

# --- Constants ---
MOTOR_STEPS = 30  # 모터의 총 스텝 수
ANGLE_PER_SECTOR = 360 / MOTOR_STEPS  # 각 구역당 각도
DETECTION_CONFIDENCE = 0.5  # YOLO 탐지 신뢰도 임계값
TARGET_CLASS = 67  # Cell Phone 클래스 ID

# --- Helper functions for vector direction ---
def draw_sector_grid(frame):
    """
    화면에 모터 스텝 수에 따른 구역 그리드를 그립니다.
    
    Parameters:
        frame: numpy.ndarray - 그리드를 그릴 이미지 프레임
    """
    h, w = frame.shape[:2]
    center = (w//2, h//2)
    radius = min(w, h) // 2 - 50  # 화면 크기에 맞게 반지름 조정
    
    # 중심점 그리기
    # cv2.circle(image, center, radius, color, thickness)
    # image: 그릴 이미지
    # center: 원의 중심점 (x, y) 좌표
    # radius: 원의 반지름
    # color: 원의 색상 (B, G, R) 형식
    # thickness: 선의 두께 (-1이면 채워진 원)
    cv2.circle(frame, center, 5, (255, 255, 255), -1)
    
    # 각 구역의 경계선 그리기
    for i in range(MOTOR_STEPS):
        angle = i * ANGLE_PER_SECTOR
        rad = math.radians(angle)
        end_x = int(center[0] + radius * math.cos(rad))
        end_y = int(center[1] - radius * math.sin(rad))
        
        # 구역 경계선
        # cv2.line(image, start_point, end_point, color, thickness)
        # image: 그릴 이미지
        # start_point: 선의 시작점 (x, y) 좌표
        # end_point: 선의 끝점 (x, y) 좌표
        # color: 선의 색상 (B, G, R) 형식
        # thickness: 선의 두께
        cv2.line(frame, center, (end_x, end_y), (100, 100, 100), 1)
        
        # 구역 번호 표시
        text_radius = radius - 30
        text_x = int(center[0] + text_radius * math.cos(rad))
        text_y = int(center[1] - text_radius * math.sin(rad))
        # cv2.putText(image, text, position, font, scale, color, thickness)
        # image: 텍스트를 그릴 이미지
        # text: 표시할 텍스트
        # position: 텍스트의 시작 위치 (x, y) 좌표
        # font: 폰트 종류
        # scale: 폰트 크기
        # color: 텍스트 색상 (B, G, R) 형식
        # thickness: 텍스트 두께
        cv2.putText(frame, str(i), (text_x-10, text_y+5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)

def calculate_vector_direction(start_point, end_point):
    """
    두 점 사이의 벡터 방향을 계산합니다.
    최적화: atan2 함수를 한 번만 호출하고, 불필요한 연산 제거
    
    Parameters:
        start_point: tuple - 벡터의 시작점 (x, y) 좌표
        end_point: tuple - 벡터의 끝점 (x, y) 좌표
    
    Returns:
        float: 벡터의 각도 (0-360도)
    """
    dx = end_point[0] - start_point[0]
    dy = end_point[1] - start_point[1]
    angle = math.degrees(math.atan2(-dy, dx))
    return angle if angle >= 0 else angle + 360

def get_sector(angle):
    """
    주어진 각도에 해당하는 구역 번호를 반환합니다.
    최적화: 나눗셈 연산을 한 번만 수행
    
    Parameters:
        angle: float - 계산된 벡터의 각도 (0-360도)
    
    Returns:
        int: 해당하는 구역 번호 (0 ~ MOTOR_STEPS-1)
    """
    return int(angle / ANGLE_PER_SECTOR) % MOTOR_STEPS

def draw_tracking_info(frame, bbox, obj_id, angle, sector):
    """
    화면에 트래킹 정보를 그립니다.
    최적화: 문자열 포맷팅을 한 번만 수행
    
    Parameters:
        frame: numpy.ndarray - 정보를 그릴 이미지 프레임
        bbox: list - 바운딩 박스 좌표 [x1, y1, x2, y2]
        obj_id: int - 트래킹 중인 객체의 ID
        angle: float - 계산된 벡터의 각도
        sector: int - 계산된 구역 번호
    """
    x1, y1, x2, y2 = map(int, bbox)
    center_x = (x1 + x2) // 2
    center_y = (y1 + y2) // 2
    h, w = frame.shape[:2]
    
    # 박스 그리기
    # cv2.rectangle(image, start_point, end_point, color, thickness)
    # image: 그릴 이미지
    # start_point: 사각형의 좌상단 (x, y) 좌표
    # end_point: 사각형의 우하단 (x, y) 좌표
    # color: 사각형의 색상 (B, G, R) 형식
    # thickness: 선의 두께
    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
    
    # 벡터 그리기
    # cv2.arrowedLine(image, start_point, end_point, color, thickness, tipLength)
    # image: 그릴 이미지
    # start_point: 화살표의 시작점 (x, y) 좌표
    # end_point: 화살표의 끝점 (x, y) 좌표
    # color: 화살표의 색상 (B, G, R) 형식
    # thickness: 선의 두께
    # tipLength: 화살표 머리의 길이 (전체 길이의 비율)
    cv2.arrowedLine(frame, (center_x, center_y), (w//2, h//2), (0, 0, 255), 2, tipLength=0.3)
    
    # 정보 텍스트
    info_text = f"ID: {obj_id} | Angle: {angle:.1f}° | Sector: {sector}"
    cv2.putText(frame, info_text, (x1, y1 - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

class KalmanBoxTracker:
    # 클래스 변수: 모든 인스턴스가 공유하는 트래커 ID 카운터
    count = 0

    def __init__(self, bbox):
        """
        칼만 필터 기반 박스 트래커 초기화
        
        Parameters:
            bbox: list - 초기 바운딩 박스 좌표 [x1, y1, x2, y2]
        """
        # 현재 트래커에 고유 ID 할당
        self.id = KalmanBoxTracker.count
        # 다음 트래커를 위한 ID 증가
        KalmanBoxTracker.count += 1
        
        # 칼만 필터 초기화
        # dim_x=7: 상태 벡터 차원 (위치 4개 + 속도 3개)
        # dim_z=4: 측정 벡터 차원 (바운딩 박스 좌표 4개)
        self.kf = KalmanFilter(dim_x=7, dim_z=4)
        
        # 상태 전이 행렬 설정 (7x7 단위 행렬)
        # 물체의 움직임을 예측하는데 사용
        self.kf.F = np.eye(7)
        
        # 측정 행렬 설정 (4x7 단위 행렬)
        # 실제 측정값과 상태 벡터를 연결
        self.kf.H = np.eye(4, 7)
        
        # 공분산 행렬 초기화 (7x7)
        # 예측의 불확실성을 나타내는 행렬
        # 큰 값으로 초기화하여 초기 예측의 불확실성 반영
        self.kf.P *= 1000
        
        # 측정 노이즈 공분산 행렬 설정 (4x4)
        # 센서(YOLO)의 측정 오차를 나타냄
        self.kf.R *= 10
        
        # 프로세스 노이즈 공분산 행렬 설정 (7x7)
        # 물체 움직임의 불확실성을 나타냄
        self.kf.Q *= 5
        
        # 초기 상태 벡터 설정
        # 바운딩 박스의 초기 위치를 상태 벡터의 처음 4개 요소에 할당
        self.kf.x[:4] = np.array([[bbox[0]], [bbox[1]], [bbox[2]], [bbox[3]]])
        
        # 마지막 업데이트 이후 경과 프레임 수
        # 물체가 얼마나 오래 감지되지 않았는지 추적
        self.time_since_update = 0
        
        # 마지막 업데이트 시간 기록
        # 물체의 추적 지속 시간을 계산하는데 사용
        self.last_update = time.time()

    def predict(self):
        """
        다음 프레임에서의 물체 위치를 예측
        
        Returns:
            numpy.ndarray: 예측된 바운딩 박스 좌표 [x1, y1, x2, y2]
        """
        # 칼만 필터를 사용하여 다음 상태 예측
        self.kf.predict()
        
        # 업데이트 없이 경과한 프레임 수 증가
        self.time_since_update += 1
        
        # 예측된 상태 벡터에서 바운딩 박스 좌표만 추출하여 반환
        # flatten()을 사용하여 1차원 배열로 변환
        return self.kf.x[:4].flatten()

    def update(self, bbox):
        """
        새로운 측정값으로 예측을 보정
        
        Parameters:
            bbox: list - 새로운 바운딩 박스 좌표 [x1, y1, x2, y2]
        """
        # 새로운 측정값으로 칼만 필터 업데이트
        # 바운딩 박스 좌표를 2차원 배열로 변환하여 입력
        self.kf.update(np.array([[bbox[0]], [bbox[1]], [bbox[2]], [bbox[3]]]))
        
        # 업데이트 이후 경과 프레임 수 초기화
        self.time_since_update = 0
        
        # 마지막 업데이트 시간 갱신
        self.last_update = time.time()

class Sort:
    """
    Simple Online and Realtime Tracking (SORT) 알고리즘 구현
    칼만 필터와 IOU 기반의 실시간 다중 객체 추적 알고리즘
    """
    def __init__(self, max_age=2.0, iou_threshold=0.3):
        """
        SORT 트래커 초기화
        
        Parameters:
            max_age: float - 트래커가 업데이트 없이 유지될 수 있는 최대 시간(초)
            iou_threshold: float - 두 박스가 같은 객체로 간주되는 최소 IOU 값
        """
        # 현재 추적 중인 모든 트래커 객체들의 리스트
        self.trackers = []
        # 트래커의 최대 수명 (초 단위)
        self.max_age = max_age
        # IOU 임계값 (이 값보다 큰 IOU를 가진 박스들은 같은 객체로 간주)
        self.iou_threshold = iou_threshold

    def iou(self, box1, box2):
        """
        두 바운딩 박스 간의 Intersection over Union (IOU) 계산
        
        Parameters:
            box1: list - 첫 번째 바운딩 박스 [x1, y1, x2, y2]
            box2: list - 두 번째 바운딩 박스 [x1, y1, x2, y2]
            
        Returns:
            float: 두 박스의 IOU 값 (0~1 사이)
        """
        # 첫 번째 박스의 좌표 추출
        x1, y1, x2, y2 = box1
        # 두 번째 박스의 좌표 추출
        x1g, y1g, x2g, y2g = box2
        
        # 교차 영역의 좌표 계산
        xi1, yi1, xi2, yi2 = max(x1, x1g), max(y1, y1g), min(x2, x2g), min(y2, y2g)
        
        # 교차 영역의 넓이 계산 (음수인 경우 0으로 처리)
        inter_area = max(0, xi2 - xi1) * max(0, yi2 - yi1)
        
        # 각 박스의 넓이 계산
        box1_area = (x2 - x1) * (y2 - y1)
        box2_area = (x2g - x1g) * (y2g - y1g)
        
        # IOU 계산 (교차 영역 / 합집합 영역)
        # 1e-6을 더해서 0으로 나누는 것을 방지
        return inter_area / float(box1_area + box2_area - inter_area + 1e-6)

    def update(self, detections):
        """
        새로운 탐지 결과로 트래커 업데이트
        
        Parameters:
            detections: numpy.ndarray - YOLO가 탐지한 바운딩 박스들의 배열
            
        Returns:
            numpy.ndarray: 업데이트된 트래킹 결과 [x1, y1, x2, y2, id]
        """
        # 현재 시간 기록
        current_time = time.time()
        
        # 오래된 트래커 제거 (max_age를 초과한 트래커)
        self.trackers = [t for t in self.trackers if (current_time - t.last_update) < self.max_age]
        
        # 업데이트된 객체들의 리스트
        updated_objects = []

        # 탐지된 객체가 있는 경우에만 처리
        if len(detections) > 0:
            # 각 탐지 결과에 대해
            for detection in detections:
                best_tracker = None  # 가장 잘 매칭되는 트래커
                max_iou = 0  # 최대 IOU 값

                # 모든 트래커와 IOU 계산
                for tracker in self.trackers:
                    # 트래커의 예측 위치와 탐지 결과의 IOU 계산
                    iou_score = self.iou(tracker.predict(), detection)
                    # 더 높은 IOU를 가진 트래커 선택
                    if iou_score > max_iou:
                        max_iou = iou_score
                        best_tracker = tracker

                # 매칭되는 트래커가 있고 IOU가 임계값을 넘는 경우
                if best_tracker and max_iou > self.iou_threshold:
                    # 트래커 업데이트
                    best_tracker.update(detection)
                    # 업데이트된 객체 정보 저장 [x1, y1, x2, y2, id]
                    updated_objects.append(np.append(best_tracker.kf.x[:4].flatten(), best_tracker.id))
                else:
                    # 새로운 트래커 생성
                    new_tracker = KalmanBoxTracker(detection)
                    self.trackers.append(new_tracker)
                    # 새로운 객체 정보 저장
                    updated_objects.append(np.append(detection, new_tracker.id))

        # 업데이트된 객체가 있으면 배열로 반환, 없으면 빈 배열 반환
        return np.array(updated_objects) if updated_objects else np.empty((0, 5))


model = YOLO('yolov8s.pt', verbose=False)
model.model.to('cpu')
tracker = Sort(max_age=2.0, iou_threshold=0.3)
cap = cv2.VideoCapture(0)

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        # 구역 그리드 그리기
        draw_sector_grid(frame)

        # YOLO 탐지 (모든 출력 제거)
        results = model(frame, imgsz=640, show=False, save=False, conf=DETECTION_CONFIDENCE, stream=True, verbose=False)
        detections = []

        for result in results:
            boxes = result.boxes.data.tolist()
            # 필터링을 한 번에 수행
            detections = [[x1, y1, x2, y2] for x1, y1, x2, y2, conf, cls_id in boxes 
                         if int(cls_id) == TARGET_CLASS]

        # SORT로 트래킹 업데이트
        tracked_objects = tracker.update(np.array(detections) if detections else np.empty((0, 4)))

        # 트래킹된 휴대폰 표시
        if tracked_objects is not None and len(tracked_objects) > 0:
            h, w = frame.shape[:2]
            center_point = (w//2, h//2)
            
            for obj in tracked_objects:
                try:
                    x1, y1, x2, y2, obj_id = map(int, obj)
                    start_point = ((x1 + x2) // 2, (y1 + y2) // 2)
                    
                    # 벡터 방향 계산
                    angle = calculate_vector_direction(start_point, center_point)
                    sector = get_sector(angle)
                    
                    # 정보 표시
                    draw_tracking_info(frame, [x1, y1, x2, y2], obj_id, angle, sector)
                    print(f"Object {obj_id} - Angle: {angle:.1f}°, Sector: {sector}")
                    
                except ValueError as e:
                    print(f"ValueError: {e}, Data: {obj}")

        cv2.imshow("Cell Phone Tracking", frame)

        key = cv2.waitKey(1)
        if key == 27 or key == ord('q'):
            break

except KeyboardInterrupt:
    print("프로그램 종료 요청됨")
except Exception as e:
    print(f"오류 발생: {e}")
finally:
    cap.release()
    cv2.destroyAllWindows()
    print("프로그램이 안전하게 종료됨")

