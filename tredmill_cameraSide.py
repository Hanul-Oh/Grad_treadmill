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

# --- Helper functions for grid and direction ---
def draw_grid(frame):
    h, w, _ = frame.shape
    step_x = w // 3
    step_y = h // 3

    for i in range(1, 3):
        cv2.line(frame, (step_x * i, 0), (step_x * i, h), (255, 255, 255), 1)
        cv2.line(frame, (0, step_y * i), (w, step_y * i), (255, 255, 255), 1)

def get_direction(cx, cy, w, h):
    col = cx // (w // 3)
    row = cy // (h // 3)
    grid_index = int(row * 3 + col)

    directions = {
        0: "down-right", 1: "down",     2: "down-left",
        3: "right",      4: "center",   5: "left",
        6: "up-right",   7: "up",       8: "up-left"
    }
    return directions.get(grid_index, "unknown")

# --- Helper functions for vector direction ---
def calculate_vector_direction(start_point, end_point):
    """
    두 점 사이의 벡터 방향을 계산합니다.
    최적화: atan2 함수를 한 번만 호출하고, 불필요한 연산 제거
    """
    dx = end_point[0] - start_point[0]
    dy = end_point[1] - start_point[1]
    angle = math.degrees(math.atan2(-dy, dx))
    return angle if angle >= 0 else angle + 360

def get_sector(angle):
    """
    주어진 각도에 해당하는 구역 번호를 반환합니다.
    최적화: 나눗셈 연산을 한 번만 수행
    """
    return int(angle / ANGLE_PER_SECTOR) % MOTOR_STEPS

def draw_tracking_info(frame, bbox, obj_id, angle, sector):
    """
    화면에 트래킹 정보를 그립니다.
    최적화: 문자열 포맷팅을 한 번만 수행
    """
    x1, y1, x2, y2 = map(int, bbox)
    center_x = (x1 + x2) // 2
    center_y = (y1 + y2) // 2
    h, w = frame.shape[:2]
    
    # 박스 그리기
    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
    
    # 벡터 그리기
    cv2.arrowedLine(frame, (center_x, center_y), (w//2, h//2), (0, 0, 255), 2, tipLength=0.3)
    
    # 정보 텍스트
    info_text = f"ID: {obj_id} | Angle: {angle:.1f}° | Sector: {sector}"
    cv2.putText(frame, info_text, (x1, y1 - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

class KalmanBoxTracker:
    count = 0

    def __init__(self, bbox):
        self.id = KalmanBoxTracker.count
        KalmanBoxTracker.count += 1
        self.kf = KalmanFilter(dim_x=7, dim_z=4)
        self.kf.F = np.eye(7)
        self.kf.H = np.eye(4, 7)
        self.kf.P *= 1000
        self.kf.R *= 10
        self.kf.Q *= 5
        self.kf.x[:4] = np.array([[bbox[0]], [bbox[1]], [bbox[2]], [bbox[3]]])
        self.time_since_update = 0
        self.last_update = time.time()

    def predict(self):
        self.kf.predict()
        self.time_since_update += 1
        return self.kf.x[:4].flatten()

    def update(self, bbox):
        self.kf.update(np.array([[bbox[0]], [bbox[1]], [bbox[2]], [bbox[3]]]))
        self.time_since_update = 0
        self.last_update = time.time()


class Sort:
    def __init__(self, max_age=2.0, iou_threshold=0.3):
        self.trackers = []
        self.max_age = max_age  # 최대 트래커 수명 (초)
        self.iou_threshold = iou_threshold

    def iou(self, box1, box2):
        x1, y1, x2, y2 = box1
        x1g, y1g, x2g, y2g = box2
        xi1, yi1, xi2, yi2 = max(x1, x1g), max(y1, y1g), min(x2, x2g), min(y2, y2g)
        inter_area = max(0, xi2 - xi1) * max(0, yi2 - yi1)
        box1_area = (x2 - x1) * (y2 - y1)
        box2_area = (x2g - x1g) * (y2g - y1g)
        return inter_area / float(box1_area + box2_area - inter_area + 1e-6)

    def update(self, detections):
        current_time = time.time()
        self.trackers = [t for t in self.trackers if (current_time - t.last_update) < self.max_age]
        updated_objects = []

        if len(detections) > 0:
            for detection in detections:
                best_tracker = None
                max_iou = 0

                for tracker in self.trackers:
                    iou_score = self.iou(tracker.predict(), detection)
                    if iou_score > max_iou:
                        max_iou = iou_score
                        best_tracker = tracker

                if best_tracker and max_iou > self.iou_threshold:
                    best_tracker.update(detection)
                    updated_objects.append(np.append(best_tracker.kf.x[:4].flatten(), best_tracker.id))
                else:
                    new_tracker = KalmanBoxTracker(detection)
                    self.trackers.append(new_tracker)
                    updated_objects.append(np.append(detection, new_tracker.id))

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

        # YOLO 탐지
        results = model(frame, imgsz=640)
        detections = []

        for result in results:
            boxes = result.boxes.data.tolist()
            # 필터링을 한 번에 수행
            detections = [[x1, y1, x2, y2] for x1, y1, x2, y2, conf, cls_id in boxes 
                         if conf >= DETECTION_CONFIDENCE and int(cls_id) == TARGET_CLASS]

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

        # --- Draw grid and show direction arrows ---
        h, w, _ = frame.shape
        draw_grid(frame)

        if tracked_objects is not None and len(tracked_objects) > 0:
            for obj in tracked_objects:
                try:
                    x1, y1, x2, y2, obj_id = map(int, obj)
                    cx, cy = (x1 + x2) // 2, (y1 + y2) // 2  # 중심 좌표

                    direction = get_direction(cx, cy, w, h)
                    print(f"Object {obj_id} 이동 방향: {direction}")

                    cv2.arrowedLine(frame, (cx, cy), (w//2, h//2), (0, 0, 255), 2, tipLength=0.3)
                    cv2.circle(frame, (cx, cy), 5, (255, 0, 0), -1)
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

# 중앙에 위치했을 때에는 안움직이는 게 아니라, 천천히 중앙으로 움직이면 좋겠음
