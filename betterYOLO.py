import cv2
import numpy as np
import torch
from ultralytics import YOLO
import time
from filterpy.kalman import KalmanFilter


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
TARGET_CLASS = 67  # Cell Phone

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        # YOLO 탐지
        results = model(frame, imgsz=640)
        detections = []

        for result in results:
            for box in result.boxes.data.tolist():
                x1, y1, x2, y2, conf, cls_id = box
                if conf >= 0.5 and int(cls_id) == TARGET_CLASS:
                    detections.append([x1, y1, x2, y2])

        print(f"Detections: {detections}")  # 디버깅 로그 추가

        # SORT로 트래킹 업데이트
        tracked_objects = tracker.update(np.array(detections) if detections else np.empty((0, 4)))

        print(f"Tracked Objects: {tracked_objects}")  # 디버깅 로그 추가

        # 트래킹된 휴대폰 표시
        if tracked_objects is not None and len(tracked_objects) > 0:
            for obj in tracked_objects:
                try:
                    x1, y1, x2, y2, obj_id = map(int, obj)
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(frame, f"Phone ID: {obj_id}", (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                except ValueError as e:
                    print(f"ValueError: {e}, Data: {obj}")  # 값 변환 오류 확인용

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
