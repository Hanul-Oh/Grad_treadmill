from ultralytics import YOLO
import cv2
import numpy as np

# 모델 불러오기
model = YOLO('best.pt')  # 학습된 커스텀 모델 로드

# 웹캠 열기
cap = cv2.VideoCapture(0)

while True:
    # 프레임 읽기
    ret, frame = cap.read()
    if not ret:
        break

    # 예측 수행
    results = model(frame)

    # 결과 시각화
    for r in results:
        boxes = r.boxes
        for box in boxes:
            # 박스 좌표
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

            # 신뢰도
            conf = float(box.conf)

            # 클래스 이름
            cls = int(box.cls)
            label = model.names[cls]

            # 박스 그리기
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            # 레이블과 정확도 표시
            cv2.putText(frame, f'{label} {conf:.2f}', (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

    # 결과 화면 표시
    cv2.imshow('ASL Detection', frame)

    # q를 누르면 종료
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 자원 해제
cap.release()
cv2.destroyAllWindows()