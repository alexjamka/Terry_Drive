from ultralytics import YOLO
import cv2
import socket
import json

model = YOLO("yolov8n.engine")

cap = cv2.VideoCapture(0)

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
PI_IP = "10.0.0.1"
PI_PORT = 5005

while True:
    ret, frame = cap.read()
    if not ret:
        break

    results = model(frame)

    detections = []
    for r in results:
        for box in r.boxes:
            detections.append({
                "class": model.names[int(box.cls[0])],
                "confidence": round(float(box.conf[0]), 2),
                "x": int(box.xyxy[0][0]),
                "y": int(box.xyxy[0][1]),
                "w": int(box.xyxy[0][2] - box.xyxy[0][0]),
                "h": int(box.xyxy[0][3] - box.xyxy[0][1])
            })

    sock.sendto(json.dumps(detections).encode(), (PI_IP, PI_PORT))
    
    if detections:
        for d in detections:
            print(f"{d['class']} ({d['confidence']})")
    else:
        print("No detections")
