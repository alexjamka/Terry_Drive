import cv2
from ultralytics import YOLO

# 1. Load the model (Nano version is best for Jetson)
model = YOLO('yolov8n.pt')

# 2. Open the camera (this 'occupies' the device)
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Could not open camera. Is another script still running?")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # 3. Run YOLO on the 'occupied' camera stream
    # We use persist=True to help track objects smoothly
    results = model(frame, stream=True)

    # 4. Draw the boxes onto the frame automatically
    for r in results:
        annotated_frame = r.plot()

    # 5. Show the window
    cv2.imshow("YOLOv8 Live Detection", annotated_frame)

    # Press 'q' to stop everything and 'free' the camera
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
