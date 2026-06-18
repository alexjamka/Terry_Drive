import cv2
from ultralytics import YOLO

# 1. Load your model (yolov8n.pt is the fastest for Jetson Nano)
# If you have a custom model, change 'yolov8n.pt' to your 'best.pt' path
model = YOLO('yolov8n.pt')

# 2. Open the webcam (0 is usually the default USB cam)
cap = cv2.VideoCapture(1)

if not cap.isOpened():
    print("Error: Could not open webcam.")
    exit()

print("Streaming... Press 'q' to quit.")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # 3. Run YOLOv8 inference on the frame
    # stream=True is more memory efficient for live video
    results = model(frame, stream=True)

    # 4. Visualize the results on the frame
    for r in results:
        annotated_frame = r.plot()

    # 5. Display the feed
    cv2.imshow("YOLOv8 Jetson Nano Feed", annotated_frame)

    # Break loop on 'q' key press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
