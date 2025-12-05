import cv2
import time
from ultralytics import YOLO
import torch
#from ultralyticsplus import YOLO, render_result

# load model
#model = YOLO('foduucom/plant-leaf-detection-and-classification')
# Optional: Initialize serial connection
import serial
ser = serial.Serial('COM6', 9600, timeout=1)
time.sleep(2)
conf=0.0 
# Load YOLOv8 model
#model = YOLO("diffplnt.pt")
model = YOLO("model1.pt")
# Check if GPU is available
device = "cuda" if torch.cuda.is_available() else "cpu"
model.to(device)
print(f"Running on {device}")

# Stepper parameters
x_position = 0
y_position = 0


step_size = 1
step_size1 = 1
min_x, max_x = -500 , 500
min_y, max_y = -100 , 100

# Camera properties
frame_width, frame_height = 640, 480
center_x, center_y = frame_width // 2, frame_height // 2
tolerance = 80

cap = cv2.VideoCapture(1)
cap.set(3, frame_width)
cap.set(4, frame_height)

def send_stepper_command(x, y):
    x = round(x, 2)
    y = round(y, 2)
    gcode_command = f'G1 X{x} F10\n'
    ser.write(gcode_command.encode())
    print(f"Sent: {gcode_command}")
   
    gcode_command = f'G1 Y{y} F10\n'
    ser.write(gcode_command.encode())
    print(f"Sent: {gcode_command}")


    
    
while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break
    
    # Run inference using YOLOv8
    results = model(frame)[0]  # Ultralytics returns list, so [0]
    if results.boxes is not None and len(results.boxes) > 0 :
       box = results.boxes[0]
       conf = float(box.conf[0])
    else:
       conf=0
    print(conf)
    if results.boxes is not None and len(results.boxes) > 0 and conf > 0.7:
        box = results.boxes[0]
        x1, y1, x2, y2 = map(int, box.xyxy[0])
        conf = float(box.conf[0])
        cls_id = int(box.cls[0])

        # Draw bounding box
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(frame, f"{model.names[cls_id]} {conf:.2f}", (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # Calculate center of detected object
        face_center_x = (x1 + x2) // 2
        face_center_y = (y1 + y2) // 2
        
        # Adjust stepper position
        if face_center_x < center_x - tolerance:
            x_position += step_size1
        elif face_center_x > center_x + tolerance:
            x_position -= step_size1

        if face_center_y < center_y - tolerance:
            y_position -= step_size
        elif face_center_y > center_y + tolerance:
            y_position += step_size

        # Constrain movement
        x_position = max(min_x, min(max_x, x_position))
        y_position = max(min_y, min(max_y, y_position))
        print("a"+str(y_position))
        # Send G-code to steppers
        send_stepper_command(y_position, x_position)
        time.sleep(300/1000)

    cv2.imshow("YOLOv8 Object Tracking", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Cleanup
cap.release()
cv2.destroyAllWindows()
# ser.close()
