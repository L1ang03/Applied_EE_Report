
import cv2
import numpy as np
from picamera2 import Picamera2
import tflite_runtime.interpreter as tflite

# === Load the model ===
interpreter = tflite.Interpreter(model_path="/home/pi/Desktop/week3.tflite")
interpreter.allocate_tensors()

# === Get input/output tensor details ===
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

# === Load class labels ===
with open("/home/pi/Desktop/week3.txt", "r") as f:
    class_names = [line.strip() for line in f.readlines()]

# === Initialize Pi Camera ===
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"format": "RGB888", "size": (224, 224)}))
picam2.start()

print("âœ… Running detection... Press 'q' to exit.")

while True:
    # 1. Capture frame
    frame = picam2.capture_array()

    # 2. Resize and normalize for model input
    input_frame = cv2.resize(frame, (224, 224))
    input_frame = input_frame.astype('float32') / 255.0  # Normalize
    input_frame = np.expand_dims(input_frame, axis=0)    # Add batch dimension

    # 3. Run inference
    interpreter.set_tensor(input_details[0]['index'], input_frame)
    interpreter.invoke()
    output_data = interpreter.get_tensor(output_details[0]['index'])
    prediction = np.argmax(output_data)
    confidence = output_data[0][prediction]

    label = f"{class_names[prediction].upper()} ({confidence*100:.1f}%)"

    # 4. Display result
    cv2.putText(frame, label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
    cv2.imshow("Teachable Machine Detection", frame)

    # 5. Exit on 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
picam2.stop()
