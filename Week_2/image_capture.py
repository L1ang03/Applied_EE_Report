import cv2
import os
from picamera2 import Picamera2
import time


symbol_name = "triangle"

def create_folder(name):
    dataset_folder = "/home/pi/Desktop/triangle"
    if not os.path.exists(dataset_folder):
        os.makedirs(dataset_folder)
    
    class_folder = os.path.join(dataset_folder, name)
    if not os.path.exists(class_folder):
        os.makedirs(class_folder)
    return class_folder

def capture_photos(name):
    folder = create_folder(name)
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(main={"format": "XRGB8888", "size": (640, 480)})
    picam2.configure(config)
    picam2.start()
    time.sleep(2)  
    photo_count = 0
    print(f"Taking photos for '{name}'. Press SPACE to capture, 'q' to quit.")
    while True:
        frame = picam2.capture_array()
        cv2.imshow("Capture Window", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord(' '):
            photo_count += 1
            filename = f"{name}{photo_count}.jpg"
            filepath = os.path.join(folder, filename)
            cv2.imwrite(filepath, frame)
            print(f"Photo {photo_count} saved: {filepath}")
        elif key == ord('q'):
            print("Quitting capture...")
            break

    picam2.stop()
    cv2.destroyAllWindows()
    print(f"Done! {photo_count} photos saved for '{name}'.")

if __name__ == "__main__":
    capture_photos(symbol_name)
