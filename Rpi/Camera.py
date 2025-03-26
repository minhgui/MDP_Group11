import base64
import json
import os
from typing import Dict, Any
from picamera import PiCamera
import cv2
import time
from datetime import datetime

FOLDER_PATH = "/home/Grp11/MDPfiles/Communication/Images"
IMAGE_PREPROCESSED_FOLDER_PATH = "/home/Grp11/MDPfiles/Images_Preprocessed"

def capture(path: str) -> None:
    camera = PiCamera()
    image_path = os.path.join(FOLDER_PATH, path)
    camera.capture(image_path)
    camera.close()
    print("[Camera] Image captured")

def preprocess_img(path: str) -> None:
    image_path = os.path.join(FOLDER_PATH, path)
    img = cv2.imread(image_path)
 
    resized_img = cv2.resize(img, (640, 640))
    image_path = os.path.join(IMAGE_PREPROCESSED_FOLDER_PATH, path)
    cv2.imwrite(image_path, resized_img)
    print("[Camera] Image preprocessing complete")

def get_image(final_image:bool=False) -> bytes:
    # Create a unique image path based on the current timestamp
    timestamp = datetime.fromtimestamp(time.time()).strftime('%d-%m_%H-%M-%S')
    path = f"img_{timestamp}.jpg"

    # Capture and preprocess the image
    capture(path)
    preprocess_img(path)

    # Construct a message with the encoded image
    encoded_string = ""
    image_path = os.path.join(IMAGE_PREPROCESSED_FOLDER_PATH, path)
    if os.path.isfile(image_path):
        with open(image_path, "rb") as img_file:
            encoded_string = base64.b64encode(img_file.read()).decode('utf-8')

    # Create a JSON message containing the image data
    message: Dict[str, Any] = {"type": 'IMAGE_TAKEN',
                                "final_image": final_image,
                                "data": {"image": encoded_string}}
    
    return json.dumps(message).encode("utf-8")
