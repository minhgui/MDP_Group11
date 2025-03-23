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

def capture(img_pth: str) -> None:
    camera = PiCamera()
    print(img_pth)
    image_save_location = os.path.join(FOLDER_PATH, img_pth)
    camera.capture(image_save_location)
    camera.close()
    print("[Camera] Image captured")

def preprocess_img(img_pth: str) -> None:
    image_save_location = os.path.join(FOLDER_PATH, img_pth)
    img = cv2.imread(image_save_location)

    # Resize image to 640x640 for model 
    resized_img = cv2.resize(img, (640, 640))
    image_save_location = os.path.join(IMAGE_PREPROCESSED_FOLDER_PATH, img_pth)
    cv2.imwrite(image_save_location, resized_img)
    print("[Camera] Image preprocessing complete")

def get_image(final_image:bool=False) -> bytes:
    # Create a unique image path based on the current timestamp
    formatted_time = datetime.fromtimestamp(time.time()).strftime('%d-%m_%H-%M-%S')
    img_pth = f"img_{formatted_time}.jpg"

    # Capture and preprocess the image
    capture(img_pth)
    preprocess_img(img_pth)

    # Construct a message with the encoded image
    encoded_string = ""
    image_save_location = os.path.join(IMAGE_PREPROCESSED_FOLDER_PATH, img_pth)
    if os.path.isfile(image_save_location):
        with open(image_save_location, "rb") as img_file:
            encoded_string = base64.b64encode(img_file.read()).decode('utf-8')

    # Create a JSON message containing the image data
    message: Dict[str, Any] = {
        "type": 'IMAGE_TAKEN',
        "final_image": final_image,
        "data": {"image": encoded_string}
    }
    
    return json.dumps(message).encode("utf-8")
