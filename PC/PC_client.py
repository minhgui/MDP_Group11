import os
import socket
import threading
from queue import Queue
import json
import time
import shutil
import base64
from ultralytics import YOLO
from pathlib import Path
from datetime import datetime

from algo.pathfinding import task1
from image_recognition.stitch_images import stitching_images

class PCClient:
    def __init__(self):
        # Connection details
        self.host = "192.168.11.1" #Rpi IP address
        self.port = 8888
        self.client_socket = None
        self.msg_queue = Queue()
        self.send_message = False

        # Task utils
        self.t1 = task1.task1()
        self.image_record = []
        self.task_2 = True 
        self.obs_order_count = 0

        # Image Inferencing Model
        model_path = Path("image_recognition") / "best_task_2_ncnn_model"
        self.model = YOLO(model_path)
        self.conf = 0.850

#   =================================== Connection functions =================================

    def connect(self):
        while not self.send_message: 
            try:
                self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.client_socket.connect((self.host, self.port))
                self.send_message = True
                print("[PC Client] Connected to PC successfully.")
            except socket.error as e:
                print("[PC Client] ERROR: Failed to connect, retrying in 1 second.")
                time.sleep(1)

    def disconnect(self):
        # Disconnect from the PC
        try:
            if self.client_socket is not None:
                self.client_socket.close()
                self.send_message = False
                print("[PC Client] Disconnected from rpi.")
        except Exception as e:
            print("[PC Client] Failed to disconnect from rpi:", str(e))
    
    def reconnect(self):
        # Disconnect and then connect again
        print("[PC Client] Reconnecting...")
        self.send_message = False
        self.disconnect()
        self.connect()

#   =================================== Data functions =================================

    def send(self):
        while True:
            if self.send_message:
                message = self.msg_queue.get()
                exception = True
                while exception:
                    try:
                        self.client_socket.sendall(self.prepend_msg_size(message))
                        print("[PC Client] Write to RPI: first 200=", message[:200])
                    except Exception as e:
                        print("[PC Client] ERROR: Failed to write to RPI -", str(e))
                        self.reconnect()
                    else:
                        exception = False
            
    def prepend_msg_size(self, message):
        message_bytes = message.encode("utf-8")
        message_len = len(message_bytes)
        length_bytes = message_len.to_bytes(4, byteorder="big")
        return length_bytes + message_bytes

    def receive_all(self, size):
        data = b""
        while len(data) < size:
            chunk = self.client_socket.recv(size - len(data))
            if not chunk:
                raise ConnectionError("Connection closed unexpectedly")
            data += chunk
        return data
    
#   =================================== Task functions =================================

    def image_inference(self, image_or_path, obs_id, image_counter, task_2:bool=True):
        timestamp = datetime.fromtimestamp(time.time()).strftime('%d-%m_%H-%M-%S')
        img = f"img_{timestamp}"

        # Run inference
        try:
            result = self.model.predict(source=image_or_path, verbose=False, project="./captured_images", 
                                         name=f"{img}_1", save=True, save_txt=True, save_conf=True, 
                                         imgsz=640, conf=self.conf)
        except Exception as e:
            print(f"Error running inference: {e}")
            return None

        # Process results
        bboxes = []
        for r in result:
            for c in r:
                label = c.names[c.boxes.cls.tolist().pop()].split("_")[0]
                bboxes.append({"label": label, "xywh": c.boxes.xywh.tolist().pop()})

        #result[0].show()
        max_label, max_area = self.find_best_label(bboxes)
        if max_area:
            img = img + "_1"          

        img_name = f"task2_obs_id_{obs_id}_{image_counter}.jpg" if task_2 else f"task1_obs_id_{obs_id}_{image_counter}.jpg"

        image_pred = {
            "type": "IMAGE_RESULTS",
            "data": {"obs_id": obs_id, 
                     "img_id": max_label, 
                     "bbox_area": max_area},
            "image_path": str(Path("captured_images") / f"{img}" / img_name)
        }

        return image_pred

    def find_best_label(self, bboxes):
        max_area = 0.0
        max_label = None

        for bbox in bboxes: 
            label = bbox['label']
            _, _, width, height = bbox['xywh']

            # Ignore bullseye
            if label == "41":
                continue  

            bbox_area = width * height
            if bbox_area > max_area:
                max_area = bbox_area
                max_label = label

        return max_label, max_area

    def receive_messages(self):
        try:
            image_counter = 0
            obs_id = 0
            cmd = None
            while True:
                # Receive the length of the message
                length_bytes = self.receive_all(4)
                if not length_bytes:
                    print("[PC Client] PC Server disconnected.")
                    self.reconnect()
                message_length = int.from_bytes(length_bytes, byteorder="big")

                # Receive the actual message data
                message = self.receive_all(message_length)
                if not message:
                    print("[PC Client] PC Server disconnected remotely.")
                    self.reconnect()

                print("[PC Client] Received message: first 200:", message[:200])

                message = json.loads(message)
                
                # Task 1 start
                if message["type"] == "START_TASK":
                    self.t1.generate_path(message)
                    cmd = self.t1.get_command_to_next_obstacle() 
                    obs_id = str(self.t1.get_obstacle_id())
                    self.msg_queue.put(json.dumps(cmd))

                # Call image recognition
                elif message["type"] == "IMAGE_TAKEN":
                    encode_image = message["data"]["image"]
                    decode_image = base64.b64decode(encode_image)
                    os.makedirs("captured_images", exist_ok=True)

                    if self.task_2:
                        path = f"captured_images/task2_obs_id_{obs_id}_{image_counter}.jpg"
                    else:
                        path = f"captured_images/task1_obs_id_{obs_id}_{image_counter}.jpg"
                    
                    with open(path, "wb") as img_file:
                        img_file.write(decode_image)

                    image_pred = self.image_inference(image_or_path=path, obs_id=str(obs_id), 
                                                            image_counter=image_counter, task_2=self.task_2)
                    
                    # print(image_pred['data']['img_id']) # for testing log

                    if image_pred['data']['img_id'] != None:
                        print("[PC Client] Image detected with ID: ", image_pred['data']['img_id'])
                    self.image_record.append(image_pred)
                    image_counter += 1

                    if message["final_image"] == True:
                        
                        # Get last prediction and move forward
                        while image_pred['data']['img_id'] == None and self.image_record is not None:
                            if self.image_record:
                                image_pred = self.image_record.pop()
                            else:
                                break
                        
                        # For checklist A.5
                        # if (image_pred['data']['img_id'] == None) and (RETRIES > retries):
                        #     data_send = {"type": "NAVIGATION", 
                        #                  "data": {"commands": ['FL045', 'FS010', 'FR045', 'FS025', 'BL090', 'RESET']}, "path": []},

                        #     self.msg_queue.put(json.dumps(data_send))
                        #     print("No face found")
                        #     retries += 1
                        #     continue
                            
                        # else:
                        #     print("Find the non-bulleye ended")
                        #     return

                        destination_folder = "images_result"
                        os.makedirs(destination_folder, exist_ok=True)
                            
                        if self.task_2:
                            destination_file = f"{destination_folder}/task2_result_obs_id_{obs_id}.jpg"
                        else:
                            destination_file = f"{destination_folder}/task1_result_obs_id_{obs_id}.jpg"
                        path = image_pred["image_path"] 
                        
                        if (image_pred['data']['img_id'] != None):
                            shutil.copy([path], destination_file)

                        message = json.dumps(image_pred)
                        self.msg_queue.put(message)
                        self.t1.update_image_id(image_pred['data']['img_id'])
                        image_counter = 0
                        if self.task_2:
                            obs_id += 1 

                        # For task 1
                        # if not self.t1.has_task_ended():
                        #     cmd = self.t1.get_command_to_next_obstacle()
                        #     self.msg_queue.put(json.dumps(cmd))
                        #     obs_id = str(self.t1.get_obstacle_id())
                        #     print("ID: ", obs_id)
                        # else:
                        #     print("[Algo] Task 1 ended")
                        #     stitching_images(r'images_result', r'image_recognition\stitched_image.jpg')
                        #     break # exit thread

                        # For task 2 
                        if obs_id > 1: # only take 2 pictures
                            print("Task 2 ended")
                            stitching_images(r'images_result', r'image_recognition\stitched_image.jpg')
                            break 

                        self.image_record = [] # reset the image record

        except socket.error as e:
            print("[PC Client] ERROR:", str(e))
    

if __name__ == "__main__":
    
    client = PCClient()
    client.connect()
    
    PC_receive = threading.Thread(target=client.receive_messages, name="PC_receive_thread")
    PC_send = threading.Thread(target=client.send, name="PC_send_thread")
    PC_receive.start()
    PC_send.start()
    print("[PC Client] Threads successfully started")

    PC_receive.join()
    PC_send.join()
    print("[PC Client] All threads concluded")

    client.disconnect()
