from queue import Queue
import bluetooth as bt
import socket
import sys
import subprocess
import json
from rpi_config import * 
from stm import *

class AndroidInterface:
    def __init__(self, RPiMain):
        # Initialize AndroidInterface with RPiMain instance
        self.RPiMain = RPiMain
        self.host = RPI_IP
        self.uuid = BT_UUID
        self.msg_queue = Queue()
        self.obstacle_list = [] # List of obstacles to be sent to PC
        self.obstacle_data = {} # Object coordinates and image direction
        self.robot_data = {} # Robot coordinates and direction
        self.data = {"type": "START_TASK", 
                     "data": {"task": "EXPLORATION", "robot": {},
                              "obstacles": []}} # Full object to be sent to PC for task 1

    def connect(self):
        subprocess.run("sudo chmod o+rw /var/run/sdp", shell=True) 
        self.socket = bt.BluetoothSocket(bt.RFCOMM)
        print("[Android] BT socket established successfully.")
    
        try:
            self.port = self.socket.getsockname()[1]
            print("[Android] Waiting for connection on RFCOMM channel", self.port)
            self.socket.bind((self.host, bt.PORT_ANY))
            print("[Android] BT socket binded successfully.")
            
            # Turning advertisable
            subprocess.run("sudo hciconfig hci0 piscan", shell=True)
            self.socket.listen(128)
            bt.advertise_service(self.socket, "Group11-Server", service_id=self.uuid, service_classes=[self.uuid, bt.SERIAL_PORT_CLASS], profiles=[bt.SERIAL_PORT_PROFILE])

        except socket.error as e:
            print("[Android] Android socket binding failed -", str(e))
            sys.exit()
            
        print("[Android] Waiting for Android connection...")

        try:
            self.client_socket, self.client_info = self.socket.accept()
            print("[Android] Accepted connection from", self.client_info)
            
        except socket.error as e:
            print("[Android] ERROR: connection failed -", str(e))

    def disconnect(self):
        # Close Bluetooth socket
        try:
            self.socket.close()
            print("[Android] Disconnected from Android successfully.")
        except Exception as e:
            print("[Android] Failed to disconnect from Android -", str(e))
            
    def reconnect(self):
        self.disconnect()
        self.connect()

    def listen(self):
        # Continuously listen for messages from Android
        while True:
            try:
                message = self.client_socket.recv(BT_BUFFER_SIZE) 

                if not message:
                    print("[Android] Android disconnected remotely. Reconnecting...")
                    self.reconnect()

                decodedMsg = message.decode("utf-8")
                if len(decodedMsg) <= 1:
                    continue

                print("[Android] Read from Android:", decodedMsg[:MSG_LOG_MAX_SIZE])

                # Route messages (receives string from android)
                if decodedMsg[0] == 'F' or decodedMsg[0] == 'B':
                    if decodedMsg[1] == 'E':
                        path_message = {"type": "NAVIGATION", "data": {"commands": ["OBS1U"], "path": []}}
                        json_path_message = json.dumps(path_message)
                        encode_path_message = json_path_message.encode("utf-8")
                        self.RPiMain.STM.msg_queue.put(encode_path_message)
                    elif decodedMsg[1] == 'S':
                        self.RPiMain.STM.msg_queue.put(decodedMsg + '010')
                    elif decodedMsg[1] == 'L':
                        self.RPiMain.STM.msg_queue.put(decodedMsg + '090')
                    else: #'R'
                        self.RPiMain.STM.msg_queue.put(decodedMsg + '090')
                elif 'Z' in decodedMsg:
                    # Ensures only send during obstacle map loading not obstacle generation
                    if decodedMsg[0] == 'O': 
                        msg = decodedMsg.split(',')
                        new_obstacle = {
                            "id": msg[1],
                            "x": int(msg[2]),
                            "y": int(msg[3]),
                            "dir": msg[4][0]}

                        print("[Android] Added obstacle ", msg[1])
                        self.obstacle_list.append(new_obstacle)
                        print(self.obstacle_list)                       

                    elif decodedMsg[0] == 'R':
                        msg = decodedMsg.split(',')
                        self.robot_data["id"] = "R"
                        self.robot_data["x"]=int(msg[1])
                        self.robot_data["y"]=int(msg[2])
                        self.robot_data["dir"] = msg[3][0]
                        self.data["data"]["obstacles"] = self.obstacle_list
                        self.data["data"]["robot"] = self.robot_data
                        print("[Android] Added robot coords")
                        print(self.data)
                        json_path_message = json.dumps(self.data)
                        encode_path_message = json_path_message.encode("utf-8")
                        self.RPiMain.PC.msg_queue.put(encode_path_message)
                else:
                    print("Unknown message: ", decodedMsg)

            except (socket.error, IOError, Exception, ConnectionResetError) as e:
                print("[Android] Error:", str(e))
    
    # Utility for Android checklist
    def send2(self):
        message = "STATUS: 123"
        self.client_socket.sendall(message)
        print("[Android] Write to Android: " + message)                

    def send(self):
        # Continuously send messages to Android
        while True: 
            message_byte = self.msg_queue.get()
            message_str = message_byte.decode("utf-8")
            message_json = json.loads(message_str)
            message_type = message_json["type"]
            print("[Android] Got message type: ", message_type)

            if message_type == "IMAGE_RESULTS" or message_type == "IMAGE_TAKEN":
                if message_json["data"]["img_id"] == None:
                    message = "No image found\n"
                else: 
                    message = "TARGET,%s,%s" % (message_json["data"]["obs_id"], message_json["data"]["img_id"])
                    self.client_socket.sendall(message) 
                    print("Image id received")
            elif message_type == "NAVIGATION":
                message = message_str
            else:
                message = "Unknown message type received" # Error handling due to many json object types

            exception = True
            while exception: 
                try:
                    if message != "Incorrect message type received":
                        self.client_socket.sendall(message)
                        print("[Android] Write to Android: " + message[:MSG_LOG_MAX_SIZE])
                except Exception as e:
                    print("[Android] Failed to write to Android -", str(e))
                    self.reconnect()  # reconnect and resend
                else:
                    exception = False  # done sending, get next message

