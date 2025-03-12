from queue import Queue
import bluetooth as bt
import socket
import sys
import subprocess
import json
from rpi_config import *

class AndroidInterface:
    """
    Represents the interface between the Raspberry Pi and an Android device over Bluetooth.

    Args:
    - RPiMain: Instance of the RPiMain class.

    Attributes:
    - RPiMain (RPiMain): Instance of the RPiMain class.
    - host (str): IP address of the Raspberry Pi.
    - uuid (str): Bluetooth UUID for the service.
    - msg_queue (Queue): Queue for storing messages.
    - socket (BluetoothSocket): Bluetooth socket for communication.
    - port (int): Port number for the socket connection.
    - client_socket (BluetoothSocket): Socket for communication with the connected Android device.
    - client_info (tuple): Information about the connected Android client.
    """
    def __init__(self, RPiMain):
        # Initialize AndroidInterface with RPiMain instance
        self.RPiMain = RPiMain
        self.host = RPI_IP
        self.uuid = BT_UUID
        self.msg_queue = Queue()
        self.obstacle_list = [] #NEW ITEM
        self.obstacle_data = {} #NEW ITEM
        self.robot_data = {} #NEW ITEM
        self.data = {"type": "START_TASK", "data": {"task": "EXPLORATION", "robot": {},
                                               "obstacles": []}}

    def connect(self):
        # Grant permission for Bluetooth access
        subprocess.run("sudo chmod o+rw /var/run/sdp", shell=True) 

        # Establish and bind socket
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
            
            # Advertise Bluetooth service
            bt.advertise_service(self.socket, "Group11-Server", service_id=self.uuid, service_classes=[self.uuid, bt.SERIAL_PORT_CLASS], profiles=[bt.SERIAL_PORT_PROFILE])

        except socket.error as e:
            print("[Android] ERROR: Android socket binding failed -", str(e))
            sys.exit()
            
        print("[Android] Waiting for Android connection...")

        try:
            self.client_socket, self.client_info = self.socket.accept()
            print("[Android] Accepted connection from", self.client_info)
            
        except socket.error as e:
            print("[Android] ERROR: connection failed -", str(e))

    def disconnect(self):
        # Close the Bluetooth socket
        try:
            self.socket.close()
            print("[Android] Disconnected from Android successfully.")
        except Exception as e:
            print("[Android] ERROR: Failed to disconnect from Android -", str(e))
            
    def reconnect(self):
        # Disconnect and then connect again
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

                #parsedMsg = json.loads(decodedMsg)
                #msg_type = parsedMsg["type"]

                #if msg_type == "NAVIGATION":
                    #self.RPiMain.STM.msg_queue.put(message)

                #elif msg_type == "START_TASK" or msg_type == "FASTEST_PATH":
                    #self.RPiMain.PC.msg_queue.put(message)
                # Route messages to the appropriate destination
                if decodedMsg[0] == 'F' or decodedMsg[0] == 'B':
                    if decodedMsg[1] == 'S':
                        self.RPiMain.stm.msg_queue.put(decodedMsg + '010')
                    elif decodedMsg[1] == 'L':
                        self.RPiMain.stm.msg_queue.put(decodedMsg + '090')
                    else: 
                        self.RPiMain.stm.msg_queue.put(decodedMsg + '090')
                elif 'Z' in decodedMsg:
                    if decodedMsg[0] == 'O':
                        msg = decodedMsg.split(',')
                        new_obstacle = {
                            "id": msg[1],
                            "x": int(msg[2]),
                            "y": int(msg[3]),
                            "dir": msg[4][0]}

                        print("Added obstacle ", msg[1])
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
                        print("Added robot coords")
                        print(self.data)
                        json_path_message = json.dumps(self.data)
                        encode_path_message = json_path_message.encode("utf-8")
                        self.RPiMain.PC.msg_queue.put(encode_path_message)
                else:
                    print("Unknown message: ", decodedMsg)

            except (socket.error, IOError, Exception, ConnectionResetError) as e:
                print("[Android] ERROR:", str(e))
    
    def send2(self):
        message = "STATUS: 123"
        self.client_socket.sendall(message)
        print("[Android] Write to Android: " + message)                

    def send(self):
        # Continuously send messages to Android
        while True: 
            # Test code start
#             message_ori =   {
#                 "type": "IMAGE_RESULTS",
               # "data": {
                #"obs_id": "11", 
               # "img_id": "30", 
               # }
            #}
            # message = {
            #     "type": "NAVIGATION",
            #     "data": {
            #     "commands":  ["LF045", "RF045", "SF040", "RF045", "LF045"],
            #     # "commands": ["UF150"],

            #     "path": [[0,1], [1,1], [2,1], [3,1], [3,3]]
            #     }
            # }
            # message_ori = "hello from rpi"
#             message = json.dumps(message).encode("utf-8")
            # test code end

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
                    #self.client_socket.sendall(message)
                    print("Image id received")
            elif message_type == "NAVIGATION":
                message = message_str
            else:
                message = "Incorrect message type received"

            exception = True
            while exception: 
                try:
                    self.client_socket.sendall(message)
                    print("[Android] Write to Android: " + message[:MSG_LOG_MAX_SIZE])
                except Exception as e:
                    print("[Android] ERROR: Failed to write to Android -", str(e))
                    self.reconnect()  # reconnect and resend
                else:
                    exception = False  # done sending, get next message

