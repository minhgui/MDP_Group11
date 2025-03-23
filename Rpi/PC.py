from queue import Queue
import socket
import json
from rpi_config import *

class PCInterface:
    def __init__(self, RPiMain, task2):
        # Initialize PCInterface with RPiMain instance and connection details
        self.RPiMain = RPiMain
        self.host = RPI_IP
        self.port = PC_PORT
        self.client_socket = None
        self.msg_queue = Queue()
        self.send_message = False
        self.obs_id = 1
        self.task2 = task2
        self.check = True
        

    def connect(self):
        # Establish a connection with the PC
        if self.client_socket is not None:
            self.disconnect()

        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) 
                print("[PC] Socket established successfully.")
                sock.bind((self.host, self.port))
                sock.listen(128)
                print("[PC] Waiting for PC connection...")
                self.client_socket, self.address = sock.accept() 
                self.send_message = True
        except socket.error as e:
            print("[PC] ERROR: Failed to connect -", str(e))
        else:
            print("[PC] PC connected successfully:", self.address)

    def disconnect(self):
        # Disconnect from the PC
        try:
            if self.client_socket is not None:
                self.client_socket.close()
                self.client_socket = None
                self.send_message = False
                print("[PC] Disconnected from PC successfully.")
        except Exception as e:
            print("[PC] Failed to disconnect from PC:", str(e))

    def reconnect(self):
        # Disconnect and then connect again
        self.disconnect()
        self.connect()

    def listen(self):
        # Continuously listen for messages from the PC
        while self.check:
            try:
                if self.client_socket is not None:
                    # # Receive the length of the message
                    length_bytes = self.client_socket.recv(4)
                    if not length_bytes:
                        print("[PC] Client disconnected.")
                        break
                    message_length = int.from_bytes(length_bytes, byteorder="big")
                    
                    # Receive the message
                    message = self.client_socket.recv(message_length)
                    if not message:
                        self.send_message = False
                        print("[PC] PC disconnected remotely. Reconnecting...")
                        self.reconnect()

                    decoded_msg = message.decode("utf-8")
                    if len(decoded_msg) <= 1:
                        continue

                    print("[PC] Read from PC:", decoded_msg[:MSG_LOG_MAX_SIZE])
                    parsed_msg = json.loads(decoded_msg)
                    msg_type = parsed_msg["type"]

                    # Route messages to the appropriate destination
                    if msg_type == 'NAVIGATION':
                        # Send commands to STM 
                        print("[RPI] Sending message to STM: ", parsed_msg["data"]["commands"])
                        self.RPiMain.STM.msg_queue.put(message)
                        # Send path to Android
                        print("[RPI] Sending message to Android: ", parsed_msg["data"]["path"])
                        self.RPiMain.Android.msg_queue.put(message)

                    elif msg_type == 'IMAGE_RESULTS':
                        # Send image to Android (commented out for task 2)
                        encode_message = decoded_msg.encode("utf-8")
                        self.RPiMain.Android.msg_queue.put(encode_message) 
                        print("[RPI]: Send to Android: ", encode_message) 

                        # Task 2 logic
                        if self.task2:
                            if self.obs_id == 1:
                                if parsed_msg["data"]["img_id"] == "39": 
                                    path_message = {"type": "NAVIGATION", "data": {"commands": ["OBS1L", "OBS2U"], "path": []}}
                                else:
                                    path_message = {"type": "NAVIGATION", "data": {"commands": ["OBS1R", "OBS2U"], "path": []}}
                                self.obs_id += 1
                            else:
                                if parsed_msg["data"]["img_id"] == "39": 
                                    path_message = {"type": "NAVIGATION", "data": {"commands": ["OBS2L"], "path": []}}
                                    self.check = False # Stop sending commands (ensure commands do not loop)
                                else:
                                    path_message = {"type": "NAVIGATION", "data": {"commands": ["OBS2R"], "path": []}}
                                    self.check = False # Stop sending commands (ensure commands do not loop)

                            json_path_message = json.dumps(path_message)
                            encode_path_message = json_path_message.encode("utf-8")
                            self.RPiMain.STM.msg_queue.put(encode_path_message)

                    # Start task 2
                    elif msg_type == "FASTEST_PATH":
                        path_message = {"type": "NAVIGATION", "data": {"commands": ["OBS1U"], "path": []}}
                        json_path_message = json.dumps(path_message)
                        encode_path_message = json_path_message.encode("utf-8")
                        self.RPiMain.STM.msg_queue.put(encode_path_message)

                    else:
                        print("[PC] ERROR: Received message with unknown type from PC -", message)
                else:
                    print("[PC Client] ERROR: Client socket is not initialized.")
                    return None

            except (socket.error, IOError, Exception, ConnectionResetError) as e:
                print("[PC] ERROR:", str(e))

    def send(self):
        # Continuously send messages to the PC Client
        while True:
            if self.send_message:
                message = self.msg_queue.get()
                message = message.decode("utf-8")
                exception = True

                while exception:
                    try:
                        message = self.prepend_msg_size(message)
                        self.client_socket.sendall(message)
                        print("[PC] Write to PC: first 100=", message[:100])
                    except Exception as e:
                        print("[PC] ERROR: Failed to write to PC -", str(e))
                        self.reconnect()
                    else:
                        exception = False

    def prepend_msg_size(self, message):
        message_bytes = message.encode("utf-8")
        message_len = len(message_bytes)
        
        length_bytes = message_len.to_bytes(4, byteorder="big")
        return length_bytes + message_bytes
    
