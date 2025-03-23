
import json
from queue import Queue
import re
import threading
import time
import serial
from Camera import get_image
from rpi_config import *
import time

class STMInterface:
    def __init__(self, RPiMain, task2):
        self.RPiMain = RPiMain 
        self.baudrate = STM_BAUDRATE
        self.serial = None
        self.msg_queue = Queue()
        self.task2 = task2

    def connect(self):
        # Connect to STM using available serial ports
        try:
            self.serial = serial.Serial("/dev/ttyUSB0", self.baudrate, write_timeout=0)
            print("[STM] Connected to STM 0 successfully.")
            self.clean_buffers()
        except:
            try:
                self.serial = serial.Serial("/dev/ttyUSB1", self.baudrate, write_timeout=0)
                print("[STM] Connected to STM 1 successfully.")
                self.clean_buffers()
            except Exception as e:
                print("[STM] ERROR: Failed to connect to STM -", str(e))
    
    def reconnect(self): 
        # Reconnect to STM by closing the current connection and establishing a new one
        if self.serial is not None and self.serial.is_open:
            self.serial.close()
        self.connect()
    
    def clean_buffers(self):
        # Reset input and output buffers of the serial connection
        self.serial.reset_input_buffer()
        self.serial.reset_output_buffer()

    def listen(self):
        # Listen for messages from STM
        message = None
        while True:
            try:
                message = self.serial.read().decode("utf-8")
                print("[STM] Read from STM:", message[:MSG_LOG_MAX_SIZE])
                
                if len(message) < 1:
                    continue
                else: 
                    break

            except Exception as e:
                message = str(e)
                break

        return message

    # Testing for task 2 without Android
    def send2(self):
        commands = ["OBS1U"]
        for command in commands:
            print("[RPI] Writing to STM:", command)
            self.write_to_stm(command)
        capture_and_send_image_thread = threading.Thread(target=self.send_image_to_pc(final_image=True), daemon=True)   
        capture_and_send_image_thread.start()
            
    def send(self):
        # Send commands to STM 
        while True: 
            # Test commands
            # commands = ["RESET", "FS010", "FR090", "BS010", "FL090"]
    
            # for command in commands:
            #         print("[RPI] Writing to STM:", command)
            #         self.write_to_stm(command)
            
            message_byte = self.msg_queue.get()
            message_str = message_byte.decode("utf-8")

            # Movement commands from Android 
            if message_str[0] == 'F' or message_str[0] == 'B':
                self.write_to_stm(message_str)
            
            # Movement commands from PC 
            elif message_str[0] == '{':
                message = json.loads(message_str)
                message_type = message["type"]
            
                if message_type == "NAVIGATION":
                    # Send commands to stm
                    for idx, command in enumerate(message["data"]["commands"]):
                        print("[RPI] Writing to STM:", command)
                        self.write_to_stm(command)
                    
                    # Start a new thread to capture and send the image to PC
                    capture_and_send_image_thread = threading.Thread(target=self.send_image_to_pc(final_image=True), daemon=True)
                    capture_and_send_image_thread.start()
                    
            else:
                print("[STM] Rejecting message with unknown type [%s] for STM" % message_type)

    def write_to_stm(self, command):
        self.clean_buffers()
        exception = True
        while exception:
            try:
                print("[STM] Sending command", command)
                encoded_string = command.encode()
                byte_array = bytearray(encoded_string)
                self.serial.write(byte_array)

            except Exception as e:
                print("[STM] ERROR: Failed to write to STM -", str(e)) 
                exception = True
                self.reconnect() 

            else:
                exception = False
                if command == STM_GYRO_RESET_COMMAND:
                    print("[STM] Waiting %ss for reset" % STM_GYRO_RESET_DELAY)
                    time.sleep(STM_GYRO_RESET_DELAY)
                else:
                    print("[STM] Waiting for ACK")
                    self.wait_for_ack()

    def wait_for_ack(self):
        # Waiting for ACK from STM
        message = self.listen()
        if message  == STM_ACK_MSG:
            print("[STM] Received ACK from STM") 
        else:
            print("[STM] ERROR: Unexpected message from STM -", message)
            self.reconnect() 

    def send_image_to_pc(self, final_image:bool):
        self.RPiMain.PC.msg_queue.put(get_image(final_image=final_image))      

