# Configuration constants
RPI_IP = "192.168.11.1"
MSG_LOG_MAX_SIZE = 150 

# PC Interface
PC_PORT = 8888
PC_BUFFER_SIZE = 2048

# Camera Interface
NUM_IMAGES = 1

# Android Interface
BT_UUID = "5bad6310-ff24-11ef-ac77-0800200c9a66"
BT_BUFFER_SIZE = 2048

# STM Interface
STM_BAUDRATE = 115200
STM_ACK_MSG = "ACK"
STM_NAV_COMMAND_FORMAT = '^[FB][[SLR][0-9]{3}$' # task 1
STM_NAV_COMMAND_FORMAT = '^OBS[1-2][ULR]$' # task 2
STM_GYRO_RESET_COMMAND = "RESET"
STM_GYRO_RESET_DELAY = 5 # time to wait for gyro reset

