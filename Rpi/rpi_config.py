# Configuration constants
# LOCATION = "OUT" # IN (indoors) / OUT (outdoors) / NONE (disable turn adjustment)
LOCATION = "NONE"

RPI_IP = "192.168.11.1"
MSG_LOG_MAX_SIZE = 150 # characters

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
STM_NAV_COMMAND_FORMAT = '^(([FB][SLR])|([UYV]F)|([IXT][LR]))[0-9]{3}$' # task 2
STM_GYRO_RESET_COMMAND = "RESET"
STM_GYRO_RESET_DELAY = 2 # time to wait for gyro reset
STM_GYRO_RESET_FREQ = 3 # number of obstacles before GRYO RESET command is sent

# Task 1: adjust commands for turns to correct turning radius to 30cm, as expected by PC-algo
STM_COMMAND_ADJUSTMENT_DICT = {
    "OUT": {
        # 90 degree turns: manually calibrated
        "FR090": ["FS007", "FR090", "BS008"],
        "FL090": ["FS007", "FL090", "BS011"],
        "BR090": ["FS009", "BR090", "BS009"],
        "BL090": ["FS009", "BL090", "BS006"],
        # 180 degree turns: manually calibrated
        "FR180": ["FS008", "FR180", "BS008"],
        "FL180": ["FS007", "FL090", "BS004", "FL090", "BS011"],
        "BR180": ["FS009", "BR180", "BS009"],
        "BL180": ["FS009", "BL090", "FS002", "BL090", "BS009"],
        # 270 degree turns: approximated using 180 degree turn + 90 degree turn
        "FR270": ["FS008", "FR270", "BS008"], 
        "FL270": ["FS007", "FL090", "BS004", "FL090", "BS004", "FL090", "BS011"],
        "BR270": ["FS009", "BR270", "BS009"],
        "BL270": ["FS009", "BL090", "FS002", "BL180", "BS006"]
    },
    "IN": {
        # 90 degree turns: manually calibrated
        "FR090": ["FS008", "FR090", "BS008"],
        "FL090": ["FS006", "FL090", "BS011"],
        "BR090": ["FS009", "BR090", "BS009"],
        "BL090": ["FS010", "BL090", "BS006"],
        # 180 degree turns: manually calibrated
        "FR180": ["FS006", "FR180", "BS012"],
        "FL180": ["FS006", "FL090", "BS005", "FL090", "BS012"],
        "BR180": ["FS009", "BR180", "BS008"],
        "BL180": ["FS009", "BL090", "FS003", "BL090", "BS007"],
        # 270 degree turns: approximated using 180 degree turn + 90 degree turn
        "FR270": ["FS006", "FR180", "BS004", "FR090", "BS008"], 
        "FL270": ["FS006", "FL090", "BS005", "FL090", "BS006", "FL090", "BS011"],
        "BR270": ["FS009", "BR270", "BS009"],
        "BL270": ["FS009", "BL090", "FS003", "BL090", "FS003", "BL090", "BS006"]
    },
    "NONE": {}
}
STM_COMMAND_ADJUSTMENT_MAP = STM_COMMAND_ADJUSTMENT_DICT[LOCATION]

# Task 2: translate PC commands for moving around obstacles to STM_NAV_COMMAND_FORMAT
STM_OBS_ROUTING_MAP = {
    "FIRSTLEFT": ["FL056", "FR056", "FS005", "FR056", "FL056"],
    "FIRSTRIGHT": ["FR056", "FL056", "FS005", "FL056", "FR056"],
    "SECONDLEFT": ["FR090", "JB100", "IB100", "FS015",
                   "BL090", "BS010", "BL090", "JB100", 
                   "IB200", "FS020", 
                   "BR090"],
    "SECONDRIGHT": ["FR090", "JB100", "IF100",
                    "FL090", "FL090", "JF100",
                    "IF200", 
                    "FL090"]
}
STM_XDIST_COMMAND_FORMAT = "^[IJ][FB][0-9]{3}$"
STM_YDIST_COMMAND_FORMAT = "^YF[0-9]{3}$"
