import serial
import struct
from enum import Enum

# PARAMETERS
BYPASS_INIT = False  # Skip DVL setup
CHECK_CHECKSUM = True  # Print checksum error warnings (will not raise an execption!) --> useful for debugging
DEBUG_MODE = False  # Check estimated message length
SHOW_TIME = True  # Print DVL's internal RTC

BAUD = 115200  # DVL should be at 115200. Default is technically 9600
PORT = "/dev/ttyUSB0"  # DVL Serial port, may need to be changed!

# COMMANDS
CMD_CS = ("cs\r").encode("ascii")
CMD_BREAK = ("===").encode("ascii")
CMD_PD0 = ("#pd0\r").encode("ascii")


# STATES
class State(Enum):
    INIT = (0,)
    SEARCHING = (1,)
    READING = (2,)
    DEBUG = 3


# Other variables
prevByte = None
ensembleLength = None
readLength = None
numDataTypes = None  # this is N
readOffset = (
    4 + 1
)  # translation factor from array to tables in documentation, accounts for index-by-0
idx = 0
prevTime = None


# Confirm data integrity with checksum, useful for debugging
# Computes modulo 65535 checksum as specified by documentation
def checkChecksum(data, checkLength):
    checksumCalc = (
        sum(data[:-2]) + sum(checkLength) + 0x7F + 0x7F
    ) % 65535  # 0x7F are for headers
    checksumCheck = data[-2] + (data[-1] << 8)
    if checksumCalc != checksumCheck:
        print("WARNING: Checksum mismatch!")


## Main Execution Loop
activeState = State.INIT

while True:
    if activeState == State.INIT:  # should NEVER return to this state!
        ser = serial.Serial(PORT, BAUD)  # configure port
        if BYPASS_INIT == False:
            ser.write(CMD_PD0)  # force PD0 datastream
            ser.write(CMD_CS)  # begin execution at reliable timing
        if DEBUG_MODE == True:
            activeState = State.DEBUG
        else:
            activeState = State.SEARCHING

    elif activeState == State.SEARCHING:
        byte = ser.read(1)
        if not byte:
            continue
        if prevByte == b"\x7F" and byte == b"\x7F":
            prevByte = None  # reset memory
            activeState = State.READING
            length = ser.read(2)
            checkLength = list(length)  # hold for checksum computation
            ensembleLength = struct.unpack("<H", length)[
                0
            ]  # offset for header IDs + ensemble length but does not include checksum!!
            readLength = ensembleLength - 2
        else:
            prevByte = byte

    elif activeState == State.READING:
        # TODO: consider refactoring to avoid blocking code in Serial.read - MK
        data = list(ser.read(readLength))  # read package AFTER ensemble length bytes
        numDataTypes = data[6 - readOffset]  # read 6th byte for number of data types

        dataIdx_3 = data[11 - readOffset] + (data[12 - readOffset] << 8)
        # print(data[7 - readOffset])
        # print(data[8 - readOffset])

        # where is the fucking third data position????????? fuck
        # print(data)
        # print(data[127])

        if SHOW_TIME:  # apologies for the magic constants...
            timeSeconds = data[2 * numDataTypes + 6 + 58 + 10 - readOffset]
            timeMinutes = data[2 * numDataTypes + 6 + 58 + 9 - readOffset]
            timeHours = data[2 * numDataTypes + 6 + 58 + 8 - readOffset]
            time = f"{timeHours}:{timeMinutes}:{timeSeconds}"
            if time != prevTime:
                print("RTC: " + time)
                prevTime = time

        if CHECK_CHECKSUM:
            checkChecksum(data, checkLength)

        activeState = State.SEARCHING

    elif activeState == State.DEBUG:  # Confirms number of bytes in message
        byte = ser.read(1)
        if prevByte == b"\x7F" and byte == b"\x7F":
            print(idx)
            prevByte = None
            idx = 0
        else:
            idx = idx + 1
            prevByte = byte
