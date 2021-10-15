import serial       # requires PySerial to be downloaded
import os
import calendar
import time

PORT = 'COM13'  # serial port for Arduino
BAUD_RATE = 115200  # set baud rate matching serial monitor

# get time
gmt = time.gmtime()
timestamp = calendar.timegm(gmt)

FILENAME = f"{timestamp}_driveData.csv"

ser = serial.Serial(PORT, BAUD_RATE)
print(f"Connected at {PORT}!")

# Get the path to the folder that this script is in: https://stackoverflow.com/a/4060259
__location__ = os.path.realpath(os.path.join(os.getcwd(), os.path.dirname(__file__)))

# Create headings
with open(os.path.join(__location__, FILENAME), 'w') as f:
    f.write("leftSensorData,rightSensorData,leftMotorData,rightSensorData\n")

samples = 25    # how many samples to collect
line = 0        # start at 0 because our header is 0 (not real data)

while line <= samples:
    getData = str(ser.readline())
    data = getData[2:-5]        # slice off EOF characters (b' - \r\n')

    if data != "Motor Shield found.":
        with open(os.path.join(__location__, FILENAME), 'a') as f:
            f.write(f"{data}\n")
        print("Collecting data at row {line}...")
        line += 1

print(f"Data saved at {FILENAME}.")