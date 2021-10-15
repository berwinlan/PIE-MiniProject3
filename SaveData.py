from serial import Serial       # requires PySerial to be downloaded
import os

PORT = 'COM13'  # serial port for Arduino
BAUD_RATE = 115200  # set baud rate matching serial monitor
FILENAME = "driveData.csv"

ser = serial.Serial(PORT, BAUD_RATE)
print(f"Connected at {PORT}!")

# Get the path to the folder that this script is in: https://stackoverflow.com/a/4060259
__location__ = os.path.realpath(os.path.join(os.getcwd(), os.path.dirname(__file__)))

# Create headings
with open(os.path.join(__location__, FILENAME), 'w') as f:
    f.write("heading1,heading2\n")

getData = str(ser.readline())
# data = getData[0:-2]        # slice off EOF characters

with open(os.path.join(__location__, FILENAME), 'a') as f:
    f.write(getData)
