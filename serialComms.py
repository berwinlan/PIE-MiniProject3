import os

os.system("pip install pyserial")

# Importing Libraries
import serial
import time

COM_PORT = 'COM13'
BAUDRATE = 115200   # must match Arduino

arduino = serial.Serial(port=COM_PORT, baudrate=BAUDRATE, timeout=.1)

def write_read(x):
    arduino.write(bytes(x, 'utf-8'))
    time.sleep(0.05)
    data = arduino.readline()
    return data
while True:
    num = input("Enter a number: ") # Taking input from user
    value = write_read(num)
    print(value) # printing the value