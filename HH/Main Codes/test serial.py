import serial
import json
import time

ser = serial.Serial('COM10', 9600)
buffer = ''
print(ser)

# while True:
#     buffer += ser.readline().decode('utf-8')
#     try:
#         data = json.loads(buffer)
#         print(data)
#         buffer = ''
#     except json.JSONDecodeError:
#         time.sleep(1)

while ser:
    print(ser.readline().decode('latin-1'))
    line = ser.readline().decode('latin-1')
    # print(type(line))
    if "HH chair" in line:
        print("OHYEAHHHH")


