import serial

ser = serial.Serial('COM9')

print(ser)

while ser:
   print(ser.readline().decode('utf-8'))