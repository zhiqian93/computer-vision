import serial

ser = serial.Serial('COM10')

print(ser)

while ser:
   print(ser.readline().decode('utf-8'))