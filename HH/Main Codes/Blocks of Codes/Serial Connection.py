import serial

ser = serial.Serial('COM10', 9600)
buffer = ''
print(ser)

while ser:
    print(ser.readline().decode('latin-1'))
    line = ser.readline().decode('latin-1')
    # print(type(line))
    if "HH chair" in line:
        print("OHYEAHHHH")
        case = "HH Performed"


