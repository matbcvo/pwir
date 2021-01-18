import serial
import struct
import time

motor1 = 0
motor2 = 0
motor3 = 0
thrower_speed = 3000
thrower_angle = 2700

while True:
    try:
        ser = serial.Serial(
            port='COM9',
            baudrate=115200,
            parity=serial.PARITY_NONE,
            bytesize=serial.EIGHTBITS
        )
        if(ser.isOpen()):
            #print("Serial is open, sending and reading data...")
            data = struct.pack('<3h3H', motor1, motor2, motor3, thrower_speed, thrower_angle, 0xAAAA)
            ser.write(data)
            try:
                response = ser.read_until(struct.pack('<H', 0xAAAA))
                #res = struct.unpack('<3h1H', response)
                res = struct.unpack('<3i6h3i1H', response)
                #print("Response from mainboard: " + str(res)))
                #print("Response from mainboard: " + str(res))
                #print("S1=" + str(res[0]) + "; S2=" + str(res[1]) + "; S3=" + str(res[2]) + "; E1=" + str(res[3]) + "; E2=" + str(res[4]) + "; E3=" + str(res[5]))
                #print("PC1=" + str(res[3]) + "; PC2=" + str(res[4]) + "; PC3=" + str(res[5]) + "; E1=" + str(res[6]) + "; E2=" + str(res[7]) + "; E3=" + str(res[8]))
                #print("PC1=" + str(res[3]) + ";\tPC2=" + str(res[4]) + ";\tPC3=" + str(res[5]) + ";\tSOE1=" + str(res[9]) + ";\tSOE2=" + str(res[10]) + ";\tSOE3=" + str(res[11]))
                print("PC1=" + str(res[3]) + "; PC2=" + str(res[4]) + "; PC3=" + str(res[5]))
            except struct.error as error:
                print(error)
            ser.close()
        else:
            print("Serial is not open, cannot send data...")
    except serial.SerialException as error:
        print(error)
    time.sleep(0.5)
