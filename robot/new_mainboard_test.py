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
            port='/dev/ttyACM0',
            baudrate=115200,
            parity=serial.PARITY_NONE,
            bytesize=serial.EIGHTBITS
        )
        if(ser.isOpen()):
            data = struct.pack('<3h3H', motor1, motor2, motor3, thrower_speed, thrower_angle, 0xAAAA)
            ser.write(data)
            ser.close()
        else:
            print("Serial is not open, cannot send data...")
    except serial.SerialException as error:
        print(error)
    time.sleep(0.5)