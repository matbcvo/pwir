import serial
import struct
import time

motor1 = 3000
motor2 = 3000
motor3 = 3000
thrower_speed = 3000
thrower_angle = 2700

while True:
    ser = serial.Serial(
        port='COM9',
        baudrate=115200,
        parity=serial.PARITY_NONE,
        bytesize=serial.EIGHTBITS
        )
    if(ser.isOpen()):
        print("Serial is open, sending and reading data...")
        data = struct.pack('<3h3H', motor1, motor2, motor3, thrower_speed, thrower_angle, 0xAAAA)
        ser.write(data)
        response = ser.read_until(struct.pack('<H', 0xAAAA))
        print("Response from mainboard: " + str(struct.unpack('<3h1H', response)))
        ser.close()
    else:
        print("Serial is not open, cannot send data...")
    time.sleep(0.5)