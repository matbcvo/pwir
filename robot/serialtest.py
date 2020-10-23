import serial
import time
ser=serial.Serial('/dev/ttyACM0', 115200, timeout=0.1)
print(ser.name)         # check which port was really used
ser.write(b'hello')     # write a string           # close port
counter = 0
time.sleep(1)

while True:
    ser.write(f"sd:10:10:10\n".encode())
    get = ser.readline().decode()
    print("get:", get)

    
ser.close()  