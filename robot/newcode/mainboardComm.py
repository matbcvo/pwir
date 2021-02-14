import serial
import math
import threading

running = True
ser=serial.Serial('/dev/ttyACM0', 115200, timeout=0.00001)
print(ser.name)
    
class Mainboard():
    ser=serial.Serial('/dev/ttyACM0', 115200, timeout=0.00001)
    print(ser.name)

    def setspeed(x, spd1, spd2, spd3):
        print(x)
        toSend = "sd:" + str(spd1) + ":" + str(spd2) + ":" + str(spd3) + "\r\n"
        ser.write(str(toSend).encode())
        if ser.in_waiting > 0:
            line = ""
            char = ser.read().decode()
            #print("get:", get)
            while char != "\n":
                line += char
                char=ser.read().decode()
            print(line)
                        
