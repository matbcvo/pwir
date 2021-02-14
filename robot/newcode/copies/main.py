import serial
from imageDetection import *
from threading import Thread
import threading
from mainboardComm import serialComm

running = True
Thread(name="imageThread", target=imageThread).start()

rightWheelAngle = 120
leftWheelAngle = 240
rearWheelAngle = 0

def toBall(ball_y, ball_x):
    robotSpeed = -20
    ball_x = ball_x - 320
    robotDirectionAngle = math.degrees(math.atan2(ball_y, ball_x))
    print("robotDirectionAngle: ",robotDirectionAngle)
    #640x400ish
    rightWheelLinearVelocity = int(robotSpeed * math.cos(math.radians(robotDirectionAngle - rightWheelAngle)))
    leftWheelLinearVelocity = int(robotSpeed * math.cos(math.radians(robotDirectionAngle - leftWheelAngle)))
    rearWheelLinearVelocity = int(robotSpeed * math.cos(math.radians(robotDirectionAngle - rearWheelAngle)))
    print("rightWheelLinearVelocity = " + str(rightWheelLinearVelocity))
    print("leftWheelLinearVelocity = " + str(leftWheelLinearVelocity))
    print("rearWheelLinearVelocity = " + str(rearWheelLinearVelocity))
    print(f"sd:"+str(int(leftWheelLinearVelocity))+":"+str(int(rightWheelLinearVelocity))+":"+str(int(rearWheelLinearVelocity))+"\n")
    #ser.write(str(f"sd:"+str(int(rightWheelLinearVelocity))+":"+str(int(leftWheelLinearVelocity))+":"+str(int(rearWheelLinearVelocity))+"\n").encode())
    
    serialComm(rightWheelLinearVelocity, leftWheelLinearVelocity, rearWheelLinearVelocity)
        
while True:
    print("siin")
    print(ball_x, ball_y)
    toBall(ball_y, ball_x)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        imageThread().join()
        running = False