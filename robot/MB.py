import threading
import time
import serial
import struct
import math

class MB_Comm:
    m1: int = 0
    m2: int = 0
    m3: int = 0
    thrower: int = 0
    servo: int = 0
    #ball_x = 
    def commandThread(self):
        while self.running:
            time.sleep(0.002)#0.002
            #print(self.m1)
            
            
    def __init__(self):
        
        #THREAD HERE
        self.running = True
        self.w = threading.Thread(name='commandThread', target=self.commandThread)
        self.w.start()
            
    def stop(self):
        self.omni_move(0, 0, 0)

    def right(self):
        print("spinning right")
        self.omni_move(10, -10, 10)
            
    def fwd(self):
        print("going forward!")
        self.omni_move(10, 0, -10)
        #(parem, kesk, vasak)
        
    

    def omni_move(self, motor1, motor2, motor3):
        m1 = self.m1 = motor1
        m2 = self.m2 = motor2
        m3 = self.m3 = motor3
        thrower_speed = 3000
        thrower_angle = 2700
        try:
            ser = serial.Serial(
                port='/dev/ttyACM0',
                baudrate=115200,
                parity=serial.PARITY_NONE,
                bytesize=serial.EIGHTBITS
            )
            if(ser.isOpen()):
                data = struct.pack('<3h3H', motor1, -motor2, motor3, thrower_speed, thrower_angle, 0xAAAA)
                ser.write(data)
                ser.close()
            else:
                print("Serial is not open, cannot send data...")
        except serial.SerialException as error:
            print(error)
            

    def toBall(self, ball_y, ball_x):
        
        rightWheelAngle = 120
        leftWheelAngle = 240
        rearWheelAngle = 0
        
        robotSpeed = 15 #äkki siis ei viska eest ära nagu vahepeal teeb
        
        print("BALL X",ball_x)
        ball_x = ball_x - 250 #
        robotDirectionAngle = math.degrees(math.atan2(ball_y, ball_x))
        print("robotDirectionAngle: ",robotDirectionAngle)
        #640x400ish
        rightWheelLinearVelocity = robotSpeed * math.cos(math.radians(robotDirectionAngle - rightWheelAngle))
        leftWheelLinearVelocity = robotSpeed * math.cos(math.radians(robotDirectionAngle - leftWheelAngle))
        rearWheelLinearVelocity = robotSpeed * math.cos(math.radians(robotDirectionAngle - rearWheelAngle))
        print("rightWheelLinearVelocity = " + str(rightWheelLinearVelocity))
        print("leftWheelLinearVelocity = " + str(leftWheelLinearVelocity))
        print("rearWheelLinearVelocity = " + str(rearWheelLinearVelocity))
        print(f"sd:"+str(int(leftWheelLinearVelocity))+":"+str(int(rightWheelLinearVelocity))+":"+str(int(rearWheelLinearVelocity))+"\n")
        
        #motor1 =  #parem ratas
        #motor2 =  #tagumine
        #motor3 =  #vasak
        self.omni_move(int(rightWheelLinearVelocity), int(rearWheelLinearVelocity), int(leftWheelLinearVelocity))
        
    def rotateAroundBallRightByY(self, center, contours):
        if(len(contours) < 1):
            return
        
            
        print("rotating around the ball")
        rearwheelspeed = 10
        ball_x = center[0]
        print(ball_x)
        ball_y = center[1]
        print(ball_y)
        #paneme raadiuse mingi 35
        if (ball_y < 362): #& (ball_x < 400):
            print("lähen lähemale ja vasakule (i hope) yyyyyy")
            self.omni_move(5, 0, -5)
        elif (ball_x > 400):
            self.omni_move(5, 0, 0)
        elif (ball_y > 368):# & (ball_x > 360):
            print("lähen lähemale ja paremale yyyyyy")
            self.omni_move(-5, 0, -5)   
        elif (ball_x < 360):
            self.omni_move(-0, 0, 5)
        else:
            #continue:
            print("pall õigel kaugusel   yyyyyy")
            self.omni_move(-1, rearwheelspeed, 0)
        return
    
    def rotateAroundBallRightByYBasket(self, center, contours, basket_x):
        if(len(contours) < 1):
            print("ei näe palli")
            return
        
            
        print("rotating around the ball")
        rearwheelspeed = 12
        ball_x = center[0]
        print(ball_x)
        ball_y = center[1]
        print(ball_y)
        self.omni_move(3, 0, -3)
        if self.basketMiddle(basket_x) == False:
            
            self.omni_move(3, 12, 0)
            
        elif self.basketMiddle(basket_x) == True:
            self.stop()
    
    def rotateAroundBallRightByRad(self, rad):
        print("rotating around the ball")
        rearwheelspeed = 10
        
        print(rad)
        #paneme raadiuse mingi 35
        
        if (rad < 32):
            print("lähen lähemale RAD")
            self.omni_move(4, 0, -4)
            #if (ball_y > 368):
            #    break
        elif (rad > 38):
            print("lähen kaugemale RAD")
            self.omni_move(-4, 0, 4)
        else:
            #continue: 
            print("pall õigel kaugusel RAD")
            self.omni_move(3, 12, 0)
        return
    
        #tsenter suht 380,330
    #640, aga keskel on kuskil 310
    def ballMiddle(self, ball_x, cam_width):
        """
        CAM_WIDTH = 640
        CAM_HEIGHT = 480
        """
        #cond1 = ball_x < int(405)#umb 380 +- 25
        #cond2 = ball_x > int(355)
        cond1 = ball_x > int(380 - 10) # 300 min 
        cond2 = ball_x < int(380 + 10) # 340 max
        
        #print(cond1)
        #print(cond2)
        #print(cond1 & cond2)
        if ( cond1 & cond2 ):
            print("pall on keskel")
            return True
        else:
            print("pall ei ole keskel")
            return False
        
    def basketMiddle(self, basket_x):
        cond1 = basket_x < int(400)
        cond2 = basket_x > int(360)
        #print(cond1)
        #print(cond2)
        #print(cond1 & cond2)
        if ( cond1 & cond2 ):
            print("pall on keskel")
            return True
        else:
            print("pall ei ole keskel")
            return False 
        
    def startThrower(self):
        m1 = 30
        m2 = 0
        m3 = -30
        thrower_speed = 6400
        thrower_angle = 2700
        try:
            ser = serial.Serial(
                port='/dev/ttyACM0',
                baudrate=115200,
                parity=serial.PARITY_NONE,
                bytesize=serial.EIGHTBITS
            )
            if(ser.isOpen()):
                data = struct.pack('<3h3H', m1, -m2, m3, thrower_speed, thrower_angle, 0xAAAA)
                ser.write(data)
                ser.close()
            else:
                print("Serial is not open, cannot send data...")
        except serial.SerialException as error:
            print(error)
            
    def stopThrower(self):
        m1 = 0
        m2 = 0
        m3 = 0
        thrower_speed = 3000
        thrower_angle = 2700
        try:
            ser = serial.Serial(
                port='/dev/ttyACM0',
                baudrate=115200,
                parity=serial.PARITY_NONE,
                bytesize=serial.EIGHTBITS
            )
            if(ser.isOpen()):
                data = struct.pack('<3h3H', m1, -m2, m3, thrower_speed, thrower_angle, 0xAAAA)
                ser.write(data)
                ser.close()
            else:
                print("Serial is not open, cannot send data...")
        except serial.SerialException as error:
            print(error)
            
        
        