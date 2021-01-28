from tkinter import *
import serial
import struct

def sel():
    ser = serial.Serial(
        port='/dev/ttyACM0',
        baudrate=115200,
        parity=serial.PARITY_NONE,
        bytesize=serial.EIGHTBITS
        )
    selection = ""
    if(ser.isOpen()):
        selection += "Serial is OPEN, data sent\n"
        data = struct.pack('<3h3H', motor1.get(), motor2.get(), motor3.get(), thrower_speed.get(), thrower_angle.get(), 0xAAAA)
        ser.write(data)
        response = ser.read_until(struct.pack('<H', 0xAAAA))
        label_response.config(text = "Response from mainboard: " + str(struct.unpack('<6h3i1H', response)))
        ser.close()
    else:
        selection += "Serial ERROR, could not send data over serial\n"
    """selection += "Motor #1 speed: " + str(motor1.get()) + "\n"
    selection += "Motor #2 speed: " + str(motor2.get()) + "\n"
    selection += "Motor #3 speed: " + str(motor3.get()) + "\n"
    selection += "Thrower speed: " + str(thrower_speed.get()) + "\n"
    selection += "Thrower angle: " + str(thrower_angle.get()) + "\n"
    label.config(text = selection)
"""

root = Tk()
root.geometry("400x300")

label_motor1 = Label(root, text="Motor #1")
label_motor1.grid(row=0, column=1)

motor1 = IntVar()
scale = Scale( root, variable = motor1, from_=-50, to=50 )
scale.grid(row=1, column=1)

label_motor2 = Label(root, text="Motor #2")
label_motor2.grid(row=0, column=2)

motor2 = IntVar()
scale = Scale( root, variable = motor2, from_=-50, to=50 )
scale.grid(row=1, column=2)

label_motor3 = Label(root, text="Motor #3")
label_motor3.grid(row=0, column=3)

motor3 = IntVar()
scale = Scale( root, variable = motor3, from_=-50, to=50 )
scale.grid(row=1, column=3)

label_thrower_speed = Label(root, text="Thrower Speed")
label_thrower_speed.grid(row=0, column=4)

thrower_speed = IntVar()
scale = Scale( root, variable = thrower_speed, from_=3000, to=6400)
scale.grid(row=1, column=4)

label_thrower_angle = Label(root, text="Thrower Angle")
label_thrower_angle.grid(row=0, column=5)

thrower_angle = IntVar()
scale = Scale( root, variable = thrower_angle, from_=2700, to=6400 )
scale.grid(row=1, column=5)

button = Button(root, text="Send data to mainboard", command=sel)
button.grid(row=2, column=0, columnspan=4)

label = Label(root, anchor="e", justify=LEFT)
label.grid(row=3, column=0, columnspan=4)

label_response = Label(root, anchor="e", justify=LEFT)
label_response.grid(row=4, column=0, columnspan=6)

root.mainloop()