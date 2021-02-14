from threading import Thread
import cv2

class IMG:
    def commandThread(self):
        while self.running:
            (self.grabbed, self.currentFrame) = self.cap.read()
            print("igaks juhuks prindin midagi")

    def __init__(self, src=0):
        #self.rs_config = config.config()
        #realsense_config.configure()
        # create initial variables for use in methods
        self.running = True
        self.cap = cv2.VideoCapture(4)
        (self.grabbed, self.currentFrame) = self.cap.read()

        # create and start the pipeline with a color image stream
        
        Thread(name="camThread", target=self.commandThread).start()

    def getFrame(self):
        return (self.grabbed, self.currentFrame)

    def setStopped(self, stopped):
        self.running = stopped
        self.cap.release()