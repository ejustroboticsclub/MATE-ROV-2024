#!/usr/bin/env python3

import cv2
import rospy
import tkinter as tk
from math import pi
from std_msgs.msg import Float32MultiArray, Float64
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion
from tkinter import Canvas, NW, Frame, Button
from screeninfo import get_monitors
from PIL import Image, ImageTk
from threading import Thread

class controlRoom:
    def __init__(self, window, geometry):
        self.window = window
        self.geometry = geometry
        self.initUI()
        self.nets = []

    def initUI(self):

        
        self.screen_width = self.geometry[0]
        self.screen_height = self.geometry[1]
        window_size = f"{self.screen_width}x{self.screen_height}+{self.geometry[2]}+{self.geometry[3]}"
        self.window.geometry(window_size)
        self.window.title("Shiru Kaijen Control Room")

        bg = Image.open("back.jpg")
        bg = bg.resize((self.screen_width, self.screen_height))
        self.bg = ImageTk.PhotoImage(bg)

        self.canvas = Canvas(self.window)
        self.canvas.create_image(0, 0, image=self.bg, anchor=NW)
        self.canvas.create_text(self.screen_width//2, 100, text= "Shiru Kaijen Control Room",
                                fill="white", font=("Garamond", 30, "bold"))
        
        stShapes_frame = Frame(self.canvas)
        stShapes = Button(stShapes_frame, text="Shapes Start", font=(50), command=self.stShapes)
        self.canvas.create_window(self.screen_width-770, self.screen_height-800, window=stShapes_frame, anchor=NW)
        
        shapes_frame = Frame(self.canvas)
        shapes = Button(shapes_frame, text="Capture", font=(50), command=self.calculateShapes)
        self.canvas.create_window(self.screen_width-500, self.screen_height-800, window=shapes_frame, anchor=NW)
        
        self.canvas.create_text(self.screen_width-770, self.screen_height-725, text= "Score: ",
                                fill="white", font=("Garamond", 20, "bold"), anchor=NW)
        
        stFish_frame = Frame(self.canvas)
        stFish = Button(stFish_frame, text="Fish Start", font=(50), command=self.stFish)
        self.canvas.create_window(self.screen_width-770, self.screen_height-675, window=stFish_frame, anchor=NW)
        
        fish_frame = Frame(self.canvas)
        fish = Button(fish_frame, text="Capture", font=(50), command=self.calculateFish)
        self.canvas.create_window(self.screen_width-500, self.screen_height-675, window=fish_frame, anchor=NW)
        
        self.canvas.create_text(self.screen_width-770, self.screen_height-600, text= "Scores: ",
                                fill="white", font=("Garamond", 20, "bold"), anchor=NW)
        
        self.mappImage = Image.open("map.png")
        self.mappImage = self.mappImage.resize((self.screen_width//3, self.screen_height//3))
        self.mapp = ImageTk.PhotoImage(self.mappImage)
        
        self.createMap()
        
        clearMap_frame = Frame(self.canvas)
        clearMap = Button(clearMap_frame, text="Undo", font=(50), command=self.clearLast)
        self.canvas.create_window(self.screen_width-770, self.screen_height-550, window=clearMap_frame, anchor=NW)
        
        self.m1 = self.canvas.create_text(self.screen_width-1700, self.screen_height-900, text="1: 9999",
                                          fill="white", font=("Garamond", 20, "bold"), anchor=tk.NW)
        self.m2 = self.canvas.create_text(self.screen_width-1700, self.screen_height-825, text="2: 9999",
                                          fill="white", font=("Garamond", 20, "bold"), anchor=tk.NW)
        self.m3 = self.canvas.create_text(self.screen_width-1700, self.screen_height-750, text="3: 9999",
                                          fill="white", font=("Garamond", 20, "bold"), anchor=tk.NW)
        self.m4 = self.canvas.create_text(self.screen_width-1700, self.screen_height-675, text="4: 9999", 
                                          fill="white", font=("Garamond", 20, "bold"), anchor=tk.NW)
        self.m5 = self.canvas.create_text(self.screen_width-1700, self.screen_height-600, text="5: 9999", 
                                          fill="white", font=("Garamond", 20, "bold"), anchor=tk.NW)
        self.m6 = self.canvas.create_text(self.screen_width-1700, self.screen_height-525, text="6: 9999", 
                                          fill="white", font=("Garamond", 20, "bold"), anchor=tk.NW)
        self.roll = self.canvas.create_text(self.screen_width-1450, self.screen_height-900, text="Roll:  999.99",
                                            fill="white", font=("Garamond", 20, "bold"), anchor=tk.NW)
        self.pitch = self.canvas.create_text(self.screen_width-1450, self.screen_height-825, text="Pitch: 999.99", 
                                             fill="white", font=("Garamond", 20, "bold"), anchor=tk.NW)
        self.yaw = self.canvas.create_text(self.screen_width-1450, self.screen_height-750, text="Yaw:   999.99",
                                           fill="white", font=("Garamond", 20, "bold"), anchor=tk.NW)
        self.wz = self.canvas.create_text(self.screen_width-1150, self.screen_height-900, text="Wz:     999.99",
                                          fill="white", font=("Garamond", 20, "bold"), anchor=tk.NW)
        self.depth = self.canvas.create_text(self.screen_width-1150, self.screen_height-825, text="Depth: 0.0",
                                             fill="white", font=("Garamond", 20, "bold"), anchor=tk.NW)
        
        # rospy.Subscriber("ROV/depth", Float64, self.depth_update)
        rospy.Subscriber("ROV/thrusters", Float32MultiArray, self.thrusters_update)
        rospy.Subscriber("/imu/data", Imu, self.imu_update)

        
        self.canvas.pack(fill="both", expand=True)
        buttons = [shapes, stShapes, stFish, fish, clearMap]
        for i in buttons:
            i.pack()
        #self.window.after(100, self.updateData)
    
    def stShapes(self):
        name = "Shapes"
        cv2.namedWindow(name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(name, 960, 540)
        cv2.moveWindow(name, 150, 440)

        IP = "rtsp://192.168.1.102:8554/unicast"
        pipeline = "rtspsrc location=" + IP + " latency=0 buffer-mode=auto ! decodebin ! videoconvert ! appsink"
        self.cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        #self.cap = cv2.VideoCapture(0)
        while 1: 

            _,img = self.cap.read()

            cv2.imshow(name, img)

            if(cv2.waitKey(1) == ord('q')):
                break

    def calculateShapes(self):
        self.cap.release()
        cv2.destroyAllWindows()
    
    def stFish(self):
        name = "Fish"
        cv2.namedWindow(name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(name, 960, 540)
        cv2.moveWindow(name, 150, 440)

        IP = "rtsp://192.168.1.102:8554/unicast"
        pipeline = "rtspsrc location=" + IP + " latency=0 buffer-mode=auto ! decodebin ! videoconvert ! appsink"
        self.cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        #self.cap = cv2.VideoCapture(0)
        while 1: 

            _,img = self.cap.read()

            cv2.imshow(name, img)

            if(cv2.waitKey(1) == ord('q')):
                break
    
    def calculateFish(self):
        self.cap.release()
        cv2.destroyAllWindows()
    
    def on_left_click(self, event):
        self.nets.append(self.map_canvas.create_text(event.x, event.y, text="X", fill="red", font=("bold")))
    
    def on_right_click(self, event):
        self.nets.append(self.map_canvas.create_text(event.x, event.y, text= "O", fill="red", font=("bold")))
        
    def clearLast(self):
        try:
            self.map_canvas.delete(self.nets[-1])
            self.nets.pop()
        except:
            pass
        
    def createMap(self):
        self.map_canvas = Canvas(self.canvas, width=self.mappImage.width, height=self.mappImage.height)
        self.map_canvas.create_image(0, 0, image=self.mapp, anchor=NW)
        self.map_canvas.bind("<Button-1>", self.on_left_click)
        self.map_canvas.bind("<Button-3>", self.on_right_click)
        self.canvas.create_window(self.screen_width-770, self.screen_height-470, window=self.map_canvas, anchor=NW)
    
    def depth_update(self,depth):
        dz = str(round(depth.data, 2))
        self.canvas.itemconfig(self.depth, text="Depth: " + dz)

    def thrusters_update(self, thrusters):
        m1Speed = str(round(thrusters.data[0], 2))
        m2Speed = str(round(thrusters.data[1], 2))
        m3Speed = str(round(thrusters.data[2], 2))
        m4Speed = str(round(thrusters.data[3], 2))
        m5Speed = str(round(thrusters.data[4], 2))
        m6Speed = str(round(thrusters.data[5], 2))
        self.canvas.itemconfig(self.m1, text="1: " + m1Speed)
        self.canvas.itemconfig(self.m2, text="2: " + m2Speed)
        self.canvas.itemconfig(self.m3, text="3: " + m3Speed)
        self.canvas.itemconfig(self.m4, text="4: " + m4Speed)
        self.canvas.itemconfig(self.m5, text="5: " + m5Speed)
        self.canvas.itemconfig(self.m6, text="6: " + m6Speed)

    def imu_update(self, imu):
        _wx, _wy, wz = [
            imu.angular_velocity.x,
            imu.angular_velocity.y,
            imu.angular_velocity.z,
        ]
        roll, pitch, yaw = euler_from_quaternion(
            [imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w]
        )
        roll *= 180 / pi
        pitch *= 180 / pi
        yaw *= 180 / pi
        roll = str(round(roll, 2))
        pitch = str(round(pitch, 2))
        yaw = str(round(yaw, 2))
        wz = str(round(wz, 2))
        self.canvas.itemconfig(self.roll, text="Roll: " + roll)
        self.canvas.itemconfig(self.pitch, text="Pitch: " + pitch)
        self.canvas.itemconfig(self.yaw, text="Yaw: " + yaw)
        self.canvas.itemconfig(self.wz, text="Wz:     " + wz)
        

rospy.init_node("rov_gui", anonymous=True)
IPs = ["rtsp://192.168.1.101:8554/unicast", "rtsp://192.168.1.103:8554/unicast",
       "rtsp://192.168.1.103:8555/unicast"]
#IPs = [0, "https://192.168.1.7:8080/video"]
names = ["Main", "Gripper", "Back"]
pos = [(1/2,1/4), (0,0), (0,1/2)]
def launchCamera(IP, name, pos, screen):
    
    w, h, x, y = screen
    cv2.namedWindow(name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(name, w//2, h//2-35)
    cv2.moveWindow(name, x+int(pos[0]*w), y+int(pos[1]*h))
    
    pipeline = "rtspsrc location=" + IP + " latency=0 buffer-mode=auto ! decodebin ! videoconvert ! appsink"
    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    #cap = cv2.VideoCapture(IP)
    while 1: 
        
        _,img = cap.read()
        
        cv2.imshow(name, img)
        
        if(cv2.waitKey(1) == ord('q')):
            break
        
    cap.release()
    cv2.destroyAllWindows()
    
def getScreensInfo(screen):
    monitors = get_monitors()
    if len(monitors) > 1 and screen == 2:
        second_screen = monitors[1]
        return second_screen.width, second_screen.height, second_screen.x, second_screen.y
    else:
        first_screen = monitors[0]
        return first_screen.width, first_screen.height, first_screen.x, first_screen.y
    
def launchControlRoom():
    window = tk.Tk()
    controlRoomApp = controlRoom(window, getScreensInfo(1))
    window.mainloop()

if __name__ == "__main__":
    controlThread = Thread(target=launchControlRoom)
    controlThread.start()
    for i in range(len(IPs)):
        thread = Thread(target=launchCamera,args=(IPs[i], names[i], pos[i], getScreensInfo(2)))
        thread.start()
    rospy.spin()
