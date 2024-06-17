import tkinter as tk
from multiprocessing import Process
import cv2

class Pilot:
    def __init__(self, window, geometry, streams, sizes, positions):
        self.window = window
        self.geometry = geometry
        self.streams = streams
        self.sizes = sizes
        self.positions = positions

        self.cams = [0] * 5
        self.imgs = [0] * len(self.streams)
        self.processes = [0] * 5
        self.switch = False

        self.create_widgets()

    def create_widgets(self):
        self.screen_width = self.geometry[0]
        self.screen_height = self.geometry[1]
        window_size = f"{self.screen_width}x{self.screen_height}+{self.geometry[2]}+{self.geometry[3]}"
        self.window.geometry(window_size)
        self.window.title("Shiru Kaijen Camera Room")

        self.switch_cam = tk.Label(self.window, width=int(self.screen_width*self.sizes[0][0]), 
                                   height=int(self.screen_height*self.sizes[0][1]))
        self.switch_button = tk.Button(self.switch_cam, text="Switch", command=self.switch_cameras)
        self.main_cam = tk.Label(self.window, width=int(self.screen_width*self.sizes[1][0]), 
                                 height=int(self.screen_height*self.sizes[1][1]))
        self.gripper_l_cam = tk.Label(self.window, width=int(self.screen_width*self.sizes[2][0]), 
                                      height=int(self.screen_height*self.sizes[2][1]))
        self.gripper_r_cam = tk.Label(self.window, width=int(self.screen_width*self.sizes[3][0]), 
                                      height=int(self.screen_height*self.sizes[3][1]))
        self.bottom_cam = tk.Label(self.window, width=int(self.screen_width*self.sizes[4][0]), 
                                   height=int(self.screen_height*self.sizes[4][1]))

        self.switch_cam.place(x=int(self.screen_width*self.positions[0][0]), y=int(self.screen_height*self.positions[0][1]))
        self.switch_button.place(x=int(self.screen_width*self.positions[0][0]) - 10, y=int(self.screen_height*self.positions[0][1]) - 10)
        self.main_cam.place(x=int(self.screen_width*self.positions[1][0]), y=int(self.screen_height*self.positions[0][1]))
        self.gripper_l_cam.place(x=int(self.screen_width*self.positions[2][0]), y=int(self.screen_height*self.positions[0][1]))
        self.gripper_r_cam.place(x=int(self.screen_width*self.positions[3][0]), y=int(self.screen_height*self.positions[0][1]))
        self.bottom_cam.place(x=int(self.screen_width*self.positions[4][0]), y=int(self.screen_height*self.positions[0][1]))
        self.cams.append(self.switch_cam)
        self.cams.append(self.main_cam)
        self.cams.append(self.gripper_l_cam)
        self.cams.append(self.gripper_r_cam)
        self.cams.append(self.bottom_cam)

        self.update_frames()

    def capture(self, index, IP):
        pipeline = "rtspsrc location=" + IP + " latency=0 buffer-mode=auto ! decodebin ! videoconvert ! appsink max-buffers=1 drop=True"
    
        cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        while 1:
            _, img = cap.read()
            cap.release()

            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            if index == 0:
                img = cv2.resize(img, (self.screen_width*self.sizes[0][0], self.screen_height*self.sizes[0][1]))
            elif index == 1:
                img = cv2.resize(img, (self.screen_width**self.sizes[1][0], self.screen_height*self.sizes[1][1]))
            else:
                img = cv2.resize(img, (self.screen_width*self.sizes[2][0], self.screen_height*self.sizes[2][1]))
            
            self.cams[index].configure(image=img)
            self.cams[index].image = img

            if(cv2.waitKey(1) == ord('q')):
                break
            
        cap.release()
        cv2.destroyAllWindows()
    
    def update_frames(self):
        for i in range(len(self.streams)):
            process = Process(target=self.capture, args=(i, self.streams[i]))
            self.processes[i] = process
            process.start()
            
    def switch_cameras(self):
        self.current_cam = not self.current_cam

        if self.current_cam:
            self.processes[0].terminate()
            process = Process(target=self.capture, args=(5, self.streams[5]))
            self.processes[0] = process
            process.start()
        else:
            self.processes[0].terminate()
            process = Process(target=self.capture, args=(0, self.streams[0]))
            self.processes[0] = process
            process.start()

def init_pilot(geometry, streams):
    window = tk.Tk()
    pilot_interface = Pilot(window, geometry, streams)
    window.mainloop()