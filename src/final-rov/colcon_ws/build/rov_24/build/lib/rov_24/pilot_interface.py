import sys
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel
from PyQt5.QtCore import Qt, QTimer, QByteArray, QBuffer, QIODevice
from PyQt5.QtGui import QPixmap, QImage
import cv2
from multiprocessing import Process, Queue

IPs = ["rtsp://192.168.1.120:8554/video0_unicast", "rtsp://192.168.1.120:8554/video2_unicast", "rtsp://192.168.1.123:8554/video0_unicast",
       "rtsp://192.168.1.123:8554/video2_unicast", "rtsp://192.168.1.123:8554/video4_unicast", "rtsp://192.168.1.123:8554/video6_unicast"]

def camera_process(cam_IP, queue):
    pipeline = "rtspsrc location=" + cam_IP + " latency=0 buffer-mode=auto ! decodebin ! videoconvert ! appsink max-buffers=1 drop=True"
    cap = cv2.VideoCapture(pipeline)
    while True:
        try:
            ret, frame = cap.read()  # Read frame from the camera
            if ret:
                # Convert frame to RGB format
                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                # Convert frame to QImage
                image = QImage(frame_rgb, frame_rgb.shape[1], frame_rgb.shape[0], QImage.Format_RGB888)
                
                # Convert QImage to bytes
                byte_array = QByteArray()
                buffer = QBuffer(byte_array)
                buffer.open(QIODevice.WriteOnly)
                image.save(buffer, "PNG")
                buffer.close()

                queue.put(byte_array)
        except:
            pass

class PilotInterface(QWidget):
    def __init__(self):
        super().__init__()
        self.button_clicked_flag = False
        self.cameras = {'under': IPs[5], 'side': IPs[4], 'front': IPs[0], 'LG': IPs[2], 'M': IPs[1], 'RG': IPs[3]}
        self.queues = [Queue() for _ in range(len(self.cameras))]  # Queue for each camera
        self.processes = [Process(target=camera_process, args=(cam_index, queue)) for cam_index, queue in zip(self.cameras.values(), self.queues)]
        for process in self.processes:
            process.start()
        self.initUI()

        self.closeEvent = self.cleanup

    def initUI(self):
        # Main layout
        self.main_layout = QVBoxLayout()
        self.setLayout(self.main_layout)

        # Top layout
        self.top_layout = QHBoxLayout()

        self.top_left_layout = QHBoxLayout()
        self.top_right_layout = QHBoxLayout()
        self.top_layout.addLayout(self.top_left_layout, 4)
        self.top_layout.addLayout(self.top_right_layout, 5)

        self.main_layout.addLayout(self.top_layout, 6)

        self.bottom_layout = QHBoxLayout()
        self.main_layout.addLayout(self.bottom_layout, 4)

        ## top left
        self.display_camera() # under / side

        ## top right
        self.display_camera(2) # Front

        # Bottom layout
        self.display_camera(3) # Left Gripper
        self.display_camera(4) # Middle
        self.display_camera(5) # Right Gripper

        self.setWindowTitle('Pilot Interface')
        self.setGeometry(100, 100, 500, 300)
        self.show()

    def display_camera(self, cam_index='none', s=0):
        layouts = {2: self.top_right_layout, 3: self.bottom_layout, 4: self.bottom_layout, 5: self.bottom_layout}
        image_label = QLabel()
        timer = QTimer(self)

        if cam_index == 'none':
            self.top_left_layout.addWidget(image_label)
            if s == 0:
                timer.timeout.connect(lambda: self.update_frame(image_label, 0))
            else:
                timer.timeout.connect(lambda: self.update_frame(image_label, 1))

            self.add_switch(image_label)

        else:
            layouts[cam_index].addWidget(image_label)
            timer.timeout.connect(lambda: self.update_frame(image_label, cam_index))  

        timer.start(40)  # Update frame every 40 ms (25 fps)

    
    def update_frame(self, image_label, cam_index):
        try:
            byte_array = self.queues[cam_index].get()
            image = QImage()
            image.loadFromData(byte_array)

            image_label.setPixmap(QPixmap.fromImage(image))
        except RuntimeError:
            pass

    def button_clicked(self):
        self.button_clicked_flag = not self.button_clicked_flag
        self.clear_layout(self.top_left_layout)
        self.display_camera('none', s=int(self.button_clicked_flag))
        print("Button Clicked:", self.button_clicked_flag)
        
    def add_switch(self, cam_label):
        switch_button = QPushButton("SWITCH")
        switch_button.setStyleSheet("background-color: #A3C1DA; color: black;")
        self.top_left_layout.addWidget(switch_button, alignment=Qt.AlignBottom | Qt.AlignRight)

        self.top_left_rect_layout = QVBoxLayout()
        switch_button.clicked.connect(self.button_clicked)

        self.top_left_rect_layout.addWidget(switch_button, alignment=Qt.AlignBottom | Qt.AlignRight)
        cam_label.setLayout(self.top_left_rect_layout)
    
    def clear_layout(self, layout):
        while layout.count():
            widget = layout.takeAt(0).widget()
            if widget is not None:
                widget.deleteLater()

    def cleanup(self, event):
        for process in self.processes:
            process.terminate()
            process.join()
        for camera in self.cameras:
            camera.release()
        super().closeEvent(event)


def init_pilot_interface():
    app = QApplication(sys.argv)
    window = PilotInterface()
    sys.exit(app.exec_())