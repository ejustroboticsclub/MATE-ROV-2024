import tkinter as tk
from tkinter import ttk
from screeninfo import get_monitors
from PIL import Image, ImageTk
import matplotlib.pyplot as plt
from multiprocessing import Process
from .pilot2 import displayCamera
from os import getcwd
# from pilot import init_pilot
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64, Int32MultiArray
from geometry_msgs.msg import Twist, Vector3
from tf_transformations import euler_from_quaternion

cams_switch = {
    "Tilt": "rtsp://192.168.1.120:8554/video0_unicast", 
    "Bottom": "rtsp://192.168.1.123:8554/video2_unicast"
}
IPS = {
    "Side": "rtsp://192.168.1.123:8555/unicast",
    "Main": "rtsp://192.168.1.120:8554/video2_unicast", 
    "Gripper_L": "rtsp://192.168.1.123:8554/video0_unicast",
    "Gripper_R": "rtsp://192.168.1.123:8554/video6_unicast",
    "Switch": cams_switch["Tilt"]
}

POS = [[0, 0], [2 / 5, 0], [0, 1 / 2], [2 / 3, 1 / 2], [1 / 3, 1 / 2]]
SIZE = [[2 / 5, 1 / 2], [3 / 5, 1 / 2], [1 / 3, 1 / 2], [1 / 3, 1 / 2], [1 / 3, 1 / 2]]
NAMES = ["Switch", "Main", "Gripper L", "Gripper R", "Bottom"]

SWITCH = False
PROCESSES = [0] * 5

class CoPilot:
    def __init__(self, window, geometry):
        self.window = window
        self.geometry = geometry
        self.dir = "/home/atef/ROV 2024/colcon_ws/src/rov_24/rov_24"

        self.thrusters = [0, 0, 0, 0, 0, 0]

        self.depth = 0.0
        self.temperature = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.vx = 0.0
        self.wz = 0.0

        self.float_graphs = 0
        self.float_pressure = []
        self.float_depth = []
        self.float_time = []

        for i in range(100):
            self.float_pressure.append(i)
            self.float_depth.append(i)
            self.float_time.append(i)


        self.create_widgets()

    def create_widgets(self):
        self.screen_width = self.geometry[0]
        self.screen_height = self.geometry[1]
        window_size = f"{self.screen_width}x{self.screen_height}+{self.geometry[2]}+{self.geometry[3]}"
        self.window.geometry(window_size)
        self.window.title("Shiru Kaijen Control Room")

        title_font = ("Garamond", 35, "bold")
        label_font = ("Garamond", 20, "bold")

        self.bg = Image.open(
            f"{self.dir}/bg.jpeg"
        )
        self.bg = self.bg.resize((self.screen_width, self.screen_height))
        self.bg = ImageTk.PhotoImage(self.bg)

        self.back_canvas = tk.Canvas(
            self.window, width=self.screen_width, height=self.screen_height
        )
        self.back_canvas.create_image(0, 0, image=self.bg, anchor="nw")
        self.back_canvas.pack(fill="both", expand=True)

        self.back_canvas.create_text(
            self.screen_width // 2,
            100,
            text="Shiru Kaijen Control Room",
            font=title_font,
            fill="#ffffff",
        )

        tk.Frame(self.back_canvas, bg="").grid(
            row=0, column=0, padx=10, pady=80, sticky="ew", columnspan=3
        )

        self.back_canvas.grid_columnconfigure(0, weight=2)
        self.back_canvas.grid_columnconfigure(1, weight=1)
        self.back_canvas.grid_columnconfigure(2, weight=2)

        self.rov_frame = tk.Frame(
            self.back_canvas, bd=2, relief="groove", bg="", width=700, height=1000
        )
        self.rov_frame.grid(row=1, column=0, padx=80, pady=10, sticky="nsew")
        self.rov_frame.grid_propagate(False)

        self.rov_image = Image.open(
            f"{self.dir}/rov.jpeg"
        )
        self.rov_image = self.rov_image.resize((700, 600))
        self.rov_image = ImageTk.PhotoImage(self.rov_image)

        self.rov_canvas = tk.Canvas(
            self.rov_frame, width=self.rov_image.width(), height=self.rov_image.height()
        )
        self.rov_canvas.create_image(0, 0, image=self.rov_image, anchor="nw")
        self.rov_canvas.grid(
            row=0, column=0, padx=10, pady=10, sticky="nsew", columnspan=2
        )
        self.rov_canvas.grid_propagate(False)

        self.thrusters_labels = [0, 0, 0, 0, 0, 0]
        self.thrusters_positions = [
            [220, 120],
            [220, 480],
            [460, 480],
            [460, 120],
            [170, 300],
            [510, 300],
        ]
        for i in range(6):
            self.thrusters_labels[i] = tk.Label(
                self.rov_canvas,
                text=str(self.thrusters[i]),
                font=label_font,
                bg="#ffffff",
            )
            self.thrusters_labels[i].place(
                x=self.thrusters_positions[i][0], y=self.thrusters_positions[i][1]
            )

        tk.Label(self.rov_frame, text="Depth", font=label_font, bg="#ffffff").grid(
            row=1, column=0, padx=10, pady=10, sticky="w"
        )
        self.depth_label = tk.Label(
            self.rov_frame, text=str(self.depth), font=label_font, bg="#ffffff"
        )
        self.depth_label.grid(row=1, column=1, padx=10, pady=10, sticky="w")

        tk.Label(
            self.rov_frame, text="Temperature", font=label_font, bg="#ffffff"
        ).grid(row=2, column=0, padx=10, pady=10, sticky="w")
        self.temperature_label = tk.Label(
            self.rov_frame, text=str(self.temperature), font=label_font, bg="#ffffff"
        )
        self.temperature_label.grid(row=2, column=1, padx=10, pady=10, sticky="w")

        self.angles_frame = tk.Frame(self.back_canvas, bd=2, relief="groove", bg="")
        self.angles_frame.grid(row=1, column=1, padx=10, pady=10, sticky="ns")

        tk.Label(self.angles_frame, text="Roll", font=label_font, bg="#ffffff").grid(
            row=0, column=0, padx=10, pady=10, sticky="w"
        )
        self.roll_label = tk.Label(
            self.angles_frame, text=self.roll, font=label_font, bg="#ffffff", width="7" 
        )
        self.roll_label.grid(row=0, column=1, padx=10, pady=20, sticky="w")

        tk.Label(self.angles_frame, text="Pitch", font=label_font, bg="#ffffff").grid(
            row=1, column=0, padx=10, pady=10, sticky="w"
        )
        self.pitch_label = tk.Label(
            self.angles_frame, text=self.pitch, font=label_font, bg="#ffffff", width="7"
        )
        self.pitch_label.grid(row=1, column=1, padx=10, pady=20, sticky="w")

        tk.Label(self.angles_frame, text="Yaw", font=label_font, bg="#ffffff").grid(
            row=2, column=0, padx=10, pady=10, sticky="w"
        )
        self.yaw_label = tk.Label(
            self.angles_frame, text=self.yaw, font=label_font, bg="#ffffff", width="7"
        )
        self.yaw_label.grid(row=2, column=1, padx=10, pady=20, sticky="w")

        tk.Label(self.angles_frame, text="Vx", font=label_font, bg="#ffffff").grid(
            row=3, column=0, padx=10, pady=10, sticky="w"
        )
        self.vx_label = tk.Label(
            self.angles_frame, text=self.vx, font=label_font, bg="#ffffff", width="7"
        )
        self.vx_label.grid(row=3, column=1, padx=10, pady=20, sticky="w")

        tk.Label(self.angles_frame, text="Wz", font=label_font, bg="#ffffff").grid(
            row=4, column=0, padx=10, pady=10, sticky="w"
        )
        self.wz_label = tk.Label(
            self.angles_frame, text=self.wz, font=label_font, bg="#ffffff", width="7"
        )
        self.wz_label.grid(row=4, column=1, padx=10, pady=20, sticky="w")

        self.coral_head_button = tk.Button(
            self.angles_frame,
            text="Coral Head",
            font=label_font,
            bg="#ffffff",
            command=self.coral_head,
        )
        self.coral_head_button.grid(
            row=5, column=0, padx=10, pady=20, sticky="sew", columnspan=2
        )

        self.switch_cam_button = tk.Button(
            self.angles_frame,
            text="Switch Camera",
            font=label_font,
            bg="#ffffff",
            command=switch_cam,
        )
        self.switch_cam_button.grid(
            row=6, column=0, padx=10, pady=20, sticky="sew", columnspan=2
        )

        self.float_frame = tk.Frame(self.back_canvas, bd=2, relief="groove", bg="")
        self.float_frame.grid(row=1, column=2, padx=100, pady=10, sticky="nse")

        self.float_table = VerticalScrolledFrame(self.float_frame)
        self.float_table.grid(row=0, column=0, padx=10, pady=10, sticky="nsew")

        self.float_data = ttk.Treeview(
            self.float_table.interior,
            columns=("Company Number", "Pressure", "Depth", "Time"),
            style="Custom.Treeview",
        )
        self.float_data.heading("#0", text="ID")
        self.float_data.heading("Company Number", text="Company Num", anchor="center")
        self.float_data.heading("Pressure", text="Pressure", anchor="center")
        self.float_data.heading("Depth", text="Depth", anchor="center")
        self.float_data.heading("Time", text="Time", anchor="center")

        self.float_data.column("#0", anchor="center", width=70)
        self.float_data.column("Company Number", anchor="center", width=150)
        self.float_data.column("Pressure", anchor="center", width=120)
        self.float_data.column("Depth", anchor="center", width=120)
        self.float_data.column("Time", anchor="center", width=120)

        self.custom_style = ttk.Style()
        self.custom_style.configure("Custom.Treeview", font=("Garamond", 16))
        self.custom_style.configure(
            "Custom.Treeview.Heading", font=("Garamond", 16, "bold")
        )

        for i in range(100):
            self.float_data.insert(
                "",
                "end",
                text=str(i),
                values=(
                    "EX",
                    str(self.float_pressure[i]),
                    str(self.float_depth[i]),
                    str(self.float_time[i]),
                ),
            )
        self.float_data.pack(fill=tk.BOTH, expand=True)

        plt.plot(self.float_time, self.float_depth)
        plt.savefig(
            f"{self.float_graphs}.png"
        )
        plt.close()

        self.float_graph = Image.open(
            rf"{self.float_graphs}.png"
        )
        self.float_graph = self.float_graph.resize((600, 400))
        self.float_graph = ImageTk.PhotoImage(self.float_graph)

        self.float_canvas = tk.Canvas(self.float_frame, width=600, height=400)
        self.float_canvas.create_image(0, 0, image=self.float_graph, anchor="nw")
        self.float_canvas.grid(row=1, column=0, padx=10, pady=10, sticky="sew")

    def coral_head(self):
        pass

def switch_cam():
    global SWITCH
    SWITCH = not SWITCH
    
    if not SWITCH:
        IPS["Switch"] = cams_switch["Tilt"]
    else:
        IPS["Switch"] = cams_switch["Bottom"]

    PROCESSES[4].terminate()
    PROCESSES[4].join()
    process = Process(
        target=displayCamera,
        args=(IPS["Switch"], "Switch", SIZE[4], POS[4], getScreensInfo(1)),
    )
    process.start()
    PROCESSES[4] = process

class CoPilotNode(Node):
    def __init__(self, copilot_obj: CoPilot):
        super().__init__("GUI_node")
        self.thrusters_subscriber = self.create_subscription(
            Int32MultiArray, "/ROV/thrusters", self.thrusters_callback, 10
        )
        self.depth_subscriber = self.create_subscription(
            Float64, "/ROV/depth", self.depth_callback, 10
        )
        self.imu_subscriber = self.create_subscription(
            Imu, "/ROV/imu", self.imu_callback, 10
        )
        self.vel_subscriber = self.create_subscription(
            Twist, "/ROV/cmd_vel", self.vel_callback, 10
        )
        self.temp_subscriber = self.create_subscription(
            Float64, "/ROV/temperature", self.temp_callback, 10
        )
        # self.float_subscriber = self.create_subscription(
        #     Vector3, "/ROV/float", self.float_callback, 10
        # )

        self.copilot_obj = copilot_obj
        # self.start_flag = False

    def thrusters_callback(self, thrusters_msg: Int32MultiArray):

        for i in range(6):
            self.copilot_obj.thrusters[i] = thrusters_msg.data[i]
            self.copilot_obj.thrusters_labels[i].config(text=str(self.copilot_obj.thrusters[i]))

    def depth_callback(self, depth_msg: Float64):
        self.copilot_obj.depth = depth_msg.data
        self.copilot_obj.depth_label.config(text=str(self.copilot_obj.depth))

    def temp_callback(self, temp_msg):
        self.copilot_obj.temperature = temp_msg.data
        self.copilot_obj.temperature_label.config(text=str(self.copilot_obj.temperature))

    def vel_callback(self, twist_msg: Twist):

        self.copilot_obj.vx = twist_msg.linear.x
        self.copilot_obj.wz = twist_msg.angular.z
        self.copilot_obj.vx_label.config(text=str(self.copilot_obj.vx))
        self.copilot_obj.wz_label.config(text=str(self.copilot_obj.wz))

    # def float_callback(self, float_msg: Vector3):
        # float_msg_data = float_msg.data

        # if float_msg_data == [0, 0, 0]:
        #     plt.clear()
        #     self.float_pressure = []
        #     self.float_depth = []
        #     self.float_time = []
        #     self.start_flag = True

        # elif float_msg_data != [10, 10, 10] and self.start_flag:
        #     pressure = float_msg_data.x
        #     depth = loat_msg_data.y
        #     time = loat_msg_data.z

        #     self.float_pressure.append(pressure)
        #     self.float_depth.append(depth)
        #     self.float_time.append(time)

        # elif float_msg_data == [10, 10, 10] and self.start_flag:
        #     self.float_data.delete(*self.float_data.get_children())
        #     for i in range(100):
        #         self.float_data.insert(
        #             "",
        #             "end",
        #             text=str(i),
        #             values=(
        #                 str(self.float_pressure[i]),
        #                 str(self.float_depth[i]),
        #                 str(self.float_time[i]),
        #             ),
        #         )

        #     self.float_data.pack(fill=tk.BOTH, expand=True)
        #     self.float_graphs += 1
        #     plt.plot(self.float_time, self.float_depth)
        #     plt.savefig(f"float_graph {self.float_graphs}.jpeg")
        #     plt.close()
        #     self.float_graph = Image.open(
        #         rf"{self.float_graphs}.jpeg"
        #     )
        #     self.float_graph = self.float_graph.resize((600, 400))
        #     self.float_graph = ImageTk.PhotoImage(self.float_graph)
        #     self.float_canvas.delete("all")
        #     self.float_canvas.create_image(0, 0, image=self.float_graph, anchor="nw")

        #     self.start_flag = False

    def imu_callback(self, imu_msg):


        orientation_list = [
            imu_msg.orientation.x,
            imu_msg.orientation.y,
            imu_msg.orientation.z,
            imu_msg.orientation.w,
        ]
        
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion(orientation_list)
        self.roll = round(self.roll, 1)
        self.pitch = round(self.pitch, 1)
        self.yaw = round(self.yaw, 1)
        self.yaw *= -1
        self.pitch *= -1

        self.copilot_obj.roll_label.config(text=str(self.roll))
        self.copilot_obj.pitch_label.config(text=str(self.pitch))
        self.copilot_obj.yaw_label.config(text=str(self.yaw))


class VerticalScrolledFrame(tk.Frame):
    def __init__(self, parent, *args, **kwargs):
        tk.Frame.__init__(self, parent, *args, **kwargs)
        self.configure(bg="")

        # Create a scrollbar
        vscrollbar = tk.Scrollbar(self, orient=tk.VERTICAL)
        vscrollbar.pack(fill=tk.Y, side=tk.RIGHT, expand=tk.FALSE)

        # Create a canvas within the frame
        canvas = tk.Canvas(
            self, bd=0, highlightthickness=0, yscrollcommand=vscrollbar.set
        )
        canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=tk.TRUE)

        vscrollbar.config(command=canvas.yview)

        # Reset the canvas view
        canvas.xview_moveto(0)
        canvas.yview_moveto(0)

        # Create an interior frame to be scrolled
        self.interior = interior = tk.Frame(canvas)
        interior_id = canvas.create_window(0, 0, window=interior, anchor=tk.NW)

        # Bind the canvas to the scrollbar
        def _configure_interior(event):
            size = (interior.winfo_reqwidth(), interior.winfo_reqheight())
            canvas.config(scrollregion="0 0 %s %s" % size)
            if interior.winfo_reqwidth() != canvas.winfo_width():
                canvas.config(width=interior.winfo_reqwidth())

        interior.bind("<Configure>", _configure_interior)

        def _configure_canvas(event):
            if interior.winfo_reqwidth() != canvas.winfo_width():
                canvas.itemconfigure(interior_id, width=canvas.winfo_width())

        canvas.bind("<Configure>", _configure_canvas)


def getScreensInfo(screen):
    monitors = get_monitors()
    if len(monitors) > 1 and screen == 2:
        second_screen = monitors[1]
        return (
            second_screen.width,
            second_screen.height,
            second_screen.x,
            second_screen.y,
        )
    else:
        first_screen = monitors[0]
        return first_screen.width, first_screen.height, 0, 0




def ros_init(copilot_obj):
    rclpy.init(args=None)
    node = CoPilotNode(copilot_obj)
    rclpy.spin(node)
    rclpy.shutdown()


def init_copilot():
    window = tk.Tk()
    copilot_interface = CoPilot(window, getScreensInfo(1))

    ros_thread = threading.Thread(target=ros_init, args = (copilot_interface, ))
    ros_thread.start()

    window.mainloop()


def main():
    for i in range(len(IPS)):
        process = Process(
            target=displayCamera,
            args=(list(IPS.values())[i], NAMES[i], SIZE[i], POS[i], getScreensInfo(1)),
        )
        PROCESSES[i] = process
        process.start()
    init_copilot()

if __name__ == "__main__":
    main()
