import tkinter as tk
from utils import getScreensInfo

# from tkinter import ttk
from enum import Enum
from paramiko import SSHClient

CLIENT = SSHClient()
CLIENT.load_system_host_keys()

IP = "192.168.1.123"

AUTH = ("raspberry", "pi")

RESET_COMMAND = "source Desktop/camera_reset.bash"


class Window:
    def __init__(self, window, geometry):
        self.window = window
        self.geometry = geometry

        self.create_widgets()

    @staticmethod
    def send_command(command: str):
        CLIENT.connect(IP, username=AUTH[0], password=AUTH[1])
        stdin, stdout, stderr = CLIENT.exec_command(command)

        _ = [print(i.strip()) for i in stdout]

        CLIENT.close()

    def reset_cams(self):
        CLIENT.connect(IP, username=AUTH[0], password=AUTH[1])
        stdin, stdout, stderr = CLIENT.exec_command(RESET_COMMAND)

        _ = [print(i.strip()) for i in stdout]
        _ = [print(i.strip()) for i in stderr]


        CLIENT.close()

    def create_widgets(self):
        self.screen_width = self.geometry[0]
        self.screen_height = self.geometry[1]
        window_size = f"{self.screen_width}x{self.screen_height}+{self.geometry[2]}+{self.geometry[3]}"
        self.window.geometry(window_size)
        self.window.title("Camera Sliders")

        self.back_canvas = tk.Canvas(
            self.window, width=self.screen_width, height=self.screen_height
        )
        self.back_canvas.pack(fill="both", expand=True)

        # title_font = ("Garamond", 35, "bold")
        label_font = ("Garamond", 20, "bold")

        self.sliders_frame = tk.Frame(self.back_canvas, bd=2, relief="groove", bg="")
        self.sliders_frame.grid(row=0, column=0, padx=10, pady=10, sticky="ns")
        self.dev_0_brightness = tk.Label(
            self.sliders_frame,
            text="dev_0 brightness",
            font=label_font,
            bg="#ffffff",
        )

        self.dev_0_brightness.grid(
            row=0, column=0, padx=10, pady=20, sticky="nsew", columnspan=2
        )
        self.brightness_slider = tk.Scale(
            self.sliders_frame, orient="horizontal", from_=30, to=255
        )
        self.brightness_slider.grid(
            row=1, column=0, padx=10, pady=20, sticky="nsew", columnspan=2
        )
        self.brightness_slider.set(75)
        self.brightness_button = tk.Button(
            self.sliders_frame,
            text="Change Brightness",
            command=lambda: self.send_command(
                f"sudo v4l2-ctl -d /dev/video0 -c brightness={self.brightness_slider.get()}"
            ),
        )
        self.brightness_button.grid(
            row=2, column=0, padx=10, pady=20, sticky="nsew", columnspan=2
        )

        self.dev_2_brightness = tk.Label(
            self.sliders_frame,
            text="dev_2 brightness",
            font=label_font,
            bg="#ffffff",
        )

        self.dev_2_brightness.grid(
            row=3, column=0, padx=10, pady=20, sticky="nsew", columnspan=2
        )
        self.brightness_slider_2 = tk.Scale(
            self.sliders_frame, orient="horizontal", from_=30, to=255
        )
        self.brightness_slider_2.grid(
            row=4, column=0, padx=10, pady=20, sticky="nsew", columnspan=2
        )
        self.brightness_slider_2.set(75)
        self.brightness_button_2 = tk.Button(
            self.sliders_frame,
            text="Change Brightness",
            command=lambda: self.send_command(
                f"sudo v4l2-ctl -d /dev/video2 -c brightness={self.brightness_slider.get()}"
            ),
        )
        self.brightness_button_2.grid(
            row=5, column=0, padx=10, pady=20, sticky="nsew", columnspan=2
        )

        self.dev_4_brightness = tk.Label(
            self.sliders_frame,
            text="dev_4 brightness",
            font=label_font,
            bg="#ffffff",
        )

        self.dev_4_brightness.grid(
            row=6, column=0, padx=10, pady=20, sticky="nsew", columnspan=2
        )
        self.brightness_slider_4 = tk.Scale(
            self.sliders_frame, orient="horizontal", from_=-64, to=64
        )
        self.brightness_slider_4.grid(
            row=7, column=0, padx=10, pady=20, sticky="nsew", columnspan=2
        )
        self.brightness_slider_4.set(0)
        self.brightness_button_4 = tk.Button(
            self.sliders_frame,
            text="Change Brightness",
            command=lambda: self.send_command(
                f"sudo v4l2-ctl -d /dev/video4 -c brightness={self.brightness_slider.get()}"
            ),
        )
        self.brightness_button_4.grid(
            row=8, column=0, padx=10, pady=20, sticky="nsew", columnspan=2
        )

        self.dev_0_contrast = tk.Label(
            self.sliders_frame,
            text="dev_0 contrast",
            font=label_font,
            bg="#ffffff",
        )

        self.dev_0_contrast.grid(
            row=0, column=4, padx=10, pady=20, sticky="nsew", columnspan=2
        )
        self.contrast_slider = tk.Scale(
            self.sliders_frame, orient="horizontal", from_=0, to=10
        )
        self.contrast_slider.grid(
            row=1, column=4, padx=10, pady=20, sticky="nsew", columnspan=2
        )
        self.contrast_slider.set(5)
        self.contrast_button = tk.Button(
            self.sliders_frame,
            text="Change contrast",
            command=lambda: self.send_command(
                f"sudo v4l2-ctl -d /dev/video0 -c contrast={self.contrast_slider.get()}",
            ),
        )
        self.contrast_button.grid(
            row=2, column=4, padx=10, pady=20, sticky="nsew", columnspan=2
        )

        self.dev_2_contrast = tk.Label(
            self.sliders_frame,
            text="dev_2 contrast",
            font=label_font,
            bg="#ffffff",
        )

        self.dev_2_contrast.grid(
            row=3, column=4, padx=10, pady=20, sticky="nsew", columnspan=2
        )
        self.contrast_slider_2 = tk.Scale(
            self.sliders_frame, orient="horizontal", from_=0, to=10
        )
        self.contrast_slider_2.grid(
            row=4, column=4, padx=10, pady=20, sticky="nsew", columnspan=2
        )
        self.contrast_slider_2.set(5)
        self.contrast_button_2 = tk.Button(
            self.sliders_frame,
            text="Change contrast",
            command=lambda: self.send_command(
                f"sudo v4l2-ctl -d /dev/video2 -c contrast={self.contrast_slider_2.get()}"
            ),
        )
        self.contrast_button_2.grid(
            row=5, column=4, padx=10, pady=20, sticky="nsew", columnspan=2
        )

        self.dev_4_contrast = tk.Label(
            self.sliders_frame,
            text="dev_4 contrast",
            font=label_font,
            bg="#ffffff",
        )

        self.dev_4_contrast.grid(
            row=6, column=4, padx=10, pady=20, sticky="nsew", columnspan=2
        )
        self.contrast_slider_4 = tk.Scale(
            self.sliders_frame, orient="horizontal", from_=0, to=64
        )
        self.contrast_slider_4.grid(
            row=7, column=4, padx=10, pady=20, sticky="nsew", columnspan=2
        )
        self.contrast_slider_4.set(32)
        self.contrast_button_4 = tk.Button(
            self.sliders_frame,
            text="Change contrast",
            command=lambda: self.send_command(
                f"sudo v4l2-ctl -d /dev/video4 -c contrast={self.contrast_slider_4.get()}"
            ),
        )
        self.contrast_button_4.grid(
            row=8, column=4, padx=10, pady=20, sticky="nsew", columnspan=2
        )

        self.dev_0_backlight = tk.Label(
            self.sliders_frame,
            text="dev_0 backlight compensation",
            font=label_font,
            bg="#ffffff",
        )

        self.dev_0_backlight.grid(
            row=0, column=8, padx=10, pady=20, sticky="nsew", columnspan=2
        )
        self.backlight_slider = tk.Scale(
            self.sliders_frame, orient="horizontal", from_=0, to=10
        )
        self.backlight_slider.grid(
            row=1, column=8, padx=10, pady=20, sticky="nsew", columnspan=2
        )
        self.backlight_slider.set(0)
        self.backlight_button = tk.Button(
            self.sliders_frame,
            text="Change backlight compensation",
            command=lambda: self.send_command(
                f"sudo v4l2-ctl -d /dev/video0 -c backlight_compensation={self.contrast_slider.get()}",
            ),
        )
        self.backlight_button.grid(
            row=2, column=8, padx=10, pady=20, sticky="nsew", columnspan=2
        )

        self.dev_2_backlight = tk.Label(
            self.sliders_frame,
            text="dev_2 backlight compensation",
            font=label_font,
            bg="#ffffff",
        )

        self.dev_2_backlight.grid(
            row=3, column=8, padx=10, pady=20, sticky="nsew", columnspan=2
        )
        self.backlight_slider_2 = tk.Scale(
            self.sliders_frame, orient="horizontal", from_=0, to=10
        )
        self.backlight_slider_2.grid(
            row=4, column=8, padx=10, pady=20, sticky="nsew", columnspan=2
        )
        self.backlight_slider_2.set(0)
        self.backlight_button_2 = tk.Button(
            self.sliders_frame,
            text="Change backlight compensation",
            command=lambda: self.send_command(
                f"sudo v4l2-ctl -d /dev/video2 -c backlight_compensation={self.contrast_slider_2.get()}"
            ),
        )
        self.backlight_button_2.grid(
            row=5, column=8, padx=10, pady=20, sticky="nsew", columnspan=2
        )

        self.dev_4_contrast = tk.Label(
            self.sliders_frame,
            text="dev_4 backlight compensation",
            font=label_font,
            bg="#ffffff",
        )

        self.dev_4_contrast.grid(
            row=6, column=8, padx=10, pady=20, sticky="nsew", columnspan=2
        )
        self.backlight_slider_4 = tk.Scale(
            self.sliders_frame, orient="horizontal", from_=0, to=8
        )
        self.backlight_slider_4.grid(
            row=7, column=8, padx=10, pady=20, sticky="nsew", columnspan=2
        )
        self.backlight_slider_4.set(4)
        self.backlight_button_4 = tk.Button(
            self.sliders_frame,
            text="Change backlight compensation",
            command=lambda: self.send_command(
                f"sudo v4l2-ctl -d /dev/video4 -c backlight_compensation={self.contrast_slider_4.get()}"
            ),
        )
        self.backlight_button_4.grid(
            row=8, column=8, padx=10, pady=20, sticky="nsew", columnspan=2
        )

        self.reset_button = tk.Button(
            self.sliders_frame, text="Reset", command=self.reset_cams
        )
        self.reset_button.grid(
            row=8, column=12, padx=10, pady=20, sticky="nsew", columnspan=2
        )


def main():
    window = tk.Tk()
    Window(window, getScreensInfo(0))
    window.mainloop()


if __name__ == "__main__":
    main()
