import tkinter as tk
import rclpy
from std_msgs.msg import String

class MyApp:
    def __init__(self, master):
        self.master = master
        master.title("ROS2 Tkinter Label Update")

        self.label = tk.Label(master, text="Waiting for message...")
        self.label.pack()

        # Initialize ROS2 node
        rclpy.init()
        self.node = rclpy.create_node('tkinter_label_node')

        # Subscribe to ROS2 topic
        self.subscriber = self.node.create_subscription(
            String, '/my_topic', self.callback, 10)

    def callback(self, msg):
        # Update label text with received message
        self.label.config(text=msg.data)

    def run(self):
        # Tkinter main loop
        self.master.mainloop()

        # Clean up ROS2 node
        self.node.destroy_node()
        rclpy.shutdown()

def main():
    root = tk.Tk()
    app = MyApp(root)
    app.run()

if __name__ == "__main__":
    main()
