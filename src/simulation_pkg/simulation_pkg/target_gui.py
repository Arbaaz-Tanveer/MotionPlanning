#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

import tkinter as tk


class PosePublisher(Node):
    def __init__(self):
        super().__init__("pose_gui")
        self.pub = self.create_publisher(Float32MultiArray, "o1/decision_target_data", 10)
        self.get_logger().info("Pose GUI Publisher started")


class PoseGUI:
    def __init__(self, node: PosePublisher):
        self.node = node

        # --- Window ---
        self.window = tk.Tk()
        self.window.title("Position + Theta Sender")

        self.canvas_size = 400
        self.canvas = tk.Canvas(
            self.window,
            width=self.canvas_size,
            height=self.canvas_size,
            bg="white"
        )
        self.canvas.grid(row=0, column=0, columnspan=4)

        # Center cross lines
        self.canvas.create_line(self.canvas_size/2, 0,
                                self.canvas_size/2, self.canvas_size)
        self.canvas.create_line(0, self.canvas_size/2,
                                self.canvas_size, self.canvas_size/2)

        self.canvas.bind("<Button-1>", self.on_canvas_click)

        # --- X Y labels and entry boxes ---
        tk.Label(self.window, text="X:").grid(row=1, column=0)
        tk.Label(self.window, text="Y:").grid(row=1, column=1)
        tk.Label(self.window, text="Theta:").grid(row=1, column=2)

        self.x_entry = tk.Entry(self.window, width=7)
        self.y_entry = tk.Entry(self.window, width=7)
        self.t_slider = tk.Scale(self.window, from_=-180, to=180,
                                 orient=tk.HORIZONTAL, length=200)

        self.x_entry.grid(row=2, column=0)
        self.y_entry.grid(row=2, column=1)
        self.t_slider.grid(row=2, column=2, columnspan=2)

        # --- Send Button ---
        tk.Button(self.window, text="SEND", command=self.send_pose)\
            .grid(row=3, column=0, columnspan=4, pady=10)

        self.window.after(10, self.spin_ros)

    # Spin ROS inside Tk event loop
    def spin_ros(self):
        rclpy.spin_once(self.node, timeout_sec=0.01)
        self.window.after(10, self.spin_ros)

    # Handle canvas clicks → fill X, Y only
    def on_canvas_click(self, event):
        FIELD_WIDTH = 22.0   # meters (X direction)
        FIELD_HEIGHT = 14.0  # meters (Y direction)

        # Center-relative pixel coordinates
        px = event.x - self.canvas_size / 2
        py = self.canvas_size / 2 - event.y

        # Scaling
        scale_x = self.canvas_size / FIELD_WIDTH   # 400 / 22
        scale_y = self.canvas_size / FIELD_HEIGHT  # 400 / 14

        x = round(px / scale_x, 3)
        y = round(py / scale_y, 3)

        # Fill entries
        self.x_entry.delete(0, tk.END)
        self.x_entry.insert(0, str(x))

        self.y_entry.delete(0, tk.END)
        self.y_entry.insert(0, str(y))

    # Publish only when SEND is pressed
    def send_pose(self):
        try:
            x = float(self.x_entry.get())
            y = float(self.y_entry.get())
            t = float(self.t_slider.get())

            msg = Float32MultiArray()
            msg.data = [x, y, t]
            self.node.pub.publish(msg)
            print("Published:", msg.data)

        except ValueError:
            print("Invalid X/Y input")


def main():
    rclpy.init()
    node = PosePublisher()
    gui = PoseGUI(node)
    gui.window.mainloop()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
