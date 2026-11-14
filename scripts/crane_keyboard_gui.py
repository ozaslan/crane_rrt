#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import tkinter as tk
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class CraneKeyboardGUI(Node):
    def __init__(self):
        super().__init__("crane_keyboard_gui_node")
        self.publisher = self.create_publisher(Float64MultiArray, "/gantry_force_input", 10)

        self.root = tk.Tk()
        self.root.title("3D Gantry Crane Force Control")
        self.root.resizable(False, False)

        top = tk.Frame(self.root, padx=12, pady=8)
        top.grid(row=0, column=0, columnspan=3)
        tk.Label(top, text="Force Magnitude [N]:").pack(side=tk.LEFT, padx=4)
        self.force_var = tk.DoubleVar(value=0.50)
        tk.Spinbox(
            top,
            from_=0.05,
            to=2.00,
            increment=0.05,
            textvariable=self.force_var,
            width=6,
            format="%.2f",
        ).pack(side=tk.LEFT, padx=4)

        self._make_btn("↑  +Fy", 1, 1, lambda: self.send_scaled(0.0, 1.0))
        self._make_btn("←  -Fx", 2, 0, lambda: self.send_scaled(-1.0, 0.0))
        self._make_btn("■  Zero", 2, 1, lambda: self.send_force(0.0, 0.0))
        self._make_btn("→  +Fx", 2, 2, lambda: self.send_scaled(1.0, 0.0))
        self._make_btn("↓  -Fy", 3, 1, lambda: self.send_scaled(0.0, -1.0))

        self.lbl_status = tk.Label(self.root, text="Current Force: (0.00, 0.00) N", pady=6)
        self.lbl_status.grid(row=4, column=0, columnspan=3)

        self.root.bind("<Up>", lambda _e: self.send_scaled(0.0, 1.0))
        self.root.bind("<Down>", lambda _e: self.send_scaled(0.0, -1.0))
        self.root.bind("<Left>", lambda _e: self.send_scaled(-1.0, 0.0))
        self.root.bind("<Right>", lambda _e: self.send_scaled(1.0, 0.0))
        self.root.bind("<space>", lambda _e: self.send_force(0.0, 0.0))

        self.root.protocol("WM_DELETE_WINDOW", self.shutdown)
        self.root.mainloop()

    def _make_btn(self, text, row, col, command):
        tk.Button(self.root, text=text, width=10, height=2, command=command).grid(
            row=row, column=col, padx=8, pady=6
        )

    def send_scaled(self, sx: float, sy: float):
        try:
            mag = float(self.force_var.get())
        except (ValueError, tk.TclError):
            mag = 0.50
        self.send_force(sx * mag, sy * mag)

    def send_force(self, fx: float, fy: float):
        msg = Float64MultiArray()
        msg.data = [float(fx), float(fy)]
        self.publisher.publish(msg)
        self.lbl_status.config(text=f"Current Force: ({fx:+.2f}, {fy:+.2f}) N")
        self.get_logger().info(f"Published force input: Fx={fx:+.2f} N, Fy={fy:+.2f} N")

    def shutdown(self):
        self.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        self.root.destroy()


def main():
    rclpy.init()
    CraneKeyboardGUI()


if __name__ == "__main__":
    main()

