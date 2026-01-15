#!/usr/bin/env python3
import tkinter as tk
from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock

class KeyboardController(Node):
    def __init__(self):
        super().__init__('keyboard_controller')
        self.publisher_ = self.create_publisher(TwistStamped, '/robot1/project_controller/cmd_vel', 10)
        self.clock = Clock()
        
        self.root = tk.Tk()
        self.root.title("WASD Controller (TwistStamped)")
        
        self.linear_speed = 0.5  
        self.angular_speed = 0.8 
        
        self.keys_pressed = {
            'w': False, 
            'a': False, 
            's': False, 
            'd': False,
            'space': False
        }
        
        self.setup_ui()
        self.setup_key_bindings()
        
        self.update_timer = self.root.after(100, self.update_velocity)

    def setup_ui(self):
        frame = tk.Frame(self.root, padx=20, pady=20)
        frame.pack()
        
        tk.Label(frame, 
                text="WASD Controller (TwistStamped)", 
                font=('Arial', 14, 'bold')).grid(row=0, columnspan=3, pady=10)
        
        controls = [
            ("W", "Forward"),
            ("A", "Rotate Left"),
            ("S", "Backward"),
            ("D", "Rotate Right"),
            ("SPACE", "Emergency Stop")
        ]
        
        for i, (key, desc) in enumerate(controls):
            tk.Label(frame, text=f"{key}:", font=('Arial', 10, 'bold')).grid(row=i+1, column=0, sticky='e', padx=5)
            tk.Label(frame, text=desc).grid(row=i+1, column=1, sticky='w')

    def setup_key_bindings(self):
        self.root.bind('<KeyPress>', self.on_key_press)
        self.root.bind('<KeyRelease>', self.on_key_release)
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def on_key_press(self, event):
        key = event.keysym.lower()
        if key in self.keys_pressed:
            self.keys_pressed[key] = True
        if event.keysym == 'space':
            self.keys_pressed['space'] = True
            self.emergency_stop()

    def on_key_release(self, event):
        key = event.keysym.lower()
        if key in self.keys_pressed:
            self.keys_pressed[key] = False

    def update_velocity(self):
        msg = TwistStamped()
        
        msg.header.stamp = self.clock.now().to_msg()
        msg.header.frame_id = "base_link" 
        
        if self.keys_pressed['w'] and not self.keys_pressed['s']:
            msg.twist.linear.x = self.linear_speed
        elif self.keys_pressed['s'] and not self.keys_pressed['w']:
            msg.twist.linear.x = -self.linear_speed
        else:
            msg.twist.linear.x = 0.0
        
        if self.keys_pressed['a'] and not self.keys_pressed['d']:
            msg.twist.angular.z = self.angular_speed
        elif self.keys_pressed['d'] and not self.keys_pressed['a']:
            msg.twist.angular.z = -self.angular_speed
        else:
            msg.twist.angular.z = 0.0
            
        self.publisher_.publish(msg)
        self.update_timer = self.root.after(100, self.update_velocity)

    def emergency_stop(self):
        msg = TwistStamped()
        msg.header.stamp = self.clock.now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.linear.x = 0.0
        msg.twist.angular.z = 0.0
        self.publisher_.publish(msg)
        
        for key in self.keys_pressed:
            self.keys_pressed[key] = False

    def on_close(self):
        self.emergency_stop()
        self.root.destroy()
        self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    controller = KeyboardController()
    
    try:
        controller.root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()