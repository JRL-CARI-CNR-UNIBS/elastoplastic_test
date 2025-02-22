import tkinter as tk
from tkinter import ttk
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
import threading

import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from geometry_msgs.msg import TransformStamped

import numpy as np



class WrenchPublisherNode(Node):
    def __init__(self):
        super().__init__('wrench_publisher_node')
        # Create a publisher for WrenchStamped messages
        self.publisher_ = self.create_publisher(WrenchStamped, '/io_and_status_controller/fake_wrench', 10)

        self.spin_node = Node('__spin_node__')

        # Create a tf2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.spin_node, spin_thread=True)

        self.get_logger().info('WrenchPublisherNode has been started.')

    def transform_force_to_tool(self, force_x, force_y, force_z):
        try:
            # Look up the transform from 'base_link' to 'tool'
            transform = self.tf_buffer.lookup_transform('demo/robotiq_ft_frame_id', 'map', rclpy.time.Time())
            # Extract rotation (quaternion) and translation from the transform
            q = transform.transform.rotation
            translation = transform.transform.translation

            # Convert quaternion to rotation matrix
            rotation_matrix = self.quaternion_to_rotation_matrix(q)

            # Apply the transformation
            force_base = np.array([force_x, force_y, force_z])
            force_tool = np.dot(rotation_matrix, force_base)

            return force_tool[0], force_tool[1], force_tool[2]
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().error(f"Could not transform force: {e}")
            return 0.0, 0.0, 0.0  # Fallback to base_link frame

    @staticmethod
    def quaternion_to_rotation_matrix(q):
        # Convert quaternion (x, y, z, w) to a rotation matrix
        x, y, z, w = q.x, q.y, q.z, q.w
        return np.array([
            [1 - 2 * (y ** 2 + z ** 2), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x ** 2 + z ** 2), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x ** 2 + y ** 2)]
        ])

    def publish_wrench(self, force_x=0.0, force_y=0.0, force_z=0.0):
        # Transform force to 'tool' frame
        force_x, force_y, force_z = self.transform_force_to_tool(force_x, force_y, force_z)

        # Create and populate the WrenchStamped message
        msg = WrenchStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "tool"
        msg.wrench.force.x = force_x
        msg.wrench.force.y = force_y
        msg.wrench.force.z = force_z
        # Optionally, set torque to zero or as needed
        msg.wrench.torque.x = 0.0
        msg.wrench.torque.y = 0.0
        msg.wrench.torque.z = 0.0
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published WrenchStamped in tool frame: x={force_x}, y={force_y}, z={force_z}')



class WrenchPublisherGUI:
    def __init__(self):
        # Initialize ROS2
        rclpy.init()

        # Create the ROS2 node
        self.node = WrenchPublisherNode()

        # Start ROS2 spinning in a separate thread
        self.ros_thread = threading.Thread(target=self.ros_spin, daemon=True)
        self.ros_thread.start()

        # Initialize the Tkinter GUI
        self.root = tk.Tk()
        self.root.title("ROS2 Wrench Publisher GUI")
        self.root.geometry("250x450")  # Width x Height

        # Prevent window resizing for fixed layout (optional)
        # self.root.resizable(False, False)

        # Configure grid layout with padding
        self.root.columnconfigure(0, weight=1)
        # Define rows: 0 - slider, 1 - directional buttons, 2 - Z buttons, 3 - entry
        self.root.rowconfigure(1, weight=1)  # Middle sections will expand if window resizes
        self.root.rowconfigure(2, weight=0)
        self.root.rowconfigure(3, weight=0)
        self.root.rowconfigure(4, weight=0)

        # Setup GUI components
        self.setup_slider()
        self.setup_directional_buttons()
        self.setup_z_buttons()
        self.setup_entry_field()

        # Bind the closing event
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

    def ros_spin(self):
        """
        Spins the ROS2 node. This function runs in a separate thread.
        """
        rclpy.spin(self.node)

    def setup_slider(self):
        # Frame for the slider and its label
        slider_frame = ttk.Frame(self.root)
        slider_frame.grid(row=0, column=0, padx=10, pady=10, sticky='ew')

        # Configure grid for slider_frame
        slider_frame.columnconfigure(0, weight=1)
        slider_frame.columnconfigure(1, weight=0)

        # Slider at the top
        self.slider = ttk.Scale(
            slider_frame,
            from_=0,
            to=30,
            orient='horizontal',
            command=lambda val: self.update_slider_label(val)
        )
        self.slider.set(0.5)  # Set default value
        self.slider.grid(row=0, column=0, sticky='ew')

        # Label to display slider value
        self.slider_value_label = ttk.Label(slider_frame, text=f"{self.slider.get():.2f}")
        self.slider_value_label.grid(row=0, column=1, padx=(5, 0))

    def setup_directional_buttons(self):
        # Frame for the star-shaped directional buttons
        button_frame = ttk.Frame(self.root)
        button_frame.grid(row=1, column=0, padx=10, pady=5)

        # Configure grid for button_frame
        for i in range(3):
            button_frame.columnconfigure(i, weight=1)
            button_frame.rowconfigure(i, weight=1)

        # Define buttons
        self.buttons = {}
        directions = ["Up", "Left", "Right", "Down"]
        positions = {
            "Up": (0, 1),
            "Left": (1, 0),
            "Right": (1, 2),
            "Down": (2, 1)
        }

        for direction in directions:
            btn = ttk.Button(button_frame, text=direction, width=6)
            btn.grid(row=positions[direction][0], column=positions[direction][1], pady=2, padx=2)
            btn.bind("<ButtonPress-1>", self.on_button_press)
            btn.bind("<ButtonRelease-1>", self.on_button_release)
            self.buttons[direction] = btn

    def setup_z_buttons(self):
        # Frame for the "UP-Z" and "DOWN-Z" buttons
        z_button_frame = ttk.Frame(self.root)
        z_button_frame.grid(row=2, column=0, padx=10, pady=5, sticky='ew')

        # Configure grid for z_button_frame
        z_button_frame.columnconfigure(0, weight=1)
        z_button_frame.columnconfigure(1, weight=1)

        # Define Z-direction buttons
        z_directions = ["UP-Z", "DOWN-Z"]
        for idx, z_dir in enumerate(z_directions):
            btn = ttk.Button(z_button_frame, text=z_dir, width=6)
            btn.grid(row=0, column=idx, padx=(0 if idx == 0 else 5, 0), pady=2, sticky='ew')
            btn.bind("<ButtonPress-1>", self.on_button_press)
            btn.bind("<ButtonRelease-1>", self.on_button_release)
            self.buttons[z_dir] = btn

    def setup_entry_field(self):
        # Frame for the text field and confirmation button
        entry_frame = ttk.Frame(self.root)
        entry_frame.grid(row=3, column=0, padx=10, pady=10, sticky='ew')

        # Configure grid for entry_frame
        entry_frame.columnconfigure(0, weight=1)
        entry_frame.columnconfigure(1, weight=0)

        # Text field (Entry)
        self.text_entry = ttk.Entry(entry_frame)
        self.text_entry.grid(row=0, column=0, padx=(0, 5), sticky='ew')

        # Confirmation Button
        confirm_button = ttk.Button(
            entry_frame,
            text="Confirm",
            command=self.confirm_action
        )
        confirm_button.grid(row=0, column=1)

    def on_button_press(self, event):
        """
        Handles button press events to publish corresponding force values.
        """
        button_name = event.widget['text']
        force_x, force_y, force_z = self.get_force_values(button_name)
        self.node.publish_wrench(force_x, force_y, force_z)

    def on_button_release(self, event):
        """
        Handles button release events to publish empty force values.
        """
        self.node.publish_wrench(0.0, 0.0, 0.0)

    def get_force_values(self, button_name):
        """
        Maps button names to their corresponding force values.
        """
        force_x, force_y, force_z = 0.0, 0.0, 0.0
        if button_name == "Up":
            force_y = 1.0 * self.slider.get()
        elif button_name == "Down":
            force_y = -1.0 * self.slider.get()
        elif button_name == "Left":
            force_x = -1.0 * self.slider.get()
        elif button_name == "Right":
            force_x = 1.0 * self.slider.get()
        elif button_name == "UP-Z":
            force_z = 1.0 * self.slider.get()
        elif button_name == "DOWN-Z":
            force_z = -1.0 * self.slider.get()
        return force_x, force_y, force_z

    def update_slider_label(self, value):
        """
        Updates the slider's value label in real-time.
        """
        self.slider_value_label.config(text=f"{float(value):.2f}")

    def confirm_action(self):
        """
        Handles the confirmation action when the "Confirm" button is clicked.
        """
        text = self.text_entry.get()
        print(f"Confirmed: {text}")

    def on_closing(self):
        """
        Ensures graceful shutdown of ROS2 and GUI when the window is closed.
        """
        self.node.get_logger().info('Shutting down ROS2 node and exiting GUI.')
        rclpy.shutdown()
        self.root.destroy()

    def run(self):
        """
        Starts the Tkinter main loop.
        """
        self.root.mainloop()


def main():
    gui = WrenchPublisherGUI()
    gui.run()


if __name__ == "__main__":
    main()
