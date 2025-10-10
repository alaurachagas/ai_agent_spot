#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import tf2_ros
import json
import os
import sys
import time
from ament_index_python.packages import get_package_share_directory

class SaveLocation(Node):
    def __init__(self, temp_name='unknown'):
        super().__init__('save_location_node')
        self.temp_name = temp_name
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.done = False

    def save_location(self):
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                'map',
                'body',
                now,
                timeout=rclpy.duration.Duration(seconds=5.0)
            )

            position = trans.transform.translation
            orientation = trans.transform.rotation

            pose_data = {
                self.temp_name: {
                    "position": {
                        "x": position.x,
                        "y": position.y,
                        "z": position.z
                    },
                    "orientation": {
                        "x": orientation.x,
                        "y": orientation.y,
                        "z": orientation.z,
                        "w": orientation.w
                    }
                }
            }

            file_path = os.path.join(
                get_package_share_directory('spot_agent'),
                'saved_data',
                'saved_locations.json'
            )   
            if os.path.exists(file_path):
                with open(file_path, 'r') as f:
                    saved_data = json.load(f)
            else:
                saved_data = {}

            saved_data.update(pose_data)

            with open(file_path, 'w') as f:
                json.dump(saved_data, f, indent=2)

            self.get_logger().info(f"Location saved as '{self.temp_name}'.")

        except Exception as e:
            self.get_logger().error(f"Failed to save location: {str(e)}")

        # Mark done to exit the main loop
        self.done = True

def main():
    rclpy.init()

    temp_name = sys.argv[1] if len(sys.argv) > 1 else "unknown"
    node = SaveLocation(temp_name=temp_name)

    start_time = time.time()

    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=0.5)
        if time.time() - start_time > 1.0:  # after 1 second, save
            node.save_location()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
