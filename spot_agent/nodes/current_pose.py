#!/usr/bin/env python3
import rclpy
from rclpy.time import Time
from rclpy.duration import Duration
import tf2_ros

def get_current_pose(target_frame: str = "map", source_frame: str = "body", timeout_sec: float = 1.0) -> dict:
    """
    Returns the current robot pose as a dict.
    Reads TF transform target_frame <- source_frame (default: map <- body).
    """
    # Create an isolated ROS 2 context so this works from any plain Python script
    ctx = rclpy.Context()
    rclpy.init(context=ctx)
    node = rclpy.create_node("tmp_pose_reader", context=ctx)

    try:
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer, node)

        # Spin until we get the transform or timeout
        trans = tf_buffer.lookup_transform(
            target_frame,
            source_frame,
            Time(),  # latest available
            timeout=Duration(seconds=timeout_sec),
        )

        pos = trans.transform.translation
        ori = trans.transform.rotation

        return {
            "position": {"x": float(pos.x), "y": float(pos.y), "z": float(pos.z)},
            "orientation": {
                "x": float(ori.x),
                "y": float(ori.y),
                "z": float(ori.z),
                "w": float(ori.w),
            },
        }

    finally:
        node.destroy_node()
        rclpy.shutdown(context=ctx)
