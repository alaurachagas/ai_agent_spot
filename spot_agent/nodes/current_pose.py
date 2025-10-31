# current_pose.py
import rclpy
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
import tf2_ros
from tf2_ros import LookupException

def get_current_pose(target_frame: str = "map", source_frame: str = "body", timeout_sec: float = 2.0) -> dict:
    """Return pose from TF: target_frame <- source_frame (e.g., map <- body)."""
    ctx = rclpy.Context()
    rclpy.init(context=ctx)
    node = rclpy.create_node("tmp_pose_reader", context=ctx)

    exec_ = SingleThreadedExecutor(context=ctx)
    exec_.add_node(node)

    try:
        buf = tf2_ros.Buffer()
        _listener = tf2_ros.TransformListener(buf, node)

        deadline = node.get_clock().now() + Duration(seconds=timeout_sec)
        while node.get_clock().now() < deadline:
            exec_.spin_once(timeout_sec=0.05)  # <-- use our executor, not the global one
            if buf.can_transform(target_frame, source_frame, Time()):
                break

        if not buf.can_transform(target_frame, source_frame, Time()):
            raise LookupException(f"Transform {target_frame} <- {source_frame} not available within {timeout_sec}s")

        t = buf.lookup_transform(target_frame, source_frame, Time())
        p, q = t.transform.translation, t.transform.rotation
        return {
            "position": {"x": float(p.x), "y": float(p.y), "z": float(p.z)},
            "orientation": {"x": float(q.x), "y": float(q.y), "z": float(q.z), "w": float(q.w)},
        }
    finally:
        exec_.remove_node(node)
        node.destroy_node()
        rclpy.shutdown(context=ctx)
