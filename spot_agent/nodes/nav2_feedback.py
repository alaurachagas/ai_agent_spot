#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped


class Nav2BlockingClient(Node):
    """
    Minimal Nav2 action client that:
      - sends a NavigateToPose goal
      - waits synchronously for the result
      - exposes some feedback info in a simple dict
    """

    def __init__(self):
        super().__init__('nav2_blocking_client')
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._latest_feedback = None

    def _feedback_callback(self, feedback_msg):
        # feedback_msg: NavigateToPose.FeedbackMessage
        self._latest_feedback = feedback_msg.feedback

    def go_to_pose_and_wait(self, goal_pose: PoseStamped, timeout_sec: float = 600.0):
        """
        Send a goal and block until Nav2 finishes or timeout.

        Returns a JSON-serializable dict, e.g.:
        {
          "status": "SUCCEEDED" | "ABORTED" | "CANCELED" | "REJECTED" | "ERROR",
          "navigation_time_sec": 12.34,
          "distance_remaining": 0.0,
          "number_of_recoveries": 0,
          "error_msg": ""
        }
        """

        # Wait for Nav2 action server
        if not self._client.wait_for_server(timeout_sec=5.0):
            return {
                "status": "ERROR",
                "error_msg": "Nav2 action server 'navigate_to_pose' not available",
                "navigation_time_sec": None,
                "distance_remaining": None,
                "number_of_recoveries": None,
            }

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        goal_msg.behavior_tree = ""  # use default BT

        # Send goal asynchronously, but we'll block on the future
        send_future = self._client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback,
        )

        # Block until goal is accepted or rejected
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=timeout_sec)
        goal_handle = send_future.result()

        if goal_handle is None:
            return {
                "status": "ERROR",
                "error_msg": "Failed to get goal_handle (timeout or error).",
                "navigation_time_sec": None,
                "distance_remaining": None,
                "number_of_recoveries": None,
            }

        if not goal_handle.accepted:
            return {
                "status": "REJECTED",
                "error_msg": "Goal was rejected by Nav2.",
                "navigation_time_sec": None,
                "distance_remaining": None,
                "number_of_recoveries": None,
            }

        # Goal accepted: wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout_sec)

        wrapped_result = result_future.result()
        if wrapped_result is None:
            return {
                "status": "ERROR",
                "error_msg": "No result from Nav2 (timeout or error).",
                "navigation_time_sec": None,
                "distance_remaining": None,
                "number_of_recoveries": None,
            }

        status_code = wrapped_result.status

        status_map = {
            GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
            GoalStatus.STATUS_ABORTED: "ABORTED",
            GoalStatus.STATUS_CANCELED: "CANCELED",
        }
        status_str = status_map.get(status_code, f"UNKNOWN({status_code})")

        fb = self._latest_feedback
        if fb is not None:
            nav_time = fb.navigation_time.sec + fb.navigation_time.nanosec * 1e-9
            distance_remaining = fb.distance_remaining
            number_of_recoveries = fb.number_of_recoveries
        else:
            nav_time = None
            distance_remaining = None
            number_of_recoveries = None

        return {
            "status": status_str,
            "error_msg": "",
            "navigation_time_sec": nav_time,
            "distance_remaining": distance_remaining,
            "number_of_recoveries": number_of_recoveries,
        }


def navigate_to_pose_blocking(pose: PoseStamped, timeout_sec: float = 600.0):
    """
    Convenience function: run one navigation, shut down.

    This is nice for your agent tools that just want a blocking call.
    """
    try:
        node = Nav2BlockingClient()
        result = node.go_to_pose_and_wait(pose, timeout_sec)
    finally:
        node.destroy_node()
    return result

