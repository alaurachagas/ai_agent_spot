import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped
import numpy as np
import matplotlib.pyplot as plt
import time

def find_frontiers_loose(occupancy_grid, max_value_for_frontier=50):
    """
    Function to detect the frontiers
    """
    frontier_mask = (occupancy_grid == -1) | ((occupancy_grid >= 1) & (occupancy_grid <= max_value_for_frontier))
    return np.argwhere(frontier_mask)

class FrontierSelector(Node):
    """
    Class to explore the frontiers and generate a map
    """
    def __init__(self):
        super().__init__('frontier_selector_loop')

        self.subscription_map = self.create_subscription(
            OccupancyGrid,
            'SOMETHING/map', # TODO: Declare map topic
            self.map_callback,
            10
        )
        self.subscription_odom = self.create_subscription(
            Odometry,
            'SOMOTHING/odom', # TODO: Declare odom topic
            self.odom_callback,
            10
        )
        self.goal_pub = self.create_publisher(PoseStamped, 'SOMETHING/goal_pose', 10) # TODO: Declare goal pose topic

        self.map_received = False
        self.odom_received = False
        self.map_data = None
        self.map_origin = None
        self.map_resolution = None
        self.robot_position = None
        self.goal_position = None

    def map_callback(self, msg):
        height = msg.info.height
        width = msg.info.width
        self.map_resolution = msg.info.resolution
        self.map_origin = msg.info.origin.position
        data_raw = np.array(msg.data, dtype=np.int8).reshape((height, width))
        self.map_data = np.flipud(np.where((data_raw > 0) & (data_raw < 100), 100, data_raw))
        self.map_received = True

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        self.robot_position = (pos.x, pos.y)
        self.odom_received = True

    def map_to_grid(self, x, y):
        j = int((x - self.map_origin.x) / self.map_resolution)
        i = int((y - self.map_origin.y) / self.map_resolution)
        i_flipped = self.map_data.shape[0] - i
        return i_flipped, j

    def grid_to_map(self, i, j):
        i_unflipped = self.map_data.shape[0] - i
        y = i_unflipped * self.map_resolution + self.map_origin.y
        x = j * self.map_resolution + self.map_origin.x
        return x, y

    def select_best_frontier(self, frontiers, min_distance=2.0):
        if not frontiers.any():
            return None, None

        robot_x, robot_y = self.robot_position
        rx, ry = self.map_to_grid(robot_x, robot_y)

        clusters = {}
        for f in frontiers:
            key = (f[0] // 3, f[1] // 3)
            clusters.setdefault(key, []).append(f)

        sorted_clusters = sorted(
            clusters.values(),
            key=lambda c: (-len(c), np.linalg.norm([c[0][0] - rx, c[0][1] - ry]))
        )

        for cluster in sorted_clusters:
            f = cluster[0]
            x, y = self.grid_to_map(f[0], f[1])
            if np.linalg.norm([x - robot_x, y - robot_y]) >= min_distance:
                return f, (x, y)

        return None, None

    def publish_goal(self, x, y):
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.w = 1.0
        self.goal_position = (x, y)
        self.goal_pub.publish(goal)
        print(f"🚀 Published goal: ({x:.2f}, {y:.2f})")

    def goal_reached(self, tolerance=0.4):
        if self.robot_position is None or self.goal_position is None:
            return False
        dist = np.linalg.norm(np.array(self.robot_position) - np.array(self.goal_position))
        return dist < tolerance

# ===== Main Loop =====
def main():
    rclpy.init()
    node = FrontierSelector()

    # Wait for first map + odom
    print("⏳ Waiting for map and odometry...")
    start_time = time.time()
    while rclpy.ok() and (not node.map_received or not node.odom_received):
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.time() - start_time > 20:
            print("❌ Timeout waiting for initial data.")
            rclpy.shutdown()
            return

    # Optional: Show initial plot
    frontiers = find_frontiers_loose(node.map_data)
    grid_robot = node.map_to_grid(*node.robot_position)
    best_f, best_coords = node.select_best_frontier(frontiers)

    plt.figure(figsize=(10, 10))
    data_visual = np.where(node.map_data == -1, 128, node.map_data)
    plt.imshow(data_visual, cmap='gray', origin='lower')
    plt.scatter(frontiers[:, 1], frontiers[:, 0], s=10, c='red', label='Frontiers')
    plt.scatter(grid_robot[1], grid_robot[0], s=50, c='blue', marker='x', label='Robot')
    if best_f is not None:
        plt.scatter(best_f[1], best_f[0], s=100, c='yellow', marker='*', label='Next Best Frontier')
    plt.legend()
    plt.title("Initial Frontier Selection")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.show()

    # === Loop forever: Navigate to frontiers ===
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.2)

        if not node.map_received or not node.odom_received:
            continue

        frontiers = find_frontiers_loose(node.map_data)
        best_f, best_coords = node.select_best_frontier(frontiers)

        if best_coords is None:
            print("⚠️ No frontier found 2m away. Stopping.")
            break

        node.publish_goal(best_coords[0], best_coords[1])

        # Wait until robot reaches the goal
        print("🕒 Navigating to goal...")
        goal_start = time.time()
        while rclpy.ok() and not node.goal_reached():
            rclpy.spin_once(node, timeout_sec=0.2)
            # Optional: timeout if stuck
            if time.time() - goal_start > 90:
                print("⚠️ Timeout reaching goal. Moving to next.")
                break

        print("✅ Goal reached or timed out. Searching next...")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()