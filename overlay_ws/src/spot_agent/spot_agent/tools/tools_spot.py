from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from langchain import Tool

def make_send_nav_goal_tool(agent: Node) -> Tool:
    def send_nav_goal(x: float, y: float) -> str:
        # Publish to /spot/goal_pose, etc.
        goal = PoseStamped()
        goal.header.stamp = agent.get_clock().now().to_msg()
        goal.header.frame_id = "map"
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.w = 1.0
        agent.nav_pub.publish(goal)
        return f"Sent navigation goal to ({x:.2f}, {y:.2f})"
    return Tool(
        name="send_nav_goal",
        func=send_nav_goal,
        description="Send a 2D navigation goal (x, y) to Spot via Nav2."
    )

def get_tools() -> list[Tool]:
    # Instantiate your SpotAgent node here if needed, or assume the agent
    # will pass itself in. For simplicity, we assume `agent` is global:
    from spot_agent.agent import SpotAgent
    # (In practice, you might want to pass `self` into the tool factory.)
    # But LangChain Tools need a callable without arguments matching the signature, so
    # you can wrap the agent methods at runtime instead of here.
    return [
        make_send_nav_goal_tool(SpotAgent._dummy_reference_),  # see note below
        # ... other tools ...
    ]
