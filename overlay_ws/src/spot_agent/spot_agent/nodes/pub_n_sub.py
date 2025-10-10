import rclpy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Bool

from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from std_msgs.msg import Float64MultiArray, String
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState


class AgentPublisher(Node):
    def __init__(self, topic: str, type):
        super().__init__("Agent_Publisher")
        self.topic = topic
        self.type = type
        self.publisher = self.create_publisher(self.type, self.topic, 100)
        self.get_logger().info("Agent Publisher initialised...")
        
    def publish_callback (self, data):
        msg = self.type()
        self.publisher.publish(msg)
        if self.type == Float64MultiArray or self.type == Bool:
            msg.data = data

        elif self.type == PoseStamped:
            # Expecting a dictionary with position/orientation
            msg.header.frame_id = data.get("frame_id", "map")
            msg.header.stamp = self.get_clock().now().to_msg()

            pos = data["position"]
            ori = data["orientation"]

            msg.pose.position.x = pos["x"]
            msg.pose.position.y = pos["y"]
            msg.pose.position.z = pos["z"]

            msg.pose.orientation.x = ori["x"]
            msg.pose.orientation.y = ori["y"]
            msg.pose.orientation.z = ori["z"]
            msg.pose.orientation.w = ori["w"]

        else:
            raise ValueError(f"Unsupported message type: {self.type}")
        
        self.publisher.publish(msg)
        self.get_logger().info(f"Published to {self.topic}: {msg}")


class AgentSubscriber(Node):
    def __init__(self, type, topic: str):
        super().__init__("Agent_Subscriber")
        qos_profile = QoSProfile(depth=1, durability=QoSDurabilityPolicy.VOLATILE)
        self.topic = topic
        self.type = type
        
        self.subscriber = self.create_subscription(self.type, self.topic, self.subscriber_callback, qos_profile=qos_profile)
        self.get_logger().info("Agent Subscriber Initialised...")
        self.recieved_message = None
        self.subscriber
        
    def subscriber_callback(self, msg):
        """
        Callback function to return the recieved message
        """
        self.get_logger().info("Waiting for tool query to be published")
        if self.type == String or self.type == Float64MultiArray:
            self.recieved_message = msg.data
        elif self.type == JointState:
            self.recieved_message = {
            "name": msg.name,
            "position": list(msg.position),
            "velocity": list(msg.velocity),
            "effort": list(msg.effort)
            }
        
        self.get_logger().info(f"Recieved Joint states: {self.recieved_message}")

        
            
    def get_message(self):
        """
        Getter function to return the recieved message
        """
        if self.recieved_message is None:
            raise ValueError("No message received. Please check the publisher.")
        msg = self.recieved_message
        
        if self.type == JointState:
            coordinates = []
            for i in range(1, 8):
                if f"joint_{i}" in msg["name"]:
                    joint_index = msg["name"].index(f"joint_{i}")
                    coordinates.append(msg["position"][joint_index])

            return coordinates
        
        self.recieved_message = None
        return msg

def publish_to(type_name, topic_name: str, pose: dict = None, coordinates: list = None, msg: bool = None) -> None:
    """
    Publishes Coordinates to MoveIt

    Args:
        pose (dict) = None : Desired pose to publish
        coordinates (list) = None :  Desired coordinates to publish
        msg (bool) = None : Desired Bool Message to publish
    """
    try:
        # Initialise the ROS2 Node to publish the coordinates
        publisher_node = AgentPublisher(type=type_name, topic=topic_name)
        if type_name == Float64MultiArray:
            publisher_node.publish_callback(coordinates)
        elif type_name == Bool:
            publisher_node.publish_callback(msg)
        elif type_name == PoseStamped:
            publisher_node.publish_callback(pose)
        
        # Spin the Node to keep it publishing
        rclpy.spin_once(publisher_node, timeout_sec=0.1)

        # Destroy Node
        publisher_node.destroy_node()
    except Exception as e:
        print(f"Error Publishing Coordinates to Robot: {e}")
        return
    finally:
        publisher_node.destroy_node()
    

def subscribe_to (topic_name: str, type_name):
    """
    Subscribe to a topic if available

    Args:
        topic (str): Topic name
        type_name : Type of the message to be recieved
    """
    
    try:
        # Initialise the ROS2 Subscriber
        subscriber_node = AgentSubscriber(type=type_name, topic=topic_name)
        
        # Spin the Node to keep it publishing
        rclpy.spin_once(subscriber_node, timeout_sec=0.1)
        
        # Return the message
        return subscriber_node.get_message()
    except Exception as e:
        print(f"Error subscribing to Topic: {e}")
    finally:
        subscriber_node.destroy_node()