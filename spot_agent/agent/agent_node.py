import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from std_msgs.msg import String

from langchain.agents import create_agent
from langchain_core.messages import HumanMessage

from ..prompts.get_prompts import get_prompts
from .llm import get_ollama_model
from ..tools.tools_spot import get_tools
from ..prompts.system_prompts import generic_system_prompts

class Agent(Node):
    def __init__(self):
        super().__init__("Spot_Agent")
        qos_profile = QoSProfile(depth=1, durability=QoSDurabilityPolicy.VOLATILE)
        
        self.subscription = self.create_subscription(
            String,
            "transcription_text",
            self.agent_callback ,
            qos_profile
        )

        # Fix to tools error
        my_tools=get_tools()
        my_model=get_ollama_model()
        my_prompts=get_prompts(generic_system_prompts)
        # Instantiate the Agent
        self.agent = create_agent(
        model=my_model,
        system_prompt=my_prompts,
        tools=my_tools
        )
        
        self.get_logger().info("Spot Agent initialised and waiting for messages ...")
        self.message_recieved = False

        
    def agent_callback (self, msg):
        """Callback function to process recieved transcription text"""
        self.get_logger().info("Waiting for message to be published ...")
        
        # Wait for message to be published
        if not self.message_recieved:
            self.message_recieved = True
        
        # Logger
        self.get_logger().info(f"Recieved message: {msg.data}")
        
        # Invoke the agent and handle the response
        try:
            self.get_logger().info(f"Invoking Agent with the query: {msg.data}")
            response = self.agent.invoke({"messages": [HumanMessage(content=msg.data)]})
            self.get_logger().info(f"Agent Response: {response}")
        except Exception as e:
            self.get_logger().error(f"Error executing tool: {e}")
        finally:
            self.message_recieved = False
            self.get_logger().info("Waiting for the next message...\n")


def main ():
    rclpy.init()
    spot_node = Agent()
    try:
        rclpy.spin(spot_node)
    except KeyboardInterrupt:
        spot_node.get_logger().info("Keyboard interrupt received, shutting down...")
    finally:
        spot_node.destroy_node()
        rclpy.shutdown()
    
    
if __name__ == "__main__":
    main()
