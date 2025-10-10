import json
import os
import subprocess

from langchain_core.tools import tool
from ament_index_python.packages import get_package_share_directory
from ..nodes.quarternion import calculate_move_forward_pose, calculate_turn_pose
from ..nodes.current_pose import get_current_pose
from ..nodes.pub_n_sub import publish_to


@tool
def move_to_goal(goal_location: str="home"):
    """
    Navigates the robot to a saved goal pose.
    If none location is given, list all saved locations
    This function expects a named location.
    """
    file_path = os.path.join(
            get_package_share_directory('spot_agent'),
            'saved_data',
            'saved_locations.json'
        )

    if not os.path.exists(file_path):
        return "Saved locations file not found."

    with open(file_path, 'r') as f:
        saved_data = json.load(f)

    if goal_location == "" or goal_location.lower() == "list":
        # Just list all saved locations
        if not saved_data:
            return "No locations saved yet."
        locations = "\n".join(f"- {loc}" for loc in saved_data.keys())
        return f"Saved Locations:\n{locations}"

    if goal_location not in saved_data:
        return f"Location '{goal_location}' not found in saved locations. Use 'go_to_location' without name to list available."

    new_pose = saved_data[goal_location]
    publish_to("geometry_msgs/msg/PoseStamped","/goal_pose",new_pose)


@tool
def save_location_tool(name: str = "unknown") -> str:
    """
    Save the current robot pose into a JSON file.
    This function expects the name of the location to be saved.
    """
    try:
        result = subprocess.run(
            ["ros2", "run", "spot_agent", "save_location", name],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=30,
            text=True
        )

        if result.returncode == 0:
            return f"Location saved successfully as '{name}'.\n{result.stdout}"
        else:
            return f"Failed to save location.\nError:\n{result.stderr}"

    except Exception as e:
        return f"Exception while saving location: {str(e)}"
    
@tool
def turn_robot(angle: float = 0.0):
    """
    Turns the robot on its exis.
    This function expects the angle in degrees to turn.
    """
    current_pose = get_current_pose()
    new_pose = calculate_turn_pose(current_pose)
    publish_to("geometry_msgs/msg/PoseStamped","/goal_pose",new_pose)

@tool
def walk_forward_robot(dist: float = 0.0):
    """
    Walk in a straght line a defined distance
    This function expects the distance in meters to walk
    """
    current_pose = get_current_pose()
    new_pose =calculate_move_forward_pose(current_pose)
    publish_to("geometry_msgs/msg/PoseStamped","/goal_pose",new_pose)

def tools_nav ():
    return [
        walk_forward_robot,
        turn_robot,
        save_location_tool,
        move_to_goal
    ]
    

