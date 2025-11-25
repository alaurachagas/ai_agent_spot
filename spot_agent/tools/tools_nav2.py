import json
import os
import subprocess

from langchain_core.tools import tool
from ament_index_python.packages import get_package_share_directory
from ..nodes.quarternion import calculate_move_forward_pose, calculate_turn_pose, dict_to_pose_stamped
from ..nodes.current_pose import get_current_pose
from ..nodes.pub_n_sub import publish_to
from ..nodes.nav2_feedback import navigate_to_pose_blocking

simulation = False

@tool
def set_simulation():
    """
    Sets the robot to simulation mode.
    """
    global simulation
    simulation = True
    return "Robot set to simulation mode."


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

    new_pose = dict_to_pose_stamped(saved_data[goal_location])
    nav_result = navigate_to_pose_blocking(new_pose, timeout_sec=600.0)
    return {
        "command": "move_to_goal",
        "nav2_status": nav_result["status"],
        "navigation_time_sec": nav_result["navigation_time_sec"],
        "distance_remaining": nav_result["distance_remaining"],
        "recoveries": nav_result["number_of_recoveries"],
        "error_msg": nav_result["error_msg"],
    }


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
    if simulation:
        current_pose = get_current_pose("map", "base_link")
    else:
        current_pose = get_current_pose()
    new_pose = dict_to_pose_stamped(calculate_turn_pose(current_pose, angle))
    nav_result = navigate_to_pose_blocking(new_pose, timeout_sec=600.0)
    return {
        "command": "turn_robot",
        "requested_angle_deg": angle,
        "nav2_status": nav_result["status"],
        "navigation_time_sec": nav_result["navigation_time_sec"],
        "distance_remaining": nav_result["distance_remaining"],
        "recoveries": nav_result["number_of_recoveries"],
        "error_msg": nav_result["error_msg"],
    }

@tool
def walk_forward_robot(dist: float = 0.0):
    """
    Walk in a straght line a defined distance
    This function expects the distance in meters to walk
    """
    if simulation:
        current_pose = get_current_pose("map", "base_link")
    else:
        current_pose = get_current_pose()
    new_pose = dict_to_pose_stamped(calculate_move_forward_pose(current_pose, dist))
    nav_result = navigate_to_pose_blocking(new_pose, timeout_sec=600.0)
    return {
        "command": "walk_forward_robot",
        "requested_distance_meters": dist,
        "nav2_status": nav_result["status"],
        "navigation_time_sec": nav_result["navigation_time_sec"],
        "distance_remaining": nav_result["distance_remaining"],
        "recoveries": nav_result["number_of_recoveries"],
        "error_msg": nav_result["error_msg"],
    }

@tool
def get_sequence(seq_name: str = ""):
    """
    Retives a sequence of tool calls necessary to execute a sequence.
    This function expects the name of the sequence (string)
    """
    file_path = os.path.join(
            get_package_share_directory('spot_agent'),
            'saved_data',
            'saved_sequences.json'
        )
    
    if not os.path.exists(file_path):
        return "Saved sequences file not found."

    with open(file_path, 'r') as f:
        saved_data = json.load(f)

    if seq_name == "" or seq_name.lower() == "list":
        # Just list all saved locations
        if not saved_data:
            return "No Sequence saved yet."
        locations = "\n".join(f"- {loc}" for loc in saved_data.keys())
        return f"Saved Sequence:\n{locations}"
    
    if seq_name not in saved_data:
        return f"Sequence '{seq_name}' not found in saved locations. Use 'get_sequence' without name to list available."
    
    seq = saved_data[seq_name]
    return {
        "name": seq_name,
        "description": seq.get("description", ""),
        "steps": seq["steps"],
    }

def tools_nav ():
    return [
        set_simulation,
        walk_forward_robot,
        turn_robot,
        save_location_tool,
        move_to_goal,
        get_sequence
    ]
    

