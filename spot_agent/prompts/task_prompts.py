# spot_task_prompts.py

from typing import Optional, Tuple

# === Pre-built “mapping” prompts ===
mapping_prompt: str = """
### Task: Autonomous Exploration and Mapping

**Critical Instructions**
You are performing autonomous exploration. After receiving a map update,
identify frontiers, pick the largest and closest one, and send a navigation goal.
If the robot becomes stuck, blacklist that frontier and choose another.

**Mission and Objectives**
Your mission is to build a complete 2D occupancy map.
Use tools: `find_frontiers`, `select_frontier`, `send_nav_goal`.
Once no frontiers remain, save the map using the `save_map` tool.

**Constraints and Guardrails**
Do not assume any frontier is reachable until navigation success is verified.
Always wait for tool feedback before proceeding.
"""


# === Pre-built “pick-and-place” prompts ===
pick_place_prompt: str = """
### Task: Pick-and-Place Operation

**Critical Instructions**
You are performing a pick-and-place operation.
First, scan for the target object using the `detect_object` tool.
Then plan an arm path with `plan_arm_path`.
Finally, execute the motion with `execute_arm_path`.
Confirm grasp success using `check_grasp`.

**Mission and Objectives**
Your mission is to pick up the specified object and place it at the designated drop point.
Use tools: `detect_object`, `plan_arm_path`, `execute_arm_path`, `open_gripper`, `close_gripper`.

**Constraints and Guardrails**
Never collide with nearby obstacles.
If the grasp attempt fails, retry up to two times, then report failure.
"""

test_prompt: str = """
### Task: Functional Testing of Tool Calls

**Critical Instructions**
You are performing a test of your functionality.
You should be able to execute three ROS 2 tool calls when asked.
Always return the tool output as your answer, adding your own short commentary.

**Mission and Objectives**
Your mission is to test your tool-calling and interpretation capabilities.

**Constraints and Guardrails**
Never execute any tools unless explicitly requested.
If a tool call fails, retry up to two times, then report the failure.
"""
