# spot_task_prompts.py

from typing import Optional, Tuple
from langchain_core.messages import SystemMessage

# === “Container” for any set of extra, task-specific instructions ===
class RobotSystemPrompts:
    def __init__(
        self,
        embodiment_and_persona: Optional[str] = None,
        critical_instructions: Optional[str] = None,
        constraints_and_guardrails: Optional[str] = None,
        mission_and_objectives: Optional[str] = None,
        about_your_environment: Optional[str] = None,
        about_your_capabilities: Optional[str] = None,
        nuance_and_assumptions: Optional[str] = None,
        environment_variables: Optional[dict] = None,
    ):
        self.embodiment = embodiment_and_persona
        self.critical_instructions = critical_instructions
        self.constraints_and_guardrails = constraints_and_guardrails
        self.mission_and_objectives = mission_and_objectives
        self.about_your_environment = about_your_environment
        self.about_your_capabilities = about_your_capabilities
        self.nuance_and_assumptions = nuance_and_assumptions
        self.environment_variables = environment_variables

    def as_message(self) -> Tuple[str, str]:
        """
        Concatenate all non-None fields into one SystemMessage-style tuple.
        """
        content = "\n==========\n"
        content += "Begin Task-Specific System Prompts\n"
        parts = []
        if self.embodiment:
            parts.append(f"Embodiment and Persona: {self.embodiment}")
        if self.critical_instructions:
            parts.append(f"Critical Instructions: {self.critical_instructions}")
        if self.constraints_and_guardrails:
            parts.append(f"Constraints and Guardrails: {self.constraints_and_guardrails}")
        if self.mission_and_objectives:
            parts.append(f"Mission and Objectives: {self.mission_and_objectives}")
        if self.about_your_environment:
            parts.append(f"About Your Environment: {self.about_your_environment}")
        if self.about_your_capabilities:
            parts.append(f"About Your Capabilities: {self.about_your_capabilities}")
        if self.nuance_and_assumptions:
            parts.append(f"Nuance and Assumptions: {self.nuance_and_assumptions}")
        if self.environment_variables:
            parts.append(f"Environment Variables: {self.environment_variables}")
        content += "\n---\n".join(parts)
        content += "\nEnd Task-Specific System Prompts\n==========\n"
        return ("system", content)


# === Pre-built “mapping” prompts ===
mapping_prompts = RobotSystemPrompts(
    critical_instructions=(
        "You are performing autonomous exploration. After receiving a map update, "
        "identify frontiers, pick the largest-closest one, and send a navigation goal. "
        "If you detect that the robot is stuck, blacklist that frontier and choose another."
    ),
    mission_and_objectives=(
        "Your mission is to build a complete 2D occupancy map. Use tools: 'find_frontiers', "
        "'select_frontier', 'send_nav_goal'. Once no frontiers remain, save the map with 'save_map' tool."
    ),
    constraints_and_guardrails=(
        "Do not assume any frontier is reachable until you verify navigation success. "
        "Always wait for tool feedback before proceeding."
    ),
)

# === Pre-built “pick-and-place” prompts ===
pick_place_prompts = RobotSystemPrompts(
    critical_instructions=(
        "You are performing a pick-and-place operation. "
        "First, scan for the target object using 'detect_object' tool. Then plan an arm path with 'plan_arm_path'. "
        "Finally execute it with 'execute_arm_path'. Confirm grasp success using 'check_grasp'."
    ),
    mission_and_objectives=(
        "Your mission is to pick up the specified object and place it at the designated drop point. "
        "Use tools: 'detect_object', 'plan_arm_path', 'execute_arm_path', 'open_gripper', 'close_gripper'."
    ),
    constraints_and_guardrails=(
        "Never collide with nearby obstacles. If the grasp attempt fails, retry up to two times, then report failure."
    ),
)

test_prompt = RobotSystemPrompts(
    critical_instructions=(
        "You are performing a test of your functionality. "
        "You should be able to excecute three ROS2 calls. When asked you should try to execute them. "
        "Always give the output of the tool call as answer with your two cents."
    ),
    mission_and_objectives=(
        "Your mission is to test you capabilities, with tool calling and interpretation."
    ),
    constraints_and_guardrails=(
        "Never do anything or execute any tools without being asked to. If the tool call fail, retry up to two times, then report the failure."
    ),
)
