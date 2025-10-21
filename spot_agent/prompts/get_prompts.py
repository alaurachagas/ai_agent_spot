from typing import Optional
    
def get_prompts(generic_prompt: str,  task_prompts: Optional[str] = None) -> str:
    """
    Merge the generic system_prompts (always present)
    with an optional task-specific RobotSystemPrompts block.
    """
    # 1) Start with a fresh copy of the generic prompts
    prompts = generic_prompt

    # 2) If the user passed in a task-specific block, append it
    if task_prompts:
        prompts = prompts + "\n\n" + task_prompts

    return prompts
    






