from langchain_core.messages import SystemMessage

class RobotSystemPrompts:
    def __init__(self, content: str):
        self.content = content

    def as_message(self):
        return SystemMessage(content=self.content)