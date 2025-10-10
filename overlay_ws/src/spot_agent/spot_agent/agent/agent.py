import time
from typing import Optional

from langchain_ollama import ChatOllama
from langchain.agents import AgentExecutor
from langchain.agents.format_scratchpad.openai_tools import format_to_openai_tool_messages
from langchain.agents.output_parsers.openai_tools import OpenAIToolsAgentOutputParser
from langchain_core.messages import HumanMessage, AIMessage
from langchain.prompts import ChatPromptTemplate, MessagesPlaceholder

from spot_agent.prompts.system_prompts import system_prompts
from spot_agent.prompts.task_prompts import RobotSystemPrompts
from spot_agent.tools.tools_spot import get_tools


class SpotAgent:
    """
    SpotAgent is a ROS2 agent designed specifically to interact with
    the Spot Robot from Boston Dynamics using Nav2 and MoveIt2.
        
    Args:
        tools (list): A list of tools defined to use with the agent.
        prompts: Prompts defining how the agent should respond.
        llm: A language model used for generating responses.
        streaming (bool): Decides if we want to stream the output of the agent. Defaults to True.
        verbose (bool): Decides if we want to build the verbose output. Defaults to False.
    """
    def __init__(
        self, 
        llm: ChatOllama, 
        streaming: bool = True,
        record_chat_history: bool = True,
        tools: Optional[list] = None,
        task_prompts: Optional[RobotSystemPrompts] = None,
    ):
        # 1) Basic LLM & flags
        self.llm = llm
        self.streaming = streaming
        self.record_chat_history = record_chat_history

        # 2) Memory/scratchpad keys (used by ChatPromptTemplate)
        self.__memory_key = "chat_history"
        self.__scratchpad = "agent_scratchpad"

        # 3) Load your Spot-specific tools
        #    If you passed in a custom `tools` list, use that; else use default get_tools()
        self.tools = get_tools()

        # 4) Build merged prompts (generic + task)
        self.prompts = self._get_prompts(task_prompts)

        # 5) Create the actual LangChain “agent” and “executor”
        self._build_executor()

        # 6) Initialize chat history container
        self.chat_history: list[HumanMessage | AIMessage] = []
        
    def _get_prompts(self, task_prompts: Optional[RobotSystemPrompts] = None) -> ChatPromptTemplate:
        """
        Merge the generic system_prompts (always present)
        with an optional task-specific RobotSystemPrompts block.
        """
        # 1) Start with a fresh copy of the generic prompts
        prompts = list(system_prompts)

        # 2) If the user passed in a task-specific block, append it
        if task_prompts:
            prompts.append(task_prompts.as_message())

        # 3) Finally add Chat History + Scratchpad placeholders
        return ChatPromptTemplate.from_messages(
            prompts
            + [
                MessagesPlaceholder(variable_name=self.__memory_key),
                ("user", "{input}"),
                MessagesPlaceholder(variable_name=self.__scratchpad),
            ]
        )
    
    def _build_executor(self):
        # 1) Bind your tools
        self.llm_with_tools = self.llm.bind_tools(self.tools)

        # 2) Build the “agent descriptor” (following LangChain’s schema)
        #    This is effectively: {"prompt": ..., "llm": llm_with_tools, "output_parser": OpenAIToolsAgentOutputParser(), "input_variables": [...]}
        self.agent_descriptor = {
            "prompt": self.prompts,
            "llm": self.llm_with_tools,
            "output_parser": OpenAIToolsAgentOutputParser(),
            "input_variables": ["input", self.__memory_key, self.__scratchpad],
        }

        # 3) Create the executor
        self.executor = AgentExecutor.from_agent_and_tools(
            agent=self.agent_descriptor,
            tools=self.tools,
            verbose=False,          # or True if you want more detailed logs
            stream_runnable=self.streaming,
        )
    
    def invoke (self, query: str) -> str:
        """
        Invoke the agent with the user query and return the agent's response
        """
        try:
            commands = query.split(" then ")
            responses = []
            for cmd in commands:
                result = self.executor.invoke(
                    input={"input": cmd.strip(), "chat_history": self.chat_history}
                )                    
                output = result["output"]

                # Record chat history for each sub-command
                if self.record_chat_history:
                    self.chat_history.append(HumanMessage(content=cmd.strip()))
                    self.chat_history.append(AIMessage(content=output))
                
                responses.append(str(output))
                print(f"\033[92mProcessed command: '{cmd.strip()}' -> Response: {output}\033[0m")
                
                final_response = "\n".join(responses)

            return final_response

        except Exception as e:
            return f"An Error occured while invoking the Agent: {e}"
        