from langchain_ollama import ChatOllama
from .agent_tools import get_tools

language_model = "llama3.1"
language_model_temperature = 0.8

def get_llm():
    return ChatOllama(model=language_model, 
                      temperature=language_model_temperature,
                      ).bind_tools(get_tools())
