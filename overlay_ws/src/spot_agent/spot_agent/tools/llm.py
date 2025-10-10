from langchain_ollama import ChatOllama
from .tools_spot import get_tools

language_model = "llama3.1"
language_model_temperature = 0.8
url_communication = "http://172.17.0.1:11434"

def get_llm():
    return ChatOllama(model=language_model, 
                      temperature=language_model_temperature,
                      base_url=url_communication,
                      ).bind_tools(get_tools())
