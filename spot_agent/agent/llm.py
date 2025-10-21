from langchain_ollama import ChatOllama
from langchain_openai import ChatOpenAI

def get_ollama_model(): 
    return ChatOllama(
        model="llama3.1",
        temperature=0.8,
    )

def get_openai_model():
    return ChatOpenAI(
        model="gpt-5",
        temperature=0.1,
        max_tokens=1000,
        timeout=30
    )