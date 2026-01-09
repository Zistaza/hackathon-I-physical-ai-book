#!/usr/bin/env python3
"""
Test assistant creation to see what response we get
"""
import os
from openai import OpenAI

# Use OpenRouter API
openrouter_key = os.getenv("OPENROUTER_API_KEY", "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")
openrouter_base_url = os.getenv("OPENROUTER_BASE_URL", "https://openrouter.ai/api/v1")

client = OpenAI(
    api_key=openrouter_key,
    base_url=openrouter_base_url
)

print("Testing assistant creation...")

try:
    # Define the retrieval tool
    retrieval_tool = {
        "type": "function",
        "function": {
            "name": "retrieve_content",
            "description": "Retrieve relevant content from the book based on the user's query",
            "parameters": {
                "type": "object",
                "properties": {
                    "query": {
                        "type": "string",
                        "description": "The user's question or query to search for in the book content"
                    },
                    "top_k": {
                        "type": "integer",
                        "description": "Number of results to return (default: 5)",
                        "minimum": 1,
                        "maximum": 10
                    },
                    "selected_text_only": {
                        "type": "boolean",
                        "description": "Whether to restrict search to selected text only",
                        "default": False
                    },
                    "selected_text_content": {
                        "type": "string",
                        "description": "The specific text content to search within when selected_text_only is true"
                    }
                },
                "required": ["query"]
            }
        }
    }

    # Create the assistant
    assistant = client.beta.assistants.create(
        name="Book RAG Assistant",
        instructions="You are a RAG (Retrieval-Augmented Generation) assistant for book content. Your primary function is to answer questions based solely on retrieved book content. You MUST: 1. ALWAYS call the retrieve_content function FIRST to get relevant information from the book 2. Only use information from the retrieved content chunks 3. Cite the source of your information (module, chapter, URL) 4. If the information is not found in the retrieved content, explicitly state 'Answer not found in book content.' 5. When in selected-text-only mode, only use the provided selected text for answers 6. Handle conflicting information by citing both sources if present 7. Do not use any external knowledge or make up information. 8. Be deterministic: identical inputs should produce identical outputs",
        model="gpt-4o",  # Use the model from config
        tools=[retrieval_tool]
    )

    print(f"Assistant created successfully!")
    print(f"Assistant type: {type(assistant)}")
    print(f"Assistant: {assistant}")

    if hasattr(assistant, 'id'):
        print(f"Assistant ID: {assistant.id}")
    else:
        print("Assistant object doesn't have an 'id' attribute")

except Exception as e:
    print(f"Error creating assistant: {e}")
    import traceback
    traceback.print_exc()