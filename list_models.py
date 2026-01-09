"""
Simple script to list available models from OpenAI API
"""
import openai
from dotenv import load_dotenv
import os

load_dotenv()

# Initialize OpenAI client
client = openai.OpenAI(api_key=os.getenv("OPENAI_API_KEY"))

try:
    # List available models
    models = client.models.list()
    print("Available models:")
    for model in models.data:
        print(f"- {model.id}")
except Exception as e:
    print(f"Error listing models: {e}")

    # Try a few common models that might be available
    common_models = ["gpt-4", "gpt-4o", "gpt-4o-mini", "gpt-3.5-turbo"]
    print("\nTrying common models to see which are available:")

    for model_name in common_models:
        try:
            # Just try to create an assistant to see if model exists
            assistant = client.beta.assistants.create(
                name="Test Assistant",
                instructions="You are a test assistant.",
                model=model_name
            )
            print(f"✓ {model_name} is available")
            # Clean up the test assistant
            client.beta.assistants.delete(assistant.id)
            break
        except Exception as e:
            print(f"✗ {model_name} not available: {str(e)[:100]}...")