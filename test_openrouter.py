#!/usr/bin/env python3
"""
Test script to diagnose OpenRouter API authentication issue
"""
import os
from openai import OpenAI
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

def test_openrouter_connection():
    """Test OpenRouter API connection"""
    openrouter_key = os.getenv("OPENROUTER_API_KEY")
    openrouter_base_url = os.getenv("OPENROUTER_BASE_URL", "https://openrouter.ai/api/v1")

    if not openrouter_key:
        print("ERROR: OPENROUTER_API_KEY not found in environment")
        return False

    print(f"Using OpenRouter base URL: {openrouter_base_url}")

    try:
        # Initialize OpenAI client with OpenRouter
        client = OpenAI(
            api_key=openrouter_key,
            base_url=openrouter_base_url
        )

        print("OpenRouter client initialized successfully")

        # Test with a simple API call
        response = client.chat.completions.create(
            model="openai/gpt-4o",
            messages=[{"role": "user", "content": "Hello, are you working?"}],
            max_tokens=10
        )

        print(f"OpenRouter API call successful: {response.choices[0].message.content}")
        return True

    except Exception as e:
        print(f"ERROR: OpenRouter API call failed: {str(e)}")
        return False

def test_openai_connection():
    """Test OpenAI API connection as comparison"""
    openai_key = os.getenv("OPENAI_API_KEY")

    if not openai_key:
        print("INFO: OPENAI_API_KEY not found in environment")
        return False

    try:
        client = OpenAI(api_key=openai_key)

        print("OpenAI client initialized successfully")

        # Test with a simple API call
        response = client.chat.completions.create(
            model="gpt-4o",
            messages=[{"role": "user", "content": "Hello, are you working?"}],
            max_tokens=10
        )

        print(f"OpenAI API call successful: {response.choices[0].message.content}")
        return True

    except Exception as e:
        print(f"ERROR: OpenAI API call failed: {str(e)}")
        return False

if __name__ == "__main__":
    print("Testing OpenRouter and OpenAI API connections...\n")

    print("=== Testing OpenRouter ===")
    openrouter_ok = test_openrouter_connection()

    print("\n=== Testing OpenAI (for comparison) ===")
    openai_ok = test_openai_connection()

    print(f"\nResults:")
    print(f"OpenRouter: {'✓ PASS' if openrouter_ok else '✗ FAIL'}")
    print(f"OpenAI: {'✓ PASS' if openai_ok else '✗ FAIL'}")