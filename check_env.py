#!/usr/bin/env python3
"""Simple script to check if environment variables are loaded properly."""

import os
from dotenv import load_dotenv

# Load environment variables from the root directory
load_dotenv()

print("Environment variables check:")
print(f"OPENROUTER_API_KEY: {'Present' if os.getenv('OPENROUTER_API_KEY') else 'Missing'}")
print(f"OPENAI_API_KEY: {'Present' if os.getenv('OPENAI_API_KEY') else 'Missing'}")
print(f"COHERE_API_KEY: {'Present' if os.getenv('COHERE_API_KEY') else 'Missing'}")
print(f"QDRANT_URL: {os.getenv('QDRANT_URL', 'Missing')}")
print(f"QDRANT_API_KEY: {'Present' if os.getenv('QDRANT_API_KEY') else 'Missing'}")

# Now also load from backend directory
import sys
sys.path.append('./backend')

from backend.config import settings
print("\nSettings from backend config:")
print(f"openrouter_api_key: {'Present' if settings.openrouter_api_key else 'Missing'}")
print(f"openai_api_key: {'Present' if settings.openai_api_key else 'Missing'}")
print(f"cohere_api_key: {'Present' if settings.cohere_api_key else 'Missing'}")
print(f"qdrant_url: {settings.qdrant_url}")
print(f"qdrant_api_key: {'Present' if settings.qdrant_api_key else 'Missing'}")