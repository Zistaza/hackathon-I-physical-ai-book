"""
Configuration file for the RAG Agent with OpenAI Agents SDK.

This file contains configuration settings and constants for the RAG agent.
"""

import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# OpenAI Configuration
OPENAI_API_KEY = os.getenv("OPENAI_API_KEY")
# Use a valid OpenAI model for assistants API, override if mistral model is specified
AGENT_MODEL_ENV = os.getenv("AGENT_MODEL", "gpt-4o")
if "mistralai" in AGENT_MODEL_ENV:
    OPENAI_MODEL = "gpt-4o"  # Use valid OpenAI model for assistants API
else:
    OPENAI_MODEL = AGENT_MODEL_ENV

# Cohere Configuration
COHERE_API_KEY = os.getenv("COHERE_API_KEY")
COHERE_MODEL = os.getenv("COHERE_MODEL", "embed-multilingual-v2.0")

# Qdrant Configuration
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
QDRANT_COLLECTION_NAME = os.getenv("QDRANT_COLLECTION_NAME", "book_embeddings")

# Retrieval Configuration
RETRIEVAL_THRESHOLD = float(os.getenv("RETRIEVAL_THRESHOLD", "0.3"))
DEFAULT_TOP_K = 5

# Agent Configuration
AGENT_NAME = "Book RAG Assistant"
AGENT_INSTRUCTIONS = """
You are a RAG (Retrieval-Augmented Generation) assistant for book content.

Your primary function is to answer questions based solely on retrieved book content.
You must:
1. Only use information from the retrieved content chunks
2. Cite the source of your information (module, chapter, URL)
3. If the information is not found in the retrieved content, explicitly state "Answer not found in book content."
4. When in selected-text-only mode, only use the provided selected text for answers
5. Handle conflicting information by citing both sources if present
6. Do not use any external knowledge or make up information.
7. Be deterministic: identical inputs should produce identical outputs
"""

# Validation Configuration
DETERMINISTIC_RUNS = 3