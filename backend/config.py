"""
Configuration module for RAG Backend API

This module handles environment variable loading and configuration management
for the RAG (Retrieval-Augmented Generation) system.
"""

from pydantic_settings import BaseSettings
from typing import Optional, List
import os


class Settings(BaseSettings):
    """
    Application settings loaded from environment variables.
    """

    # API Keys
    openai_api_key: Optional[str] = None
    openrouter_api_key: Optional[str] = None
    cohere_api_key: Optional[str] = None

    # Qdrant Configuration
    qdrant_url: str
    qdrant_api_key: str
    qdrant_collection_name: str = "book_embeddings"

    # Cohere Model Configuration
    cohere_model: str = "embed-english-v3.0"

    # Agent Configuration
    agent_model: str = "gpt-4o"
    openrouter_base_url: str = "https://openrouter.ai/api/v1"

    # Retrieval Configuration
    retrieval_threshold: float = 0.3

    # Server Configuration
    server_host: str = "0.0.0.0"
    server_port: int = 8000
    debug: bool = False

    class Config:
        env_file = ".env"
        case_sensitive = False
        env_prefix = ""
        extra = "ignore"  # Ignore extra environment variables


# Global settings instance
settings = Settings()


def validate_config():
    """
    Validate that all required configuration values are present.

    Returns:
        tuple: (is_valid: bool, error_messages: List[str])
    """
    errors = []

    # Validate required API keys
    if not settings.cohere_api_key:
        errors.append("COHERE_API_KEY is required")

    if not settings.qdrant_url:
        errors.append("QDRANT_URL is required")

    if not settings.qdrant_api_key:
        errors.append("QDRANT_API_KEY is required")

    # Check if at least one of OpenAI or OpenRouter API key is provided
    if not settings.openai_api_key and not settings.openrouter_api_key:
        errors.append("Either OPENAI_API_KEY or OPENROUTER_API_KEY must be provided")

    return len(errors) == 0, errors


def get_config_summary():
    """
    Get a summary of the current configuration (without sensitive data).

    Returns:
        dict: Configuration summary
    """
    return {
        "qdrant_url": settings.qdrant_url,
        "qdrant_collection_name": settings.qdrant_collection_name,
        "cohere_model": settings.cohere_model,
        "agent_model": settings.agent_model,
        "retrieval_threshold": settings.retrieval_threshold,
        "server_host": settings.server_host,
        "server_port": settings.server_port,
        "debug": settings.debug
    }


if __name__ == "__main__":
    # Validate configuration when run as main module
    is_valid, errors = validate_config()

    if is_valid:
        print("✓ Configuration is valid")
        print(f"Configuration summary: {get_config_summary()}")
    else:
        print("✗ Configuration validation failed:")
        for error in errors:
            print(f"  - {error}")