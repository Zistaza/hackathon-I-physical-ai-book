"""
Pydantic models for health check endpoints.

This module defines the data models for health check requests and responses
used in the RAG Backend API.
"""

from pydantic import BaseModel
from typing import Dict, Any
from datetime import datetime


class HealthResponse(BaseModel):
    """
    Response model for health check endpoint.

    Fields:
    - status: Overall system status ("healthy", "degraded", "unhealthy")
    - timestamp: Unix timestamp of check
    - dependencies: Status of individual dependencies (Qdrant, OpenAI API, etc.)
    - version: API version
    """
    status: str
    timestamp: float
    dependencies: Dict[str, bool]
    version: str = "1.0.0"

    class Config:
        json_schema_extra = {
            "example": {
                "status": "healthy",
                "timestamp": 1678886400.123456,
                "dependencies": {
                    "qdrant": True,
                    "openai_api": True,
                    "cohere_api": True
                },
                "version": "1.0.0"
            }
        }


class HealthCheckRequest(BaseModel):
    """
    Request model for health check endpoint (currently empty as health checks are simple GET requests).
    """
    pass