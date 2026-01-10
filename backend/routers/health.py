"""
Health check router for the RAG Backend API.

This module implements the health check endpoint that verifies system health
and dependency availability.
"""

from fastapi import APIRouter, HTTPException, Depends
from datetime import datetime
import logging
import asyncio

from ..models.health import HealthResponse
from ..config import settings, validate_config

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Create router
router = APIRouter(prefix="/health", tags=["health"])

# Dependency check functions
async def check_qdrant_connection():
    """Check if Qdrant is accessible."""
    try:
        from qdrant_client import QdrantClient

        client = QdrantClient(
            url=settings.qdrant_url,
            api_key=settings.qdrant_api_key
        )

        # Try to get collection info to verify connection
        client.get_collection(settings.qdrant_collection_name)
        return True
    except Exception as e:
        logger.error(f"Qdrant connection check failed: {str(e)}")
        return False


async def check_cohere_connection():
    """Check if Cohere API is accessible."""
    try:
        import cohere

        if not settings.cohere_api_key:
            return False

        client = cohere.Client(settings.cohere_api_key)

        # Try a simple API call to verify connectivity
        response = client.embed(
            texts=["health check"],
            model=settings.cohere_model
        )

        return len(response.embeddings[0]) > 0
    except Exception as e:
        logger.error(f"Cohere connection check failed: {str(e)}")
        return False


async def check_openai_connection():
    """Check if OpenAI/OpenRouter API is accessible."""
    try:
        from openai import OpenAI
        import os

        # Use OpenRouter if available, otherwise use OpenAI
        if settings.openrouter_api_key:
            client = OpenAI(
                api_key=settings.openrouter_api_key,
                base_url=settings.openrouter_base_url
            )
        elif settings.openai_api_key:
            client = OpenAI(api_key=settings.openai_api_key)
        else:
            return False

        # Try a simple API call to verify connectivity
        response = client.models.list()

        return len(response.data) > 0
    except Exception as e:
        logger.error(f"OpenAI/OpenRouter connection check failed: {str(e)}")
        return False


@router.get("/", response_model=HealthResponse)
async def health_check():
    """
    Health check endpoint that returns system health status and dependency availability.

    Returns:
        HealthResponse: Health status with dependency information
    """
    logger.info("Health check requested")

    # Validate configuration
    config_valid, config_errors = validate_config()

    if not config_valid:
        logger.warning(f"Configuration validation failed: {', '.join(config_errors)}")
        # Continue with checks but mark status as degraded if possible

    # Run dependency checks concurrently
    qdrant_status, cohere_status, openai_status = await asyncio.gather(
        check_qdrant_connection(),
        check_cohere_connection(),
        check_openai_connection(),
        return_exceptions=True
    )

    # Handle exceptions in the results
    if isinstance(qdrant_status, Exception):
        logger.error(f"Qdrant check raised exception: {qdrant_status}")
        qdrant_status = False

    if isinstance(cohere_status, Exception):
        logger.error(f"Cohere check raised exception: {cohere_status}")
        cohere_status = False

    if isinstance(openai_status, Exception):
        logger.error(f"OpenAI check raised exception: {openai_status}")
        openai_status = False

    # Determine overall status based on dependency statuses
    all_deps_ok = qdrant_status and cohere_status and openai_status
    overall_status = "healthy" if all_deps_ok else "degraded" if config_valid else "unhealthy"

    health_response = HealthResponse(
        status=overall_status,
        timestamp=datetime.utcnow().timestamp(),
        dependencies={
            "qdrant": qdrant_status,
            "cohere_api": cohere_status,
            "openai_api": openai_status
        },
        version="1.0.0"
    )

    logger.info(f"Health check completed with status: {overall_status}")

    # Return 503 if any critical dependency is down
    if not all_deps_ok:
        raise HTTPException(status_code=503, detail=health_response)

    return health_response


@router.get("/detailed")
async def detailed_health_check():
    """
    Detailed health check that provides more information about system status.

    Returns:
        dict: Detailed health information
    """
    logger.info("Detailed health check requested")

    # Get basic health info
    basic_health = await health_check()

    # Add additional system information
    system_info = {
        "health_response": basic_health,
        "config_valid": validate_config()[0],
        "server_time": datetime.utcnow().isoformat(),
        "version": "1.0.0",
        "service_name": "RAG Backend API"
    }

    return system_info


# Background task to periodically update health status
# This could be expanded to track health over time
last_health_status = {"status": "unknown", "timestamp": 0}

def update_last_health_status(status: str):
    """Update the last recorded health status."""
    global last_health_status
    last_health_status = {
        "status": status,
        "timestamp": datetime.utcnow().timestamp()
    }