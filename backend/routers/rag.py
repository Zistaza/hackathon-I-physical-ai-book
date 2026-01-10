"""
RAG query router for the RAG Backend API.

This module implements the RAG query endpoints that process user questions
and return answers grounded in indexed book content.
"""

from fastapi import APIRouter, HTTPException, Depends
from typing import Optional
import logging

from ..models.query import QueryRequest, SelectedTextQueryRequest, QueryResponse, ErrorResponse
from ..services.rag_service import RAGService
from ..config import settings

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Create router
router = APIRouter(prefix="", tags=["rag"])

# Global RAG service instance
rag_service: Optional[RAGService] = None


def get_rag_service():
    """
    Dependency to get the RAG service instance.

    Returns:
        RAGService: The RAG service instance
    """
    global rag_service
    if rag_service is None:
        rag_service = RAGService()
    return rag_service


@router.post("/query", response_model=QueryResponse, responses={400: {"model": ErrorResponse}, 500: {"model": ErrorResponse}})
async def query_rag(
    query_request: QueryRequest,
    rag_svc: RAGService = Depends(get_rag_service)
):
    """
    Query the RAG system with general book-wide questions.

    Process a user query against the full book content and return an answer
    grounded in the indexed book content with source citations.

    Args:
        query_request: The query request from the frontend
        rag_svc: The RAG service instance

    Returns:
        QueryResponse: The response from the RAG system
    """
    logger.info(f"Received general RAG query: {query_request.question[:50]}...")

    # Validate the query input
    validation_errors = rag_svc.validate_query_input(query_request)
    if validation_errors:
        logger.warning(f"Query validation failed: {', '.join(validation_errors)}")
        raise HTTPException(
            status_code=400,
            detail=ErrorResponse(
                error=f"Invalid query: {', '.join(validation_errors)}",
                code="INVALID_INPUT",
                details={"validation_errors": validation_errors}
            ).dict()
        )

    try:
        # Process the query with the RAG service
        response = rag_svc.process_query(query_request)

        logger.info(f"Query processed successfully, response length: {len(response.answer)}")
        return response

    except Exception as e:
        logger.error(f"Error processing RAG query: {str(e)}")
        raise HTTPException(
            status_code=500,
            detail=ErrorResponse(
                error=f"Error processing query: {str(e)}",
                code="PROCESSING_ERROR",
                details={}
            ).dict()
        )


@router.post("/query-selected-text", response_model=QueryResponse, responses={400: {"model": ErrorResponse}, 500: {"model": ErrorResponse}})
async def query_rag_selected_text(
    query_request: SelectedTextQueryRequest,
    rag_svc: RAGService = Depends(get_rag_service)
):
    """
    Query the RAG system with selected text constraints.

    Process a user query restricted to user-selected text content and return an answer
    based only on that specific text with source citations.

    Args:
        query_request: The selected text query request from the frontend
        rag_svc: The RAG service instance

    Returns:
        QueryResponse: The response from the RAG system
    """
    logger.info(f"Received selected text RAG query: {query_request.question[:50]}...")

    # Validate the query input
    # Create a base QueryRequest to validate the common fields
    base_request = QueryRequest(
        id=query_request.id,
        question=query_request.question,
        session_id=query_request.session_id,
        metadata=query_request.metadata
    )

    validation_errors = rag_svc.validate_query_input(base_request)

    # Additional validation for selected text constraints
    if query_request.selected_text_only and not query_request.selected_text_content:
        validation_errors.append("selected_text_content must be provided when selected_text_only is true")

    if validation_errors:
        logger.warning(f"Selected text query validation failed: {', '.join(validation_errors)}")
        raise HTTPException(
            status_code=400,
            detail=ErrorResponse(
                error=f"Invalid selected text query: {', '.join(validation_errors)}",
                code="INVALID_INPUT",
                details={"validation_errors": validation_errors}
            ).dict()
        )

    try:
        # Process the selected text query with the RAG service
        response = rag_svc.process_selected_text_query(query_request)

        logger.info(f"Selected text query processed successfully, response length: {len(response.answer)}")
        return response

    except Exception as e:
        logger.error(f"Error processing selected text RAG query: {str(e)}")
        raise HTTPException(
            status_code=500,
            detail=ErrorResponse(
                error=f"Error processing selected text query: {str(e)}",
                code="PROCESSING_ERROR",
                details={}
            ).dict()
        )


@router.on_event("shutdown")
async def shutdown_event():
    """
    Event handler for application shutdown.
    Clean up the RAG service resources.
    """
    global rag_service
    if rag_service:
        rag_service.close()
        logger.info("RAG service shut down successfully")


# Test endpoint to verify basic functionality
@router.get("/test")
async def test_endpoint():
    """
    Test endpoint to verify the RAG router is functioning.

    Returns:
        dict: Test response
    """
    logger.info("Test endpoint called")
    return {"status": "ok", "message": "RAG router is functioning"}


# Health check for RAG-specific components
@router.get("/health")
async def rag_specific_health(rag_svc: RAGService = Depends(get_rag_service)):
    """
    Health check for RAG-specific components.

    Args:
        rag_svc: The RAG service instance

    Returns:
        dict: Health check results for RAG components
    """
    logger.info("RAG-specific health check called")

    try:
        health_result = rag_svc.health_check()
        return health_result
    except Exception as e:
        logger.error(f"RAG-specific health check failed: {str(e)}")
        raise HTTPException(
            status_code=503,
            detail=ErrorResponse(
                error=f"RAG service health check failed: {str(e)}",
                code="HEALTH_CHECK_FAILED",
                details={}
            ).dict()
        )