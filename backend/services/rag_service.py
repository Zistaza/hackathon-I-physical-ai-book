"""
RAG Service for the RAG Backend API.

This module implements the core RAG (Retrieval-Augmented Generation) service
that orchestrates between the frontend requests and backend agent/retrieval logic.
"""

from typing import Dict, Any, Optional, List
from datetime import datetime
import logging
import hashlib
import time
from ..models.query import QueryRequest, SelectedTextQueryRequest, QueryResponse, Citation, RetrievedChunk
from ..config import settings
from ..retrieve import RAGRetriever, RetrievedChunk as RetrievedChunkExternal
from ..agent import RAGAgent, UserQuery, AgentResponse


# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class RAGService:
    """
    Main RAG service class that handles orchestration between frontend requests
    and backend agent/retrieval functionality.
    """

    def __init__(self):
        """
        Initialize the RAG service with necessary components.
        """
        logger.info("Initializing RAG Service")

        # Initialize the existing agent and retrieval components
        self.agent = RAGAgent()
        self.retriever = RAGRetriever()

        logger.info("RAG Service initialized successfully")

    def process_query(self, query_request: QueryRequest) -> QueryResponse:
        """
        Process a general RAG query against the full book content.

        Args:
            query_request: The query request from the frontend

        Returns:
            QueryResponse: The response from the RAG system
        """
        logger.info(f"Processing general query: {query_request.question[:50]}...")

        try:
            # Create a UserQuery object for the agent
            user_query = UserQuery(
                id=query_request.id or f"query_{int(time.time())}",
                text=query_request.question,
                selected_text_only=False,
                selected_text_content=None,
                timestamp=time.time()
            )

            # Process the query with the agent
            agent_response = self.agent.query(user_query)

            # Convert the agent response to the API response format
            response = self._convert_agent_response_to_api_response(
                agent_response,
                query_request.id or user_query.id,
                query_request.session_id
            )

            logger.info(f"Query processed successfully, response length: {len(response.answer)}")
            return response

        except Exception as e:
            logger.error(f"Error processing query: {str(e)}")
            # Return an error response
            return self._create_error_response(str(e), query_request.session_id)

    def process_selected_text_query(self, query_request: SelectedTextQueryRequest) -> QueryResponse:
        """
        Process a RAG query restricted to user-selected text content.

        Args:
            query_request: The selected text query request from the frontend

        Returns:
            QueryResponse: The response from the RAG system
        """
        logger.info(f"Processing selected text query: {query_request.question[:50]}...")

        try:
            # Create a UserQuery object for the agent with selected text constraints
            user_query = UserQuery(
                id=query_request.id or f"query_{int(time.time())}",
                text=query_request.question,
                selected_text_only=query_request.selected_text_only,
                selected_text_content=query_request.selected_text_content,
                timestamp=time.time()
            )

            # Process the query with the agent
            agent_response = self.agent.query(user_query)

            # Convert the agent response to the API response format
            response = self._convert_agent_response_to_api_response(
                agent_response,
                query_request.id or user_query.id,
                query_request.session_id
            )

            logger.info(f"Selected text query processed successfully, response length: {len(response.answer)}")
            return response

        except Exception as e:
            logger.error(f"Error processing selected text query: {str(e)}")
            # Return an error response
            return self._create_error_response(str(e), query_request.session_id)

    def _convert_agent_response_to_api_response(
        self,
        agent_response: AgentResponse,
        request_id: str,
        session_id: Optional[str]
    ) -> QueryResponse:
        """
        Convert the agent response to the API response format.

        Args:
            agent_response: The response from the RAG agent
            request_id: The original request ID
            session_id: The session ID from the request

        Returns:
            QueryResponse: The converted response in API format
        """
        # Convert citations from agent format to API format
        citations = [
            Citation(
                source_url=citation.get('source_url'),
                module=citation.get('module'),
                chapter=citation.get('chapter'),
                section=citation.get('section'),
                page_number=citation.get('page_number'),
                paragraph_number=citation.get('paragraph_number'),
                citation_type=citation.get('citation_type')
            )
            for citation in agent_response.citations
        ]

        # Convert retrieved chunks from agent format to API format
        retrieved_chunks = [
            RetrievedChunk(
                text=chunk.text,
                score=chunk.score,
                source_url=chunk.source_url,
                module=chunk.module,
                chapter=chunk.chapter,
                chunk_index=chunk.chunk_index
            )
            for chunk in agent_response.retrieved_chunks_used
        ]

        # Create the API response
        response = QueryResponse(
            id=request_id,
            answer=agent_response.content,
            citations=citations,
            retrieved_chunks=retrieved_chunks,
            refusal_reason=agent_response.refusal_reason,
            deterministic_hash=agent_response.deterministic_hash,
            timestamp=agent_response.timestamp or time.time(),
            session_id=session_id
        )

        return response

    def _create_error_response(self, error_message: str, session_id: Optional[str]) -> QueryResponse:
        """
        Create an error response when processing fails.

        Args:
            error_message: The error message to include
            session_id: The session ID from the request

        Returns:
            QueryResponse: An error response
        """
        return QueryResponse(
            id=f"error_{int(time.time())}",
            answer="An error occurred while processing your request. Please try again later.",
            citations=[],
            retrieved_chunks=[],
            refusal_reason=f"Error: {error_message}",
            deterministic_hash=None,
            timestamp=time.time(),
            session_id=session_id
        )

    def validate_query_input(self, query_request: QueryRequest) -> List[str]:
        """
        Validate the query input for common issues.

        Args:
            query_request: The query request to validate

        Returns:
            List[str]: List of validation errors (empty if valid)
        """
        errors = []

        # Validate question is not empty
        if not query_request.question or not query_request.question.strip():
            errors.append("Question cannot be empty")

        # For selected text queries, validate additional constraints
        if hasattr(query_request, 'selected_text_only') and query_request.selected_text_only:
            if not hasattr(query_request, 'selected_text_content') or not query_request.selected_text_content:
                errors.append("selected_text_content must be provided when selected_text_only is true")

        return errors

    def check_content_availability(self, query: str) -> bool:
        """
        Check if the requested content is likely available in the indexed content.

        Args:
            query: The query to check for content availability

        Returns:
            bool: True if content is likely available, False otherwise
        """
        try:
            # Retrieve some content to see if there are relevant results
            retrieved_chunks = self.retriever.retrieve(query, top_k=3)

            # If we got results with reasonable scores, content is likely available
            if len(retrieved_chunks) > 0:
                # Check if the highest scoring result has a decent score
                highest_score = max([chunk.score for chunk in retrieved_chunks])
                return highest_score > settings.retrieval_threshold

            return False
        except Exception as e:
            logger.error(f"Error checking content availability: {str(e)}")
            return False

    async def health_check(self) -> Dict[str, Any]:
        """
        Perform a health check of the RAG service and its dependencies.

        Returns:
            Dict[str, Any]: Health check results
        """
        logger.info("Performing RAG service health check")

        # Check if we can connect to the retrieval system
        retrieval_healthy = True
        try:
            # Try a simple retrieval to verify the system works
            test_results = self.retriever.retrieve("test", top_k=1)
        except Exception as e:
            logger.error(f"Retrieval system health check failed: {str(e)}")
            retrieval_healthy = False

        # Check if we can process a simple query with the agent
        agent_healthy = True
        try:
            test_query = UserQuery(
                id="health_check",
                text="What is this system for?",
                selected_text_only=False
            )
            # We won't actually call the agent here to avoid hitting APIs unnecessarily
            # Instead, we'll just verify the agent is initialized
            agent_initialized = self.agent is not None
            agent_healthy = agent_initialized
        except Exception as e:
            logger.error(f"Agent health check failed: {str(e)}")
            agent_healthy = False

        health_status = {
            "service": "rag",
            "status": "healthy" if (retrieval_healthy and agent_healthy) else "unhealthy",
            "timestamp": datetime.utcnow().isoformat(),
            "checks": {
                "retrieval_system": retrieval_healthy,
                "agent_system": agent_healthy
            }
        }

        logger.info(f"RAG service health check completed: {health_status['status']}")
        return health_status

    def close(self):
        """
        Clean up resources used by the RAG service.
        """
        logger.info("Closing RAG Service")
        if hasattr(self, 'agent') and self.agent:
            self.agent.close()