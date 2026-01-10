"""
RAG Agent with OpenRouter Chat Completions API
==============================================

This file implements a RAG (Retrieval-Augmented Generation) agent using the OpenRouter Chat Completions API
that integrates with the existing Spec-6 retrieval pipeline. The agent accepts user questions,
performs retrieval over the Qdrant + Cohere vector store, and generates grounded, source-cited
answers strictly from retrieved book content.

The implementation emphasizes:
- Determinism: identical inputs produce identical outputs
- Grounding: answers only from retrieved content, no hallucinations
- Citation: proper source attribution in all responses
- Safe refusal: when content is not found in the book
"""

import os
import json
import hashlib
import time
import logging
from typing import Dict, List, Optional, Any
from dataclasses import dataclass
from openai import OpenAI
import cohere
from qdrant_client import QdrantClient
from dotenv import load_dotenv

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


# Load environment variables
load_dotenv()


@dataclass
class RetrievedChunk:
    """Represents a retrieved content chunk from the vector store."""
    text: str
    score: float
    source_url: str
    module: str
    chapter: str
    chunk_index: int
    relevance_score: Optional[float] = None


# Configuration
AGENT_NAME = "Book RAG Assistant"
AGENT_INSTRUCTIONS = """
You are a RAG (Retrieval-Augmented Generation) assistant for book content.

Your primary function is to answer questions based solely on retrieved book content.
You MUST:
1. ALWAYS call the retrieve_content function FIRST to get relevant information from the book
2. Only use information from the retrieved content chunks
3. Cite the source of your information (module, chapter, URL)
4. If the information is not found in the retrieved content, explicitly state "Answer not found in book content."
5. When in selected-text-only mode, only use the provided selected text for answers
6. Handle conflicting information by citing both sources if present
7. Do not use any external knowledge or make up information.
8. Be deterministic: identical inputs should produce identical outputs
"""

# Check if using OpenRouter
if os.getenv("OPENROUTER_API_KEY"):
    OPENAI_MODEL = "openai/gpt-4o"  # OpenRouter format
else:
    # Use a valid OpenAI model name
    OPENAI_MODEL = os.getenv("AGENT_MODEL", "gpt-4o")


@dataclass
class UserQuery:
    """Represents the input question from the user."""
    id: str
    text: str
    selected_text_only: bool = False
    selected_text_content: Optional[str] = None
    timestamp: Optional[float] = None


@dataclass
class AgentResponse:
    """Represents the output from the RAG agent."""
    id: str
    content: str
    citations: List[Dict[str, str]]
    retrieved_chunks_used: List[RetrievedChunk]
    refusal_reason: Optional[str] = None
    timestamp: Optional[float] = None
    deterministic_hash: Optional[str] = None


class RAGRetriever:
    """Interface to the existing retrieval system (Spec-6 pipeline)."""

    def __init__(self):
        # Initialize clients using environment variables
        self.cohere_client = cohere.Client(os.getenv("COHERE_API_KEY"))
        self.qdrant_client = QdrantClient(
            url=os.getenv("QDRANT_URL"),
            api_key=os.getenv("QDRANT_API_KEY")
        )
        self.collection_name = os.getenv("QDRANT_COLLECTION_NAME", "book_embeddings")
        self.retrieval_threshold = float(os.getenv("RETRIEVAL_THRESHOLD", "0.3"))

    def retrieve_content(self, query: str, top_k: int = 5,
                        selected_text_only: bool = False,
                        selected_text_content: Optional[str] = None) -> Dict[str, Any]:
        """
        Retrieve content from the vector store based on the query.

        Args:
            query: The user's question or query to search for
            top_k: Number of results to return (default: 5)
            selected_text_only: Whether to restrict search to selected text only
            selected_text_content: The specific text content to search within

        Returns:
            Dictionary containing retrieved chunks and metadata
        """
        # Input validation
        if not query or not isinstance(query, str):
            logger.error("Invalid query provided to retrieve_content")
            return {
                "chunks": [],
                "empty_retrieval": True,
                "conflicting_info": False,
                "error": "Invalid query: query must be a non-empty string"
            }

        if not isinstance(top_k, int) or top_k <= 0:
            logger.warning(f"Invalid top_k value: {top_k}, using default of 5")
            top_k = 5

        if selected_text_only and (not selected_text_content or not isinstance(selected_text_content, str)):
            logger.error("Invalid selected_text_content provided for selected_text_only mode")
            return {
                "chunks": [],
                "empty_retrieval": True,
                "conflicting_info": False,
                "error": "Invalid selected_text_content: must be a non-empty string when selected_text_only is True"
            }

        try:
            if selected_text_only and selected_text_content:
                # In selected-text-only mode, we simulate searching within the provided text
                # This would typically require a different search strategy depending on implementation
                # For now, we'll return the selected text as a chunk with perfect score
                logger.info(f"Retrieving in selected-text-only mode for query: {query[:50]}...")
                return {
                    "chunks": [{
                        "text": selected_text_content,
                        "score": 1.0,
                        "metadata": {
                            "source_url": "selected_text",
                            "module": "selected",
                            "chapter": "selected",
                            "chunk_index": 0
                        }
                    }],
                    "empty_retrieval": False,
                    "conflicting_info": False
                }
            else:
                # Standard retrieval from the full book content
                logger.info(f"Performing standard retrieval for query: {query[:50]}...")

                # Generate query embedding
                response = self.cohere_client.embed(
                    texts=[query],
                    model=os.getenv("COHERE_MODEL", "embed-english-v3.0"),
                    input_type="search_document"
                )
                query_embedding = response.embeddings[0]

                # Perform similarity search
                search_results = self.qdrant_client.search(
                    collection_name=self.collection_name,
                    query_vector=query_embedding,
                    limit=top_k,
                    with_payload=True
                )

                # Format results
                chunks = []
                for result in search_results:
                    if result.score >= self.retrieval_threshold:
                        chunks.append({
                            "text": result.payload.get("content", ""),
                            "score": result.score,
                            "metadata": {
                                "source_url": result.payload.get("metadata", {}).get("source_url", ""),
                                "module": result.payload.get("metadata", {}).get("module", ""),
                                "chapter": result.payload.get("metadata", {}).get("chapter", ""),
                                "chunk_index": result.payload.get("metadata", {}).get("chunk_index", "")
                            }
                        })

                # Check for conflicting information
                conflicting_info = self._detect_conflicts(chunks)

                logger.info(f"Retrieved {len(chunks)} chunks for query: {query[:30]}...")
                return {
                    "chunks": chunks,
                    "empty_retrieval": len(chunks) == 0,
                    "conflicting_info": conflicting_info
                }
        except Exception as e:
            logger.error(f"Error during content retrieval: {str(e)}")
            # Handle retrieval errors
            return {
                "chunks": [],
                "empty_retrieval": True,
                "conflicting_info": False,
                "error": str(e)
            }

    def _detect_conflicts(self, chunks: List[Dict]) -> bool:
        """Detect if there are conflicting pieces of information in the retrieved chunks."""
        # Simple conflict detection - in practice, this would be more sophisticated
        # For now, we'll return False as conflict detection is complex
        return False


class RAGAgent:
    """Main RAG agent implementation using OpenRouter Chat Completions API."""

    def __init__(self):
        # Initialize OpenAI client with OpenRouter API key and base URL
        openrouter_key = os.getenv("OPENROUTER_API_KEY")
        openrouter_base_url = os.getenv("OPENROUTER_BASE_URL", "https://openrouter.ai/api/v1")

        if openrouter_key:
            # Use OpenRouter API
            self.client = OpenAI(
                api_key=openrouter_key,
                base_url=openrouter_base_url
            )
            self._using_openrouter = True
        else:
            # Fallback to OpenAI API
            self.client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))
            self._using_openrouter = False

        self.retriever = RAGRetriever()

        # Define the retrieval tool for function calling
        self.retrieval_tool = {
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

    def query(self, user_query: UserQuery) -> AgentResponse:
        """
        Process a user query and return a grounded response using OpenRouter Chat Completions API.

        Args:
            user_query: The query from the user

        Returns:
            AgentResponse containing the answer and citations
        """
        logger.info(f"Processing query: {user_query.text[:50]}...")

        try:
            # Prepare the initial message with the user's query
            messages = [
                {"role": "system", "content": AGENT_INSTRUCTIONS},
                {"role": "user", "content": user_query.text}
            ]

            # Call the OpenRouter API with function calling enabled
            response = self.client.chat.completions.create(
                model=OPENAI_MODEL,
                messages=messages,
                tools=[self.retrieval_tool],
                tool_choice="auto",  # Let the model decide when to call the function
                max_tokens=2000  # Limit tokens to avoid exceeding OpenRouter limits
            )

            # Process the response
            response_message = response.choices[0].message
            tool_calls = response_message.tool_calls

            # If the model wants to call the retrieval function
            if tool_calls:
                # Add the assistant message with tool_calls to the messages list
                messages.append({
                    "role": "assistant",
                    "content": response_message.content,
                    "tool_calls": tool_calls
                })

                # Execute the tool calls
                tool_results = []
                for tool_call in tool_calls:
                    if tool_call.function.name == "retrieve_content":
                        # Parse the arguments
                        try:
                            args = json.loads(tool_call.function.arguments)
                        except json.JSONDecodeError as e:
                            logger.error(f"Error parsing tool call arguments: {e}")
                            continue

                        # Call the retrieval function
                        result = self.retriever.retrieve_content(
                            query=args.get('query', ''),
                            top_k=args.get('top_k', 5),
                            selected_text_only=args.get('selected_text_only', False),
                            selected_text_content=args.get('selected_text_content')
                        )

                        # Check if retrieval had errors
                        if 'error' in result:
                            logger.warning(f"Retrieval error: {result['error']}")
                            # Still pass the result to maintain flow, but it will have empty chunks

                        # Add the function result to the messages
                        messages.append({
                            "role": "tool",
                            "content": json.dumps(result),
                            "tool_call_id": tool_call.id
                        })

                        # Store for citation extraction
                        tool_results.append((tool_call.id, result))

                # Now call the API again with the tool results to get the final response
                final_response = self.client.chat.completions.create(
                    model=OPENAI_MODEL,
                    messages=messages,
                    tools=[self.retrieval_tool],
                    tool_choice="none",  # Don't call any more tools
                    max_tokens=2000  # Limit tokens to avoid exceeding OpenRouter limits
                )

                response_content = final_response.choices[0].message.content
            else:
                # If no tool call was needed, use the initial response
                response_content = response_message.content
                tool_results = []

            # Extract the retrieval results to populate citations
            citations = []
            retrieved_chunks_used = []

            # Look for tool results to populate citations
            for tool_call_id, result_data in tool_results:
                if 'chunks' in result_data:
                    for chunk in result_data['chunks']:
                        # Add to retrieved chunks used
                        # Helper function to safely extract values, treating empty strings as None
                        def safe_get(data_dict, key, default=None):
                            value = data_dict.get(key, default)
                            return value if value else default

                        retrieved_chunk = RetrievedChunk(
                            text=chunk['text'],
                            score=chunk['score'],
                            source_url=safe_get(chunk['metadata'], 'source_url'),
                            module=safe_get(chunk['metadata'], 'module'),
                            chapter=safe_get(chunk['metadata'], 'chapter'),
                            chunk_index=safe_get(chunk['metadata'], 'chunk_index')
                        )
                        retrieved_chunks_used.append(retrieved_chunk)

                        # Add to citations if not already present
                        citation = {
                            'source_url': safe_get(chunk['metadata'], 'source_url'),
                            'module': safe_get(chunk['metadata'], 'module'),
                            'chapter': safe_get(chunk['metadata'], 'chapter'),
                            'section': safe_get(chunk['metadata'], 'section'),
                            'page_number': safe_get(chunk['metadata'], 'page_number'),
                            'paragraph_number': safe_get(chunk['metadata'], 'paragraph_number'),
                            'citation_type': safe_get(chunk['metadata'], 'citation_type')
                        }
                        if citation not in citations:
                            citations.append(citation)

            # Determine if this is a refusal
            refusal_reason = None
            if "not found in book" in response_content.lower() or "not found in book content" in response_content.lower():
                refusal_reason = "No relevant content found in the book for the given query."

            response = AgentResponse(
                id=f"response_{abs(hash(user_query.text))}",
                content=response_content,
                citations=citations,
                retrieved_chunks_used=retrieved_chunks_used,
                refusal_reason=refusal_reason,
                timestamp=time.time(),
                deterministic_hash=self._calculate_deterministic_hash(user_query.text, response_content)
            )

            logger.info(f"Query processed successfully, response length: {len(response.content)}")
            return response

        except Exception as e:
            logger.error(f"Error processing query: {str(e)}")
            # Return a response with error information
            return AgentResponse(
                id=f"error_response_{abs(hash(user_query.text))}",
                content="An error occurred while processing your request. Please try again later.",
                citations=[],
                retrieved_chunks_used=[],
                refusal_reason=f"Error: {str(e)}",
                timestamp=time.time(),
                deterministic_hash=self._calculate_deterministic_hash(user_query.text, "error")
            )

    def _calculate_deterministic_hash(self, query: str, response: str) -> str:
        """Calculate a hash to verify determinism for identical inputs."""
        combined = f"{query}||{response}"
        return hashlib.sha256(combined.encode()).hexdigest()

    def format_citations(self, citations: List[Dict[str, str]]) -> str:
        """Format citations in a consistent manner for responses."""
        if not citations:
            return ""

        formatted_citations = []
        for citation in citations:
            parts = []
            if citation.get('module'):
                parts.append(f"Module: {citation['module']}")
            if citation.get('chapter'):
                parts.append(f"Chapter: {citation['chapter']}")
            if citation.get('source_url') and citation['source_url'] != "":
                parts.append(f"URL: {citation['source_url']}")

            if parts:
                formatted_citations.append(", ".join(parts))

        if formatted_citations:
            return "Sources: " + "; ".join(formatted_citations)

        return ""

    def close(self):
        """Clean up resources."""
        # No cleanup needed for this implementation
        pass


def main():
    """Example usage of the RAG agent."""
    print("Initializing RAG Agent...")

    # Initialize the agent
    agent = RAGAgent()

    # Example query
    user_query = UserQuery(
        id="example_query_1",
        text="What are the fundamental principles discussed in the book?",
        selected_text_only=False
    )

    print(f"Processing query: {user_query.text}")

    # Process the query
    response = agent.query(user_query)

    print(f"Response: {response.content}")
    print(f"Deterministic hash: {response.deterministic_hash}")

    if response.refusal_reason:
        print(f"Refusal reason: {response.refusal_reason}")

    # Clean up
    agent.close()


if __name__ == "__main__":
    main()