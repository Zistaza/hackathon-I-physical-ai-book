"""
Pydantic models for RAG query requests and responses.

This module defines the data models for RAG query requests and responses
used in the RAG Backend API.
"""

from pydantic import BaseModel, Field, field_validator
from pydantic import model_validator
from typing import List, Dict, Any, Optional
from datetime import datetime


class Citation(BaseModel):
    """
    Represents a source citation for content in the response.

    Fields:
    - source_url: URL of the source document
    - module: Module identifier from the book
    - chapter: Chapter identifier from the book
    - section: Section identifier if available
    - page_number: Page number if available
    - paragraph_number: Paragraph number if available
    - citation_type: Type of citation (e.g., "definition", "theorem", "example")
    """
    source_url: Optional[str] = Field(None, description="URL of the source document")
    module: Optional[str] = Field(None, description="Module identifier from the book")
    chapter: Optional[str] = Field(None, description="Chapter identifier from the book")
    section: Optional[str] = Field(None, description="Section identifier if available")
    page_number: Optional[int] = Field(None, description="Page number if available")
    paragraph_number: Optional[int] = Field(None, description="Paragraph number if available")
    citation_type: Optional[str] = Field(None, description="Type of citation (e.g., 'definition', 'theorem', 'example')")

    class Config:
        json_schema_extra = {
            "example": {
                "source_url": "/docs/fundamentals/principles",
                "module": "Fundamentals",
                "chapter": "Introduction to Robotics",
                "section": "Core Principles",
                "page_number": 15,
                "paragraph_number": 3,
                "citation_type": "definition"
            }
        }


class RetrievedChunk(BaseModel):
    """
    Represents a content chunk retrieved from the vector store.

    Fields:
    - text: The actual text content retrieved
    - score: Similarity score (0.0 to 1.0)
    - source_url: URL of the source document
    - module: Module identifier from the book
    - chapter: Chapter identifier from the book
    - chunk_index: Index of the chunk within the source
    """
    text: str = Field(..., description="The actual text content retrieved")
    score: float = Field(..., ge=0.0, le=1.0, description="Similarity score (0.0 to 1.0)")
    source_url: Optional[str] = Field(None, description="URL of the source document")
    module: Optional[str] = Field(None, description="Module identifier from the book")
    chapter: Optional[str] = Field(None, description="Chapter identifier from the book")
    chunk_index: Optional[int] = Field(None, description="Index of the chunk within the source")

    @field_validator('text')
    @classmethod
    def validate_text_not_empty(cls, v):
        if not v.strip():
            raise ValueError('Text must not be empty')
        return v

    @field_validator('score')
    @classmethod
    def validate_score_range(cls, v):
        if not 0.0 <= v <= 1.0:
            raise ValueError('Score must be between 0.0 and 1.0')
        return v

    class Config:
        json_schema_extra = {
            "example": {
                "text": "The fundamental principles of robotics include kinematics, which studies motion...",
                "score": 0.85,
                "source_url": "/docs/fundamentals/principles",
                "module": "Fundamentals",
                "chapter": "Introduction to Robotics",
                "chunk_index": 1
            }
        }


class QueryRequest(BaseModel):
    """
    Represents a user's question and context from the frontend.

    Fields:
    - id: Unique identifier for the query, generated if not provided
    - question: The user's question text
    - session_id: Session identifier for conversation tracking
    - metadata: Additional metadata for the query
    """
    id: Optional[str] = Field(None, description="Unique identifier for the query, generated if not provided")
    question: str = Field(..., min_length=1, description="The user's question text")
    session_id: Optional[str] = Field(None, description="Session identifier for conversation tracking")
    metadata: Optional[Dict[str, Any]] = Field(None, description="Additional metadata for the query")

    @field_validator('question')
    @classmethod
    def validate_question_not_empty(cls, v):
        if not v.strip():
            raise ValueError('Question must not be empty')
        return v

    class Config:
        json_schema_extra = {
            "example": {
                "id": "query_12345",
                "question": "What are the fundamental principles discussed in the book?",
                "session_id": "session_abc123",
                "metadata": {}
            }
        }


class SelectedTextQueryRequest(QueryRequest):
    """
    Extended query request for queries constrained to selected text.

    Fields:
    - selected_text_only: Whether to restrict search to selected text only
    - selected_text_content: The specific text content to search within when selected_text_only is true
    """
    selected_text_only: bool = Field(default=False, description="Whether to restrict search to selected text only")
    selected_text_content: Optional[str] = Field(None, description="The specific text content to search within when selected_text_only is true")

    @model_validator(mode='after')
    def validate_selected_text_constraint(self):
        if self.selected_text_only and (self.selected_text_content is None or not self.selected_text_content.strip()):
            raise ValueError('selected_text_content must be provided and non-empty when selected_text_only is true')
        return self

    class Config:
        json_schema_extra = {
            "example": {
                "id": "query_12345",
                "question": "What does this text say about robotics?",
                "selected_text_only": True,
                "selected_text_content": "The fundamental principles of robotics include kinematics, dynamics, control theory...",
                "session_id": "session_abc123",
                "metadata": {}
            }
        }


class QueryResponse(BaseModel):
    """
    Represents the response from the RAG agent to the frontend.

    Fields:
    - id: Unique identifier for the response
    - answer: The final answer generated by the RAG agent
    - citations: List of sources used to generate the answer
    - retrieved_chunks: Details of chunks used in the response
    - refusal_reason: Reason if the query was refused (e.g., "not found in book")
    - deterministic_hash: Hash for verifying determinism
    - timestamp: Unix timestamp of response generation
    - session_id: Session identifier if provided in request
    """
    id: str = Field(..., description="Unique identifier for the response")
    answer: str = Field(..., description="The final answer generated by the RAG agent")
    citations: List[Citation] = Field(..., description="List of sources used to generate the answer")
    retrieved_chunks: List[RetrievedChunk] = Field(..., description="Details of chunks used in the response")
    refusal_reason: Optional[str] = Field(None, description="Reason if the query was refused (e.g., 'not found in book')")
    deterministic_hash: Optional[str] = Field(None, description="Hash for verifying determinism")
    timestamp: float = Field(..., description="Unix timestamp of response generation")
    session_id: Optional[str] = Field(None, description="Session identifier if provided in request")

    @field_validator('answer')
    @classmethod
    def validate_answer_not_empty_when_no_refusal(cls, v, values):
        # Only validate if there's no refusal_reason
        if not values.data.get('refusal_reason') and not v.strip():
            raise ValueError('Answer must not be empty when no refusal reason is provided')
        return v

    class Config:
        json_schema_extra = {
            "example": {
                "id": "response_67890",
                "answer": "The fundamental principles include kinematics, dynamics, and control theory...",
                "citations": [
                    {
                        "source_url": "/docs/fundamentals/principles",
                        "module": "Fundamentals",
                        "chapter": "Introduction to Robotics"
                    }
                ],
                "retrieved_chunks": [
                    {
                        "text": "The fundamental principles of robotics include kinematics, which studies motion...",
                        "score": 0.85,
                        "source_url": "/docs/fundamentals/principles",
                        "module": "Fundamentals",
                        "chapter": "Introduction to Robotics",
                        "chunk_index": 1
                    }
                ],
                "refusal_reason": None,
                "deterministic_hash": "a1b2c3d4e5f6...",
                "timestamp": 1678886400.123456,
                "session_id": "session_abc123"
            }
        }


class ErrorResponse(BaseModel):
    """
    Represents an error response from the API.

    Fields:
    - error: Error message
    - code: Error code
    - details: Additional error details
    """
    error: str = Field(..., description="Error message")
    code: str = Field(..., description="Error code")
    details: Optional[Dict[str, Any]] = Field(None, description="Additional error details")

    class Config:
        json_schema_extra = {
            "example": {
                "error": "Invalid query: question must be a non-empty string",
                "code": "INVALID_INPUT",
                "details": {}
            }
        }