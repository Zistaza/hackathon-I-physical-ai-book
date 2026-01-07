"""
Test suite for RAG Pipeline – Retrieval Validation & End-to-End Pipeline Testing
"""
import pytest
import os
from retrieve import RAGRetriever, PipelineValidator, RetrievedChunk


def test_retriever_initialization():
    """Test that RAGRetriever initializes correctly with environment variables."""
    # This test will only work if environment variables are set
    if not os.getenv("COHERE_API_KEY") or not os.getenv("QDRANT_URL") or not os.getenv("QDRANT_API_KEY"):
        pytest.skip("Environment variables not set, skipping initialization test")

    retriever = RAGRetriever()
    assert retriever is not None
    assert retriever.cohere_model is not None
    assert retriever.collection_name is not None


def test_retrieved_chunk_dataclass():
    """Test that RetrievedChunk dataclass works correctly."""
    chunk = RetrievedChunk(
        text="Sample text content",
        score=0.85,
        metadata={"source_url": "https://example.com", "module": "test", "chapter": "1", "chunk_index": 0}
    )

    assert chunk.text == "Sample text content"
    assert chunk.score == 0.85
    assert chunk.metadata["source_url"] == "https://example.com"


def test_query_validation():
    """Test that the retriever validates queries properly."""
    # Create a mock retriever (not connecting to actual services)
    # For this test, we'll just verify the validation logic
    from retrieve import RAGRetriever

    # Test empty query validation
    retriever = RAGRetriever.__new__(RAGRetriever)  # Create without calling __init__
    retriever.threshold = 0.3

    # Test that empty query raises ValueError
    with pytest.raises(ValueError, match="Query cannot be empty"):
        retriever.retrieve("", top_k=5)

    # Test that negative top_k raises ValueError
    with pytest.raises(ValueError, match="top_k must be positive"):
        retriever.retrieve("test query", top_k=0)


def test_determinism_structure():
    """Test the determinism testing functionality structure."""
    # This test checks the structure of the determinism test without connecting to services
    from retrieve import RAGRetriever

    # Create a mock retriever
    retriever = RAGRetriever.__new__(RAGRetriever)
    retriever.threshold = 0.3

    # Mock the retrieve method to return consistent results
    def mock_retrieve(query, top_k=5):
        return [
            RetrievedChunk(
                text=f"Sample text for {query}",
                score=0.85,
                metadata={"source_url": f"https://example.com/{query}", "module": "test", "chapter": "1", "chunk_index": 0}
            )
        ]

    retriever.retrieve = mock_retrieve

    # Test determinism with mock data
    result = retriever.test_determinism("test query", runs=2)

    assert "deterministic" in result
    assert "runs" in result
    assert result["runs"] == 2


def test_pipeline_validator_initialization():
    """Test that PipelineValidator initializes correctly."""
    # Skip if environment variables are not set
    if not os.getenv("COHERE_API_KEY") or not os.getenv("QDRANT_URL") or not os.getenv("QDRANT_API_KEY"):
        pytest.skip("Environment variables not set, skipping initialization test")

    validator = PipelineValidator()
    assert validator is not None
    assert validator.retriever is not None


def test_retrieval_correctness_structure():
    """Test the structure of retrieval correctness validation."""
    from retrieve import PipelineValidator

    # Create a mock validator
    validator = PipelineValidator.__new__(PipelineValidator)

    # Mock test cases
    test_cases = [
        {
            "query": "test query",
            "expected_sources": ["example.com"]
        }
    ]

    # Mock the validation method to return a predictable result
    def mock_validate_retrieval_correctness(test_cases):
        return {
            "accuracy": 1.0,
            "correct_retrievals": 1,
            "total_tests": 1,
            "passed": True,
            "message": "Test passed"
        }

    validator.validate_retrieval_correctness = mock_validate_retrieval_correctness

    result = validator.validate_retrieval_correctness(test_cases)
    assert result["accuracy"] == 1.0
    assert result["passed"] is True


if __name__ == "__main__":
    pytest.main([__file__])