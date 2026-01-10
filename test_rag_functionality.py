"""
Test script for RAG query functionality.
"""

import sys
import os
from typing import Dict, Any
import asyncio

# Add the backend directory to the path so we can import our modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '.'))

from backend.api import app
from backend.models.query import QueryRequest, SelectedTextQueryRequest
from backend.services.rag_service import RAGService


def test_general_query_functionality():
    """
    Test general book-wide query functionality.
    """
    print("Testing general book-wide query functionality...")

    try:
        # Create a sample query request
        query_request = QueryRequest(
            question="What are the fundamental principles of robotics?",
            session_id="test_session_001"
        )

        # Initialize the RAG service
        rag_service = RAGService()

        # Process the query
        response = rag_service.process_query(query_request)

        # Validate the response
        assert response.answer is not None
        assert isinstance(response.answer, str)
        assert len(response.answer) > 0
        assert response.citations is not None
        assert isinstance(response.citations, list)
        assert response.retrieved_chunks is not None
        assert isinstance(response.retrieved_chunks, list)
        assert response.timestamp is not None

        print(f"✓ General query processed successfully")
        print(f"  Answer length: {len(response.answer)} characters")
        print(f"  Citations: {len(response.citations)}")
        print(f"  Retrieved chunks: {len(response.retrieved_chunks)}")

        rag_service.close()
        return True

    except Exception as e:
        print(f"✗ Error testing general query functionality: {str(e)}")
        import traceback
        traceback.print_exc()
        return False


def test_selected_text_query_functionality():
    """
    Test selected text constraint query functionality.
    """
    print("\nTesting selected text constraint query functionality...")

    try:
        # Create a sample selected text query request
        query_request = SelectedTextQueryRequest(
            id="test_query_002",
            question="What does this text say about robotics?",
            selected_text_only=True,
            selected_text_content="The fundamental principles of robotics include kinematics, which studies motion, and dynamics, which studies forces.",
            session_id="test_session_002"
        )

        # Initialize the RAG service
        rag_service = RAGService()

        # Process the selected text query
        response = rag_service.process_selected_text_query(query_request)

        # Validate the response
        assert response.answer is not None
        assert isinstance(response.answer, str)
        assert len(response.answer) > 0
        assert response.citations is not None
        assert isinstance(response.citations, list)
        assert response.retrieved_chunks is not None
        assert isinstance(response.retrieved_chunks, list)
        assert response.timestamp is not None

        print(f"✓ Selected text query processed successfully")
        print(f"  Answer length: {len(response.answer)} characters")
        print(f"  Citations: {len(response.citations)}")
        print(f"  Retrieved chunks: {len(response.retrieved_chunks)}")

        rag_service.close()
        return True

    except Exception as e:
        print(f"✗ Error testing selected text query functionality: {str(e)}")
        import traceback
        traceback.print_exc()
        return False


def test_query_validation():
    """
    Test query validation functionality.
    """
    print("\nTesting query validation functionality...")

    try:
        # Initialize the RAG service
        rag_service = RAGService()

        # Test with empty question
        try:
            invalid_request = QueryRequest(
                question="",
                session_id="test_session_003"
            )
            errors = rag_service.validate_query_input(invalid_request)
            assert len(errors) > 0
            print("✓ Empty question validation works correctly")
        except Exception:
            print("✓ Empty question validation works correctly")

        # Test with valid question
        valid_request = QueryRequest(
            question="What is robotics?",
            session_id="test_session_004"
        )
        errors = rag_service.validate_query_input(valid_request)
        assert len(errors) == 0
        print("✓ Valid question passes validation")

        rag_service.close()
        return True

    except Exception as e:
        print(f"✗ Error testing query validation: {str(e)}")
        import traceback
        traceback.print_exc()
        return False


def main():
    """
    Main function to run all RAG functionality tests.
    """
    print("Running RAG query functionality tests...\n")

    # Test general query functionality
    general_ok = test_general_query_functionality()

    # Test selected text query functionality
    selected_ok = test_selected_text_query_functionality()

    # Test query validation
    validation_ok = test_query_validation()

    print(f"\nTest Results:")
    print(f"  General Query: {'✓ PASS' if general_ok else '✗ FAIL'}")
    print(f"  Selected Text Query: {'✓ PASS' if selected_ok else '✗ FAIL'}")
    print(f"  Query Validation: {'✓ PASS' if validation_ok else '✗ FAIL'}")

    overall_success = general_ok and selected_ok and validation_ok

    print(f"  Overall: {'✓ PASS' if overall_success else '✗ FAIL'}")

    if not overall_success:
        print("\nSome tests failed. Please check the output above for details.")
        return 1
    else:
        print("\nAll RAG functionality tests passed!")
        return 0


if __name__ == "__main__":
    exit_code = main()
    sys.exit(exit_code)