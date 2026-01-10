"""
Test script for citation functionality.
"""

import sys
import os
from typing import Dict, Any
import asyncio

# Add the backend directory to the path so we can import our modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '.'))

from backend.models.query import QueryRequest, SelectedTextQueryRequest, Citation
from backend.services.rag_service import RAGService


def test_citation_structure():
    """
    Test that citations have the expected structure with enhanced metadata fields.
    """
    print("Testing citation structure with enhanced metadata fields...")

    try:
        # Create a citation with all enhanced fields
        citation = Citation(
            source_url="/docs/test/chapter1",
            module="Test Module",
            chapter="Test Chapter",
            section="Test Section",
            page_number=42,
            paragraph_number=3,
            citation_type="definition"
        )

        # Validate that all fields are present
        assert citation.source_url == "/docs/test/chapter1"
        assert citation.module == "Test Module"
        assert citation.chapter == "Test Chapter"
        assert citation.section == "Test Section"
        assert citation.page_number == 42
        assert citation.paragraph_number == 3
        assert citation.citation_type == "definition"

        print(f"✓ Citation structure test passed")
        print(f"  All enhanced fields present: source_url, module, chapter, section, page_number, paragraph_number, citation_type")

        return True

    except Exception as e:
        print(f"✗ Error testing citation structure: {str(e)}")
        import traceback
        traceback.print_exc()
        return False


def test_citation_with_minimal_fields():
    """
    Test that citations work with minimal required fields.
    """
    print("\nTesting citation with minimal required fields...")

    try:
        # Create a citation with only required fields
        citation = Citation(
            source_url="/docs/test/chapter2",
            module="Test Module",
            chapter="Test Chapter"
        )

        # Validate required fields
        assert citation.source_url == "/docs/test/chapter2"
        assert citation.module == "Test Module"
        assert citation.chapter == "Test Chapter"

        # Validate optional fields are None
        assert citation.section is None
        assert citation.page_number is None
        assert citation.paragraph_number is None
        assert citation.citation_type is None

        print(f"✓ Minimal citation test passed")
        print(f"  Required fields present, optional fields are None")

        return True

    except Exception as e:
        print(f"✗ Error testing minimal citation: {str(e)}")
        import traceback
        traceback.print_exc()
        return False


def test_query_response_citation_integration():
    """
    Test that QueryResponse properly handles citations with enhanced fields.
    """
    print("\nTesting QueryResponse citation integration...")

    try:
        # Initialize the RAG service
        rag_service = RAGService()

        # Create a sample query request
        query_request = QueryRequest(
            question="Test citation functionality",
            session_id="test_session_citation"
        )

        # Process the query to get a response
        response = rag_service.process_query(query_request)

        # Validate that response has citations
        assert hasattr(response, 'citations')
        assert isinstance(response.citations, list)

        print(f"✓ QueryResponse citation integration test passed")
        print(f"  Response contains {len(response.citations)} citations")

        rag_service.close()
        return True

    except Exception as e:
        print(f"✗ Error testing QueryResponse citation integration: {str(e)}")
        import traceback
        traceback.print_exc()
        return False


def test_various_query_types():
    """
    Test citation functionality with various query types.
    """
    print("\nTesting citation functionality with various query types...")

    try:
        # Initialize the RAG service
        rag_service = RAGService()

        query_types = [
            ("What are definitions?", "general_definition"),
            ("Explain concepts", "general_concept"),
            ("Show examples", "general_example")
        ]

        results = []
        for query_text, query_type in query_types:
            query_request = QueryRequest(
                question=query_text,
                session_id=f"test_session_{query_type}"
            )

            response = rag_service.process_query(query_request)

            result = {
                'query': query_text,
                'citation_count': len(response.citations),
                'has_citations': len(response.citations) > 0
            }
            results.append(result)

            print(f"  Query '{query_text}': {len(response.citations)} citations")

        rag_service.close()

        print(f"✓ Various query types test passed")
        print(f"  Tested {len(results)} different query types")

        return True

    except Exception as e:
        print(f"✗ Error testing various query types: {str(e)}")
        import traceback
        traceback.print_exc()
        return False


def main():
    """
    Main function to run all citation functionality tests.
    """
    print("Running citation functionality tests...\n")

    # Test citation structure
    structure_ok = test_citation_structure()

    # Test minimal citation
    minimal_ok = test_citation_with_minimal_fields()

    # Test QueryResponse integration
    integration_ok = test_query_response_citation_integration()

    # Test various query types
    query_types_ok = test_various_query_types()

    print(f"\nTest Results:")
    print(f"  Citation Structure: {'✓ PASS' if structure_ok else '✗ FAIL'}")
    print(f"  Minimal Citation: {'✓ PASS' if minimal_ok else '✗ FAIL'}")
    print(f"  Integration: {'✓ PASS' if integration_ok else '✗ FAIL'}")
    print(f"  Various Query Types: {'✓ PASS' if query_types_ok else '✗ FAIL'}")

    overall_success = structure_ok and minimal_ok and integration_ok and query_types_ok

    print(f"  Overall: {'✓ PASS' if overall_success else '✗ FAIL'}")

    if not overall_success:
        print("\nSome tests failed. Please check the output above for details.")
        return 1
    else:
        print("\nAll citation functionality tests passed!")
        return 0


if __name__ == "__main__":
    exit_code = main()
    sys.exit(exit_code)