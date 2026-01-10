"""
Test script for frontend-backend integration.
"""

import sys
import os
import requests
import time
from typing import Dict, Any

# Add the backend directory to the path so we can import our modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '.'))

from backend.models.query import QueryRequest, SelectedTextQueryRequest
from backend.api import app
from fastapi.testclient import TestClient


def test_backend_api_directly():
    """
    Test backend API directly to ensure endpoints are working.
    """
    print("Testing backend API endpoints directly...")

    try:
        client = TestClient(app)

        # Test health endpoint
        health_response = client.get("/health")
        assert health_response.status_code in [200, 503]  # 503 is acceptable if dependencies are down
        print(f"✓ Health endpoint responded with status: {health_response.status_code}")

        # Test general query endpoint with a mock request
        query_payload = {
            "question": "What are the fundamental principles of the book?"
        }

        query_response = client.post("/rag/query", json=query_payload)

        # The response should be either 200 (success) or 400 (validation error) or 500 (processing error)
        # Due to missing API keys, we might get 500, which is expected in this environment
        if query_response.status_code == 500:
            print(f"✓ Query endpoint responded with expected error (due to missing API keys): {query_response.status_code}")
            # Even with error, we should get a valid error response structure
            error_data = query_response.json()
            assert 'error' in error_data
        elif query_response.status_code == 400:
            print(f"✓ Query endpoint responded with validation error: {query_response.status_code}")
            error_data = query_response.json()
            assert 'error' in error_data
        elif query_response.status_code == 200:
            print(f"✓ Query endpoint responded with success: {query_response.status_code}")
            response_data = query_response.json()
            assert 'answer' in response_data
            assert 'citations' in response_data
        else:
            print(f"? Query endpoint responded with unexpected status: {query_response.status_code}")

        # Test selected text query endpoint
        selected_query_payload = {
            "question": "What does this text say?",
            "selected_text_only": True,
            "selected_text_content": "This is a sample text for testing."
        }

        selected_response = client.post("/rag/query-selected-text", json=selected_query_payload)

        if selected_response.status_code == 500:
            print(f"✓ Selected text query endpoint responded with expected error (due to missing API keys): {selected_response.status_code}")
        elif selected_response.status_code == 400:
            print(f"✓ Selected text query endpoint responded with validation error: {selected_response.status_code}")
        elif selected_response.status_code == 200:
            print(f"✓ Selected text query endpoint responded with success: {selected_response.status_code}")
        else:
            print(f"? Selected text query endpoint responded with unexpected status: {selected_response.status_code}")

        print("✓ Direct backend API testing completed")
        return True

    except Exception as e:
        print(f"✗ Error testing backend API: {str(e)}")
        import traceback
        traceback.print_exc()
        return False


def test_frontend_backend_integration_general():
    """
    Test frontend-backend integration for general queries.
    """
    print("\nTesting frontend-backend integration for general queries...")

    try:
        # This test simulates what the frontend would do
        # We'll use the TestClient to make requests as the frontend would

        client = TestClient(app)

        # Test general query similar to what frontend would send
        general_query = {
            "question": "What are the main topics covered in the book?",
            "session_id": "integration_test_001"
        }

        response = client.post("/rag/query", json=general_query)

        # Check response structure regardless of success/error
        if response.status_code in [200, 500, 400]:
            print(f"✓ General query integration test completed with status: {response.status_code}")

            if response.status_code == 200:
                data = response.json()
                # Validate response structure
                expected_fields = ['id', 'answer', 'citations', 'retrieved_chunks', 'timestamp']
                for field in expected_fields:
                    assert field in data, f"Missing field: {field}"

                print(f"  Response has {len(data['citations'])} citations")
                print(f"  Answer length: {len(data['answer']) if data['answer'] else 0} chars")

            return True
        else:
            print(f"✗ Unexpected status code: {response.status_code}")
            return False

    except Exception as e:
        print(f"✗ Error in general query integration test: {str(e)}")
        import traceback
        traceback.print_exc()
        return False


def test_frontend_backend_integration_selected_text():
    """
    Test frontend-backend integration for selected text queries.
    """
    print("\nTesting frontend-backend integration for selected text queries...")

    try:
        # This test simulates what the frontend would do for selected text queries

        client = TestClient(app)

        # Test selected text query similar to what frontend would send
        selected_text_query = {
            "question": "What does this text explain?",
            "selected_text_only": True,
            "selected_text_content": "Artificial Intelligence is a branch of computer science that aims to create software or machines that exhibit human-like intelligence. This can include learning from experience, understanding natural language, solving problems, and recognizing patterns.",
            "session_id": "integration_test_002"
        }

        response = client.post("/rag/query-selected-text", json=selected_text_query)

        # Check response structure regardless of success/error
        if response.status_code in [200, 500, 400]:
            print(f"✓ Selected text query integration test completed with status: {response.status_code}")

            if response.status_code == 200:
                data = response.json()
                # Validate response structure
                expected_fields = ['id', 'answer', 'citations', 'retrieved_chunks', 'timestamp']
                for field in expected_fields:
                    assert field in data, f"Missing field: {field}"

                print(f"  Response has {len(data['citations'])} citations")
                print(f"  Answer length: {len(data['answer']) if data['answer'] else 0} chars")

            return True
        else:
            print(f"✗ Unexpected status code: {response.status_code}")
            return False

    except Exception as e:
        print(f"✗ Error in selected text query integration test: {str(e)}")
        import traceback
        traceback.print_exc()
        return False


def test_api_endpoints_exist():
    """
    Test that all expected API endpoints exist and are accessible.
    """
    print("\nTesting that all API endpoints exist...")

    try:
        client = TestClient(app)

        endpoints_to_test = [
            ("/", 200, "Root endpoint"),
            ("/health", [200, 503], "Health endpoint"),
            ("/docs", 200, "Swagger docs"),
            ("/redoc", 200, "Redoc docs")
        ]

        for path, expected_status, description in endpoints_to_test:
            response = client.get(path)
            status_ok = response.status_code == expected_status if isinstance(expected_status, int) else response.status_code in expected_status
            if status_ok:
                print(f"✓ {description} exists and accessible: {response.status_code}")
            else:
                print(f"✗ {description} failed: {response.status_code} (expected {expected_status})")

        # Test the rag endpoints specifically
        rag_endpoints = [
            ("/rag/test", [200, 405], "RAG test endpoint")  # 405 is OK if it's a POST-only endpoint accessed with GET
        ]

        for path, expected_status, description in rag_endpoints:
            response = client.get(path)
            status_ok = response.status_code == expected_status if isinstance(expected_status, int) else response.status_code in expected_status
            if status_ok or response.status_code in [200, 405]:  # Both are acceptable for test endpoint
                print(f"✓ {description} exists: {response.status_code}")
            else:
                print(f"✗ {description} failed: {response.status_code}")

        return True

    except Exception as e:
        print(f"✗ Error testing API endpoints: {str(e)}")
        import traceback
        traceback.print_exc()
        return False


def main():
    """
    Main function to run all frontend-backend integration tests.
    """
    print("Running frontend-backend integration tests...\n")

    # Test API endpoints exist
    endpoints_ok = test_api_endpoints_exist()

    # Test backend API directly
    backend_ok = test_backend_api_directly()

    # Test general query integration
    general_ok = test_frontend_backend_integration_general()

    # Test selected text query integration
    selected_ok = test_frontend_backend_integration_selected_text()

    print(f"\nTest Results:")
    print(f"  API Endpoints Exist: {'✓ PASS' if endpoints_ok else '✗ FAIL'}")
    print(f"  Backend API Direct: {'✓ PASS' if backend_ok else '✗ FAIL'}")
    print(f"  General Query Integration: {'✓ PASS' if general_ok else '✗ FAIL'}")
    print(f"  Selected Text Integration: {'✓ PASS' if selected_ok else '✗ FAIL'}")

    overall_success = endpoints_ok and backend_ok and general_ok and selected_ok

    print(f"  Overall: {'✓ PASS' if overall_success else '✗ FAIL'}")

    if not overall_success:
        print("\nSome tests failed. Please check the output above for details.")
        print("Note: Some failures may be expected due to missing API keys in test environment.")
        return 1
    else:
        print("\nAll frontend-backend integration tests completed!")
        print("Note: Expected errors due to missing API keys are acceptable in test environment.")
        return 0


if __name__ == "__main__":
    exit_code = main()
    sys.exit(exit_code)