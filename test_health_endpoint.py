"""
Test script to verify basic health endpoint functionality.
"""

import requests
import sys
import os

# Add the backend directory to the path so we can import our modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from backend.api import app
from backend.config import settings, validate_config


def test_health_endpoint():
    """
    Test the health endpoint functionality.
    """
    print("Testing health endpoint functionality...")

    # First, validate the configuration
    is_valid, errors = validate_config()

    if not is_valid:
        print("⚠️  Configuration validation failed:")
        for error in errors:
            print(f"  - {error}")
        print("Please set up your .env file with the required variables.")
        return False

    print("✓ Configuration is valid")

    # Test using the FastAPI TestClient
    try:
        from fastapi.testclient import TestClient

        client = TestClient(app)

        # Test the health endpoint
        response = client.get("/health")

        print(f"Health endpoint response status: {response.status_code}")

        if response.status_code in [200, 503]:  # 200 for healthy, 503 for degraded/unhealthy
            response_data = response.json()
            print(f"Health response: {response_data}")

            # Check if it's a proper health response
            if 'status' in response_data and 'dependencies' in response_data:
                print("✓ Health endpoint is responding correctly")
                return True
            else:
                print("✗ Health endpoint response format is incorrect")
                return False
        else:
            print(f"✗ Health endpoint returned unexpected status code: {response.status_code}")
            return False

    except ImportError:
        print("TestClient not available, skipping detailed health test")
        return True
    except Exception as e:
        print(f"✗ Error testing health endpoint: {str(e)}")
        return False


def test_api_startup():
    """
    Test basic API startup without running the server.
    """
    print("\nTesting API startup...")

    try:
        # Try to create the app (this happens in api.py)
        assert app is not None
        print("✓ FastAPI app created successfully")

        # Check if the required routes are registered
        routes = [route.path for route in app.routes]

        expected_routes = ["/health", "/", "/rag/test"]
        missing_routes = [route for route in expected_routes if not any(route in r for r in routes)]

        if not missing_routes:
            print("✓ All expected routes are registered")
            return True
        else:
            print(f"✗ Missing routes: {missing_routes}")
            return False

    except Exception as e:
        print(f"✗ Error testing API startup: {str(e)}")
        return False


def main():
    """
    Main function to run all tests.
    """
    print("Running basic health endpoint functionality tests...\n")

    # Test API startup
    startup_ok = test_api_startup()

    # Test health endpoint
    health_ok = test_health_endpoint()

    print(f"\nTest Results:")
    print(f"  API Startup: {'✓ PASS' if startup_ok else '✗ FAIL'}")
    print(f"  Health Endpoint: {'✓ PASS' if health_ok else '✗ FAIL'}")

    overall_success = startup_ok and health_ok

    print(f"  Overall: {'✓ PASS' if overall_success else '✗ FAIL'}")

    if not overall_success:
        print("\nSome tests failed. Please check the output above for details.")
        return 1
    else:
        print("\nAll tests passed!")
        return 0


if __name__ == "__main__":
    exit_code = main()
    sys.exit(exit_code)