"""
Test script to verify selected-text-only mode functionality.
"""

import sys
import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Add the project directory to the path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from backend.agent import RAGAgent, UserQuery

def test_selected_text_mode():
    """Test the selected-text-only mode functionality."""
    print("Testing selected-text-only mode...")

    try:
        # Initialize the agent
        print("Initializing RAG Agent...")
        agent = RAGAgent()

        # Create a test query with selected text only mode
        sample_text = "Robots are machines designed to execute one or more tasks automatically."
        test_query = UserQuery(
            id="test_selected_text_1",
            text="What are robots?",
            selected_text_only=True,
            selected_text_content=sample_text
        )

        print(f"Sending query in selected-text-only mode: {test_query.text}")
        print(f"Selected text: {sample_text}")

        # Process the query
        response = agent.query(test_query)

        print("Query processed successfully!")
        print(f"Response received: {response.content}")
        print(f"Citations: {response.citations}")
        print(f"Retrieved chunks used: {len(response.retrieved_chunks_used)}")
        print(f"Is refusal: {'Yes' if response.refusal_reason else 'No'}")

        if response.refusal_reason:
            print(f"Refusal reason: {response.refusal_reason}")

        # Clean up
        agent.close()

        print("\n✓ Selected-text-only mode test completed successfully!")
        return True

    except Exception as e:
        print(f"✗ Error during selected-text-only mode test: {str(e)}")
        import traceback
        traceback.print_exc()
        return False

def test_full_book_mode():
    """Test the full book mode functionality for comparison."""
    print("\nTesting full book mode for comparison...")

    try:
        # Initialize the agent
        print("Initializing RAG Agent...")
        agent = RAGAgent()

        # Create a test query in full book mode
        test_query = UserQuery(
            id="test_full_book_1",
            text="What is robotics?",
            selected_text_only=False
        )

        print(f"Sending query in full book mode: {test_query.text}")

        # Process the query
        response = agent.query(test_query)

        print("Query processed successfully!")
        print(f"Response received: {response.content[:100]}...")
        print(f"Citations: {response.citations}")
        print(f"Retrieved chunks used: {len(response.retrieved_chunks_used)}")
        print(f"Is refusal: {'Yes' if response.refusal_reason else 'No'}")

        # Clean up
        agent.close()

        print("\n✓ Full book mode test completed successfully!")
        return True

    except Exception as e:
        print(f"✗ Error during full book mode test: {str(e)}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    success1 = test_selected_text_mode()
    success2 = test_full_book_mode()

    if success1 and success2:
        print("\n✓ All mode tests completed successfully!")
    else:
        sys.exit(1)