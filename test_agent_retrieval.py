"""
Test script to verify basic communication between agent and retrieval system.
"""

import sys
import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Add the project directory to the path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from backend.agent import RAGAgent, UserQuery

def test_agent_retrieval_communication():
    """Test basic communication between agent and retrieval system."""
    print("Testing agent and retrieval system communication...")

    try:
        # Initialize the agent
        print("Initializing RAG Agent...")
        agent = RAGAgent()

        print("Agent initialized successfully!")

        # Create a test query
        test_query = UserQuery(
            id="test_query_1",
            text="What is the book about?",
            selected_text_only=False
        )

        print(f"Sending query: {test_query.text}")

        # Process the query
        response = agent.query(test_query)

        print("Query processed successfully!")
        print(f"Response received: {response.content[:100]}...")
        print(f"Is refusal: {'Yes' if response.refusal_reason else 'No'}")
        print(f"Deterministic hash: {response.deterministic_hash}")

        # Clean up
        agent.close()

        print("\n✓ Communication test completed successfully!")
        return True

    except Exception as e:
        print(f"✗ Error during communication test: {str(e)}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    success = test_agent_retrieval_communication()
    if not success:
        sys.exit(1)