"""
Verification script to test connection to the existing retrieval system.
"""

import os
import sys
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Add the backend directory to the path so we can import our agent
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

try:
    from backend.agent import RAGRetriever

    def verify_retrieval_system():
        """Verify that we can connect to the existing retrieval system."""
        print("Verifying connection to the retrieval system...")

        try:
            # Initialize the retriever
            retriever = RAGRetriever()

            # Test a simple retrieval
            print("Testing retrieval with a sample query...")
            result = retriever.retrieve_content(
                query="What is the book about?",
                top_k=3
            )

            print(f"Retrieval successful!")
            print(f"Number of chunks retrieved: {len(result['chunks'])}")
            print(f"Empty retrieval: {result['empty_retrieval']}")
            print(f"Conflicting info: {result['conflicting_info']}")

            if 'error' in result:
                print(f"Error occurred: {result['error']}")
                return False

            if len(result['chunks']) > 0:
                print("Sample chunk preview:")
                first_chunk = result['chunks'][0]
                print(f"  Text preview: {first_chunk['text'][:100]}...")
                print(f"  Score: {first_chunk['score']}")
                print(f"  Metadata: {first_chunk['metadata']}")

            return True

        except Exception as e:
            print(f"Error connecting to retrieval system: {str(e)}")
            return False

    if __name__ == "__main__":
        success = verify_retrieval_system()
        if success:
            print("\n✓ Connection to retrieval system verified successfully!")
        else:
            print("\n✗ Failed to verify connection to retrieval system.")
            sys.exit(1)

except ImportError as e:
    print(f"Error importing RAGRetriever: {str(e)}")
    print("Make sure backend/agent.py exists and contains the RAGRetriever class.")
    sys.exit(1)