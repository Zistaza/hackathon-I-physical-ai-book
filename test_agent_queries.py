#!/usr/bin/env python3
"""
Test the RAG agent with specific queries to verify retrieval is working
"""
import os
import sys
from backend.agent import RAGAgent, UserQuery

def test_specific_queries():
    """Test the agent with specific queries that should match content in the vector store"""

    print("Initializing RAG Agent...")
    agent = RAGAgent()

    # Test queries that might match content in the book
    test_queries = [
        "What is the physical AI book about?",
        "Explain digital twin technology",
        "What are VLA robots?",
        "Describe the book content",
        "What topics does this book cover?"
    ]

    for i, query_text in enumerate(test_queries, 1):
        print(f"\n--- Test Query {i}: {query_text} ---")

        user_query = UserQuery(
            id=f"test_query_{i}",
            text=query_text,
            selected_text_only=False
        )

        response = agent.query(user_query)

        print(f"Response length: {len(response.content)}")
        if response.content.strip():
            print(f"Response: {response.content[:500]}...")  # First 500 chars
        else:
            print("No content retrieved")

        print(f"Citations: {len(response.citations)} found")
        for citation in response.citations:
            print(f"  - {citation}")

        if response.refusal_reason:
            print(f"Refusal reason: {response.refusal_reason}")

    agent.close()

if __name__ == "__main__":
    test_specific_queries()