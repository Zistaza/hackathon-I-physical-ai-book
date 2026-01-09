#!/usr/bin/env python3
"""
Test the retrieval functionality directly to see if content exists in the vector store
"""
import os
import sys
from backend.agent import RAGRetriever

def test_direct_retrieval():
    """Test retrieval directly from the vector store"""

    print("Testing direct retrieval from vector store...")

    # Initialize the retriever
    retriever = RAGRetriever()

    # Test queries that might match content
    test_queries = [
        "physical ai",
        "digital twin",
        "robot",
        "book",
        "ai",
        "learning"
    ]

    for query in test_queries:
        print(f"\n--- Query: '{query}' ---")

        result = retriever.retrieve_content(query, top_k=3)

        print(f"Retrieved {len(result['chunks'])} chunks")
        print(f"Empty retrieval: {result.get('empty_retrieval', True)}")

        if 'error' in result:
            print(f"Error: {result['error']}")

        for i, chunk in enumerate(result['chunks'][:2]):  # Show first 2 chunks
            print(f"  Chunk {i+1}: Score={chunk['score']:.3f}")
            print(f"    Text: {chunk['text'][:200]}...")
            print(f"    Metadata: {chunk['metadata']}")
            print()

if __name__ == "__main__":
    test_direct_retrieval()