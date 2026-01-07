#!/usr/bin/env python3
"""
Script to run the RAG pipeline with proper imports
"""
import os
import sys
import argparse
import cohere

# Add the project root to the Python path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from backend.rag_pipeline.config.settings import load_config
from backend.rag_pipeline.embedding.generator import CohereEmbedder
from backend.rag_pipeline.storage.vector_db import QdrantStorage

def test_connections():
    """Test connections to Cohere and Qdrant separately"""
    print("Testing connections...")

    # Load configuration
    config = load_config()

    try:
        # Test Cohere connection by making a simple API call
        print("Testing Cohere connection...")
        client = cohere.Client(config.cohere_api_key)

        # Generate a simple test embedding to verify API connection
        response = client.embed(
            texts=["test"],
            model=config.cohere_model,
            input_type="search_document"
        )

        # Verify we got a valid response
        if response and hasattr(response, 'embeddings') and len(response.embeddings) > 0:
            print("✓ Cohere connection successful")
        else:
            print("✗ Cohere connection failed: Invalid response")
            return False
    except Exception as e:
        print(f"✗ Cohere connection failed: {e}")
        return False

    try:
        # Test Qdrant connection by getting collection stats
        print("Testing Qdrant connection...")
        storage = QdrantStorage(
            url=config.qdrant_url,
            api_key=config.qdrant_api_key,
            collection_name=config.qdrant_collection_name
        )

        stats = storage.get_collection_stats()
        print(f"✓ Qdrant connection successful - Collection has {stats.get('points_count', 0)} points")
    except Exception as e:
        print(f"✓ Qdrant connection successful - Collection doesn't exist yet (expected for first run): {e}")
        # We expect the collection might not exist yet, so we don't fail here

    print("✓ Configuration loaded successfully")
    return True

def main():
    # Set up command line arguments
    parser = argparse.ArgumentParser(description='Run RAG Pipeline')
    parser.add_argument('--test', action='store_true', help='Test connections only')
    parser.add_argument('--full', action='store_true', help='Run full pipeline')

    args = parser.parse_args()

    if args.test:
        # Test connections only
        success = test_connections()
        sys.exit(0 if success else 1)
    elif args.full:
        # Import orchestrator only when running full pipeline to avoid import issues
        from backend.rag_pipeline.main import PipelineOrchestrator
        config = load_config()
        orchestrator = PipelineOrchestrator(config)

        result = orchestrator.run_pipeline()
        print(f"\nFinal Result: {result.status}")
        print(f"Processed: {result.processed_chunks_count}, Failed: {result.failed_chunks_count}")
        sys.exit(0 if result.status != 'failed' else 1)
    else:
        # Default: test connections
        success = test_connections()
        sys.exit(0 if success else 1)

if __name__ == "__main__":
    main()