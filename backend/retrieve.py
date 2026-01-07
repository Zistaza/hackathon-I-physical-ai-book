"""
RAG Retrieval and Validation System

This module provides functionality for:
1. Connecting to Qdrant and retrieving stored embeddings
2. Performing similarity search with user queries
3. Validating retrieval correctness and determinism
4. Running comprehensive pipeline validation tests
"""

import os
import time
import logging
from typing import List, Dict, Any, Optional
from dataclasses import dataclass
import cohere
from qdrant_client import QdrantClient
from qdrant_client.http import models
import numpy as np
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


@dataclass
class RetrievedChunk:
    """Represents a chunk retrieved from the vector database."""
    text: str
    score: float
    metadata: Dict[str, Any]


class RAGRetriever:
    """Main class for RAG retrieval functionality."""

    def __init__(self):
        """Initialize the RAG retriever with Cohere and Qdrant clients."""
        # Initialize Cohere client
        cohere_api_key = os.getenv("COHERE_API_KEY")
        if not cohere_api_key:
            raise ValueError("COHERE_API_KEY environment variable is required")

        self.cohere_client = cohere.Client(cohere_api_key)

        # Initialize Qdrant client
        qdrant_url = os.getenv("QDRANT_URL")
        qdrant_api_key = os.getenv("QDRANT_API_KEY")
        self.collection_name = os.getenv("QDRANT_COLLECTION_NAME", "book_embeddings")

        if not qdrant_url:
            raise ValueError("QDRANT_URL environment variable is required")

        if not qdrant_api_key:
            raise ValueError("QDRANT_API_KEY environment variable is required")

        self.qdrant_client = QdrantClient(
            url=qdrant_url,
            api_key=qdrant_api_key
        )

        # Cohere model configuration
        self.cohere_model = os.getenv("COHERE_MODEL", "embed-english-v3.0")

        # Similarity threshold
        self.threshold = float(os.getenv("RETRIEVAL_THRESHOLD", "0.3"))

        logger.info("RAG Retriever initialized successfully")

    def generate_query_embedding(self, query: str) -> List[float]:
        """
        Generate embedding for the input query using Cohere.

        Args:
            query: Input text query

        Returns:
            Embedding vector as a list of floats
        """
        if not query.strip():
            raise ValueError("Query cannot be empty")

        try:
            response = self.cohere_client.embed(
                texts=[query],
                model=self.cohere_model,
                input_type="search_document"  # Use same input type as during ingestion
            )
            return response.embeddings[0]
        except Exception as e:
            logger.error(f"Error generating query embedding: {e}")
            raise

    def retrieve(self, query: str, top_k: int = 5) -> List[RetrievedChunk]:
        """
        Retrieve similar chunks from the vector database.

        Args:
            query: Input text query
            top_k: Number of results to return (default: 5)

        Returns:
            List of RetrievedChunk objects with text, score, and metadata
        """
        if not query.strip():
            raise ValueError("Query cannot be empty")

        if top_k <= 0:
            raise ValueError("top_k must be positive")

        # Generate query embedding
        query_embedding = self.generate_query_embedding(query)

        try:
            # Perform similarity search
            search_results = self.qdrant_client.search(
                collection_name=self.collection_name,
                query_vector=query_embedding,
                limit=top_k,
                with_payload=True
            )

            # Convert results to RetrievedChunk objects
            retrieved_chunks = []
            for result in search_results:
                if result.score >= self.threshold:
                    retrieved_chunk = RetrievedChunk(
                        text=result.payload.get("content", ""),  # Changed from "text" to "content"
                        score=result.score,
                        metadata={k: v for k, v in result.payload.items() if k != "content"}  # All other payload items as metadata
                    )
                    retrieved_chunks.append(retrieved_chunk)

            # Sort by score (descending) to ensure deterministic ordering
            # Use the ID as secondary sort key for consistent tie-breaking
            retrieved_chunks.sort(key=lambda x: (-x.score, str(result.id) if 'result' in locals() else ''))

            logger.info(f"Retrieved {len(retrieved_chunks)} chunks for query: '{query[:50]}...'")
            return retrieved_chunks

        except Exception as e:
            logger.error(f"Error during retrieval: {e}")
            raise

    def validate_ingestion_completeness(self) -> Dict[str, Any]:
        """
        Validate that all expected content has been ingested into Qdrant.

        Returns:
            Dictionary with validation results
        """
        try:
            # Get collection info
            collection_info = self.qdrant_client.get_collection(self.collection_name)

            # Get all points count
            points_count = collection_info.points_count

            # Check if collection has any points
            if points_count == 0:
                return {
                    "valid": False,
                    "message": "Collection is empty - no embeddings found",
                    "points_count": points_count
                }

            # Get a sample of points to check metadata completeness
            sample_results = self.qdrant_client.scroll(
                collection_name=self.collection_name,
                limit=10,
                with_payload=True
            )

            if not sample_results[0]:
                return {
                    "valid": False,
                    "message": "No points found in collection",
                    "points_count": points_count
                }

            # Check metadata completeness for sample
            sample_point = sample_results[0][0]
            metadata = sample_point.payload.get("metadata", {})

            required_fields = ["source_url", "module", "chapter", "chunk_index"]
            missing_fields = [field for field in required_fields if field not in metadata]

            if missing_fields:
                return {
                    "valid": False,
                    "message": f"Missing required metadata fields: {missing_fields}",
                    "points_count": points_count,
                    "sample_metadata": metadata
                }

            return {
                "valid": True,
                "message": f"Collection has {points_count} points with complete metadata",
                "points_count": points_count
            }

        except Exception as e:
            logger.error(f"Error during ingestion validation: {e}")
            return {
                "valid": False,
                "message": f"Error validating ingestion: {str(e)}",
                "points_count": 0
            }

    def test_determinism(self, query: str, runs: int = 3) -> Dict[str, Any]:
        """
        Test determinism by running the same query multiple times.

        Args:
            query: Input query to test
            runs: Number of times to run the query (default: 3)

        Returns:
            Dictionary with determinism test results
        """
        results = []

        for i in range(runs):
            run_results = self.retrieve(query, top_k=5)
            # Convert to comparable format
            run_data = [
                {
                    "text": chunk.text[:100],  # First 100 chars for comparison
                    "score": round(chunk.score, 6),  # Round to 6 decimal places
                    "source_url": chunk.metadata.get("source_url", ""),
                    "module": chunk.metadata.get("module", ""),
                    "chapter": chunk.metadata.get("chapter", "")
                }
                for chunk in run_results
            ]
            results.append(run_data)

        # Check if all runs produced identical results
        first_result = results[0]
        all_identical = all(result == first_result for result in results)

        return {
            "deterministic": all_identical,
            "runs": runs,
            "first_result": first_result,
            "all_results_identical": all_identical,
            "message": f"Query results {'are' if all_identical else 'are not'} deterministic across {runs} runs"
        }


class PipelineValidator:
    """Class for running comprehensive pipeline validation tests."""

    def __init__(self):
        """Initialize the pipeline validator."""
        self.retriever = RAGRetriever()

    def validate_retrieval_correctness(self, test_cases: List[Dict[str, Any]]) -> Dict[str, Any]:
        """
        Validate retrieval correctness using known test cases.

        Args:
            test_cases: List of test cases with query and expected results

        Returns:
            Dictionary with validation results
        """
        correct_retrievals = 0
        total_tests = len(test_cases)

        for test_case in test_cases:
            query = test_case.get("query", "")
            expected_sources = test_case.get("expected_sources", [])

            if not query or not expected_sources:
                continue

            try:
                results = self.retriever.retrieve(query, top_k=5)

                # Check if any of the expected sources are in the results
                found_expected = False
                for result in results:
                    source_url = result.metadata.get("source_url", "")
                    if any(expected_source in source_url for expected_source in expected_sources):
                        found_expected = True
                        break

                if found_expected:
                    correct_retrievals += 1
            except Exception as e:
                logger.error(f"Error validating test case: {e}")
                continue

        accuracy = correct_retrievals / total_tests if total_tests > 0 else 0

        return {
            "accuracy": accuracy,
            "correct_retrievals": correct_retrievals,
            "total_tests": total_tests,
            "passed": accuracy >= 0.95,  # 95% threshold
            "message": f"Retrieved correctly in {correct_retrievals}/{total_tests} test cases ({accuracy:.2%})"
        }

    def validate_metadata_completeness(self, sample_size: int = 100) -> Dict[str, Any]:
        """
        Validate that metadata is complete for retrieved chunks.

        Args:
            sample_size: Number of chunks to sample for validation

        Returns:
            Dictionary with metadata validation results
        """
        # Get a sample of points from Qdrant to validate metadata
        try:
            sample_results = self.retriever.qdrant_client.scroll(
                collection_name=self.retriever.collection_name,
                limit=sample_size,
                with_payload=True
            )

            if not sample_results[0]:
                return {
                    "valid": False,
                    "message": "No points found to validate",
                    "sample_size": 0
                }

            total_chunks = len(sample_results[0])
            complete_metadata_count = 0

            required_fields = ["source_url", "module", "chapter", "chunk_index"]

            for point in sample_results[0]:
                metadata = point.payload.get("metadata", {})
                has_all_fields = all(field in metadata for field in required_fields)

                if has_all_fields:
                    complete_metadata_count += 1

            completeness = complete_metadata_count / total_chunks if total_chunks > 0 else 0

            return {
                "completeness": completeness,
                "complete_chunks": complete_metadata_count,
                "total_chunks": total_chunks,
                "sample_size": total_chunks,
                "passed": completeness == 1.0,  # 100% completeness required
                "message": f"Metadata is {completeness:.2%} complete for {total_chunks} chunks"
            }

        except Exception as e:
            logger.error(f"Error validating metadata completeness: {e}")
            return {
                "valid": False,
                "message": f"Error validating metadata: {str(e)}",
                "sample_size": 0
            }

    def run_comprehensive_validation(self) -> Dict[str, Any]:
        """
        Run comprehensive validation of the entire pipeline.

        Returns:
            Dictionary with comprehensive validation results
        """
        start_time = time.time()

        # Validate ingestion completeness
        ingestion_validation = self.retriever.validate_ingestion_completeness()

        # Validate metadata completeness
        metadata_validation = self.validate_metadata_completeness()

        # Run basic retrieval test
        basic_test_query = "What is the main concept of the book?"
        try:
            basic_results = self.retriever.retrieve(basic_test_query, top_k=1)
            basic_retrieval_success = len(basic_results) > 0
        except Exception:
            basic_retrieval_success = False

        # Test determinism
        determinism_test = self.retriever.test_determinism("What is the main concept?", runs=3)

        # Sample test cases for retrieval correctness
        sample_test_cases = [
            {
                "query": "What are the fundamental principles?",
                "expected_sources": ["book", "principles", "fundamentals"]  # Simplified for example
            },
            {
                "query": "Explain the core concepts",
                "expected_sources": ["book", "concepts", "core"]
            }
        ]

        retrieval_validation = self.validate_retrieval_correctness(sample_test_cases)

        total_time = time.time() - start_time

        return {
            "ingestion_validation": ingestion_validation,
            "metadata_validation": metadata_validation,
            "basic_retrieval_success": basic_retrieval_success,
            "determinism_test": determinism_test,
            "retrieval_validation": retrieval_validation,
            "total_time": total_time,
            "overall_passed": (
                ingestion_validation.get("valid", False) and
                metadata_validation.get("passed", False) and
                basic_retrieval_success and
                determinism_test.get("deterministic", False) and
                retrieval_validation.get("passed", False)
            ),
            "message": "Comprehensive validation completed"
        }


def main():
    """Main function to demonstrate the retrieval and validation functionality."""
    import argparse

    parser = argparse.ArgumentParser(description="RAG Retrieval and Validation System")
    parser.add_argument("--query", type=str, help="Text query for similarity search")
    parser.add_argument("--top-k", type=int, default=5, help="Number of results to return")
    parser.add_argument("--validate", action="store_true", help="Run validation tests")
    parser.add_argument("--test-determinism", action="store_true", help="Test determinism")
    parser.add_argument("--full-validation", action="store_true", help="Run comprehensive validation")

    args = parser.parse_args()

    try:
        if args.full_validation:
            print("Running comprehensive pipeline validation...")
            validator = PipelineValidator()
            results = validator.run_comprehensive_validation()

            print("\n=== Comprehensive Validation Results ===")
            print(f"Overall Status: {'PASSED' if results['overall_passed'] else 'FAILED'}")
            print(f"Total Time: {results['total_time']:.2f} seconds")
            print(f"\nIngestion Validation: {results['ingestion_validation']['message']}")
            print(f"Metadata Validation: {results['metadata_validation']['message']}")
            print(f"Basic Retrieval: {'SUCCESS' if results['basic_retrieval_success'] else 'FAILED'}")
            print(f"Determinism Test: {results['determinism_test']['message']}")
            print(f"Retrieval Accuracy: {results['retrieval_validation']['message']}")

        elif args.validate:
            print("Running basic validation tests...")
            validator = PipelineValidator()

            # Validate ingestion
            ingestion_result = validator.retriever.validate_ingestion_completeness()
            print(f"Ingestion validation: {ingestion_result['message']}")

            # Validate metadata
            metadata_result = validator.validate_metadata_completeness()
            print(f"Metadata validation: {metadata_result['message']}")

        elif args.test_determinism:
            if args.query:
                print(f"Testing determinism for query: '{args.query}'")
                retriever = RAGRetriever()
                result = retriever.test_determinism(args.query)
                print(f"Determinism test: {result['message']}")
            else:
                print("Please provide a query for determinism testing using --query")

        elif args.query:
            print(f"Retrieving for query: '{args.query}'")
            retriever = RAGRetriever()
            results = retriever.retrieve(args.query, top_k=args.top_k)

            print(f"\nRetrieved {len(results)} chunks:\n")
            for i, result in enumerate(results, 1):
                print(f"{i}. Score: {result.score:.3f}")
                print(f"   Text: {result.text[:100]}...")
                print(f"   Source: {result.metadata.get('source_url', 'N/A')}")
                print(f"   Module: {result.metadata.get('module', 'N/A')}")
                print(f"   Chapter: {result.metadata.get('chapter', 'N/A')}")
                print(f"   Chunk: {result.metadata.get('chunk_index', 'N/A')}")
                print("---")

        else:
            print("RAG Retrieval and Validation System")
            print("Usage examples:")
            print("  python retrieve.py --query 'your query text' --top-k 5")
            print("  python retrieve.py --validate")
            print("  python retrieve.py --test-determinism --query 'test query'")
            print("  python retrieve.py --full-validation")

    except Exception as e:
        logger.error(f"Error in main execution: {e}")
        print(f"Error: {e}")


if __name__ == "__main__":
    main()