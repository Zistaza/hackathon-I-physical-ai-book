# Quickstart Guide: RAG Pipeline – Retrieval Validation & End-to-End Pipeline Testing

## Overview

This guide provides instructions for setting up and running the RAG retrieval validation system. The system connects to Qdrant to retrieve stored embeddings, executes similarity-based retrieval for given text queries, and validates the correctness, completeness, and determinism of the ingestion pipeline.

## Prerequisites

- Python 3.11 or higher
- pip package manager
- Access to Cohere API (API key)
- Access to Qdrant Cloud (URL and API key)
- Git for version control

## Environment Setup

### 1. Clone the Repository
```bash
git clone <repository-url>
cd hackathon-I-physical-ai-book
git checkout 006-rag-retrieval-validation
```

### 2. Create Virtual Environment
```bash
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

### 3. Install Dependencies
```bash
pip install -r requirements.txt
# Or install required packages directly:
pip install cohere qdrant-client python-dotenv pytest
```

### 4. Configure Environment Variables
Create a `.env` file in the project root with the following variables:

```env
COHERE_API_KEY=your_cohere_api_key_here
QDRANT_URL=your_qdrant_cloud_url_here
QDRANT_API_KEY=your_qdrant_api_key_here
QDRANT_COLLECTION_NAME=book_embeddings
COHERE_MODEL=embed-english-v3.0
```

## Running the Retrieval System

### 1. Basic Retrieval
To run a basic retrieval test:

```bash
python retrieve.py --query "your query text here" --top-k 5
```

### 2. Validation Testing
To run validation tests:

```bash
python retrieve.py --validate
```

### 3. Determinism Testing
To test for deterministic behavior:

```bash
python retrieve.py --test-determinism
```

### 4. Full Pipeline Validation
To run comprehensive pipeline validation:

```bash
python retrieve.py --full-validation
```

## Using the API Programmatically

### Basic Retrieval Example
```python
from backend.retrieve import RAGRetriever

# Initialize the retriever
retriever = RAGRetriever()

# Perform a retrieval
results = retriever.retrieve("your query text", top_k=5)

# Print results with metadata
for result in results:
    print(f"Score: {result['score']}")
    print(f"Text: {result['text'][:100]}...")
    print(f"Source: {result['metadata']['source_url']}")
    print(f"Module: {result['metadata']['module']}")
    print("---")
```

### Validation Example
```python
from backend.retrieve import PipelineValidator

validator = PipelineValidator()

# Run basic validation
validation_results = validator.validate_pipeline()

# Run specific test cases
test_results = validator.run_test_cases([
    {
        "query": "What is the main concept in Chapter 1?",
        "expected_sources": ["https://book-url/module1/chapter1"]
    }
])

print(f"Validation passed: {validation_results['passed']}")
print(f"Accuracy: {validation_results['accuracy']:.2%}")
```

## Configuration Options

### Command Line Options
- `--query`: Text query for similarity search
- `--top-k`: Number of results to return (default: 5)
- `--validate`: Run validation tests
- `--test-determinism`: Test for deterministic behavior
- `--full-validation`: Run comprehensive validation suite
- `--collection`: Qdrant collection name (default: book_embeddings)
- `--model`: Cohere model to use (default: embed-english-v3.0)

### Environment Variables
- `COHERE_API_KEY`: API key for Cohere services
- `QDRANT_URL`: URL for Qdrant Cloud instance
- `QDRANT_API_KEY`: API key for Qdrant Cloud
- `QDRANT_COLLECTION_NAME`: Name of the collection to query (default: book_embeddings)
- `COHERE_MODEL`: Embedding model to use (default: embed-english-v3.0)
- `RETRIEVAL_THRESHOLD`: Minimum similarity score threshold (default: 0.3)

## Running Tests

### Unit Tests
```bash
pytest tests/test_retrieval.py
```

### Validation Tests
```bash
pytest tests/test_validation.py
```

### Determinism Tests
```bash
pytest tests/test_determinism.py
```

### All Tests
```bash
pytest tests/
```

## Expected Output

When running retrieval, you should see output similar to:

```
Query: "What is the main concept in Chapter 1?"
Retrieved 5 chunks:

1. Score: 0.847
   Text: "The main concept in Chapter 1 is the fundamental principle of..."
   Source: https://book-url/module1/chapter1
   Module: Module 1
   Chapter: Chapter 1
   Chunk: 0

2. Score: 0.792
   Text: "This chapter introduces the core concepts that form the foundation..."
   Source: https://book-url/module1/chapter1
   Module: Module 1
   Chapter: Chapter 1
   Chunk: 1
...
```

## Troubleshooting

### Common Issues

1. **Connection to Qdrant fails**:
   - Verify `QDRANT_URL` and `QDRANT_API_KEY` in your `.env` file
   - Check that your Qdrant Cloud instance is accessible

2. **Cohere API error**:
   - Verify `COHERE_API_KEY` in your `.env` file
   - Check that your API key has the necessary permissions

3. **No results returned**:
   - Verify that the collection contains embeddings
   - Check that your query is related to the book content
   - Adjust the `RETRIEVAL_THRESHOLD` if needed

4. **Determinism test fails**:
   - This may indicate an issue with the retrieval pipeline
   - Check for any random factors in the ranking algorithm

### Getting Help
- Check the `specs/006-rag-retrieval-validation/` directory for detailed specifications
- Review the implementation in `backend/rag_pipeline/` modules
- Look at the test files in `tests/` for usage examples