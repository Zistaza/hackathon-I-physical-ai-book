# Data Model: RAG Pipeline – Retrieval Validation & End-to-End Pipeline Testing

## Date: 2026-01-06
## Feature: 006-rag-retrieval-validation

## 1. Core Entities

### 1.1 Query
**Description**: A text input that requires similarity-based retrieval from the vector database

**Fields**:
- `id`: Unique identifier for the query (string)
- `text`: The actual query text (string, required)
- `embedding`: Vector representation of the query text (list[float])
- `timestamp`: When the query was created/processed (datetime)
- `metadata`: Additional query metadata (dict, optional)

**Validation Rules**:
- `text` must not be empty or only whitespace
- `embedding` must have the correct dimensionality for the Cohere model
- `timestamp` is automatically generated when query is processed

### 1.2 Embedding
**Description**: Vector representation of text content used for similarity matching

**Fields**:
- `vector`: The actual embedding vector (list[float], required)
- `dimension`: Number of dimensions in the vector (int, required)
- `model`: Name/version of the embedding model used (string, required)
- `text_hash`: Hash of the original text for verification (string, required)

**Validation Rules**:
- `vector` must have consistent dimensionality across all embeddings
- `model` must match the Cohere model used in Spec-1
- `text_hash` must correspond to the original text content

### 1.3 Retrieved Chunk
**Description**: A segment of book content returned as a result of similarity search

**Fields**:
- `id`: Unique identifier for the chunk (string, required)
- `text`: The actual content text (string, required)
- `embedding`: Vector representation of the content (list[float], required)
- `score`: Similarity score relative to the query (float, required)
- `metadata`: Associated metadata including source information (dict, required)

**Validation Rules**:
- `score` must be between -1 and 1 (for cosine similarity)
- `metadata` must contain all required fields (source URL, module, chapter, chunk index)
- `text` must not be empty

### 1.4 Metadata
**Description**: Information associated with each chunk including source URL, module, chapter, and chunk index

**Fields**:
- `source_url`: URL of the original content source (string, required)
- `module`: Module identifier in the book structure (string, required)
- `chapter`: Chapter identifier within the module (string, required)
- `chunk_index`: Sequential index of the chunk within the chapter (int, required)
- `original_length`: Length of the original text before chunking (int, optional)
- `processed_at`: Timestamp when the chunk was processed (datetime, optional)

**Validation Rules**:
- All required fields must be present and non-empty
- `chunk_index` must be a non-negative integer
- `original_length` must be a positive integer if present
- `source_url` must be a valid URL format

### 1.5 Validation Test
**Description**: A test case that verifies the correctness and completeness of the retrieval pipeline

**Fields**:
- `id`: Unique identifier for the test case (string, required)
- `name`: Descriptive name for the test (string, required)
- `description`: Detailed description of what the test validates (string, required)
- `query_text`: The input query text (string, required)
- `expected_results`: Expected chunks and metadata (list[dict], required)
- `actual_results`: Actual results from the test run (list[dict], optional)
- `status`: Pass/fail status of the test (enum: PASS, FAIL, ERROR, PENDING)
- `timestamp`: When the test was last executed (datetime, optional)
- `execution_time_ms`: Time taken to execute the test (float, optional)

**Validation Rules**:
- `expected_results` must contain at least one expected chunk
- `status` must be one of the defined enum values
- `execution_time_ms` must be non-negative if present

## 2. Relationships

### 2.1 Query → Retrieved Chunk (One-to-Many)
- A single query can retrieve multiple chunks
- Each retrieved chunk is associated with exactly one query
- Relationship established during retrieval process

### 2.2 Retrieved Chunk → Metadata (One-to-One)
- Each retrieved chunk has exactly one metadata object
- Metadata is embedded within the retrieved chunk
- Critical for source attribution and traceability

### 2.3 Validation Test → Query (One-to-One)
- Each validation test is based on a specific query
- The query defines the input for the validation test
- Test results are associated with the specific query used

### 2.4 Validation Test → Retrieved Chunk (One-to-Many)
- A validation test can expect multiple retrieved chunks
- Actual results from the test include multiple retrieved chunks
- Used for comparison between expected and actual results

## 3. Data Flow

### 3.1 Query Processing Flow
1. User provides `Query.text`
2. System generates `Query.embedding` using Cohere model
3. System performs similarity search against stored `Embedding` vectors
4. System retrieves multiple `Retrieved Chunk` objects
5. Each chunk includes associated `Metadata`
6. Results are returned with similarity scores

### 3.2 Validation Flow
1. `Validation Test` defines expected results for a specific query
2. System executes the query and retrieves actual results
3. System compares `actual_results` with `expected_results`
4. System calculates `status` based on comparison
5. System records `execution_time_ms` and `timestamp`

## 4. Validation Requirements

### 4.1 Metadata Completeness
- Every `Retrieved Chunk` must have complete `Metadata`
- Required fields: `source_url`, `module`, `chapter`, `chunk_index`
- Missing metadata fields should result in validation failure

### 4.2 Retrieval Accuracy
- `Retrieved Chunk.text` should match expected content for known queries
- Similarity `score` should meet minimum threshold (configurable)
- Results should be ranked by similarity score (highest first)

### 4.3 Determinism
- Identical queries should produce identical results across multiple runs
- `Retrieved Chunk` ordering should be consistent
- Similarity `score` values should be identical for identical inputs

### 4.4 Error Handling
- Invalid queries should result in appropriate error handling
- Missing or corrupted embeddings should be detected and reported
- Qdrant connectivity issues should be handled gracefully

## 5. Indexing Strategy

### 5.1 Qdrant Collection Structure
- Collection name: `book_embeddings` (or configurable)
- Vector size: Match Cohere embedding dimension (typically 1024)
- Payload schema: Include all `Metadata` fields for filtering
- Index type: HNSW for efficient similarity search

### 5.2 Metadata Indexing
- Index `module` field for fast filtering by book module
- Index `chapter` field for fast filtering by chapter
- Index `source_url` for source-based queries
- Consider composite indexes for multi-field queries