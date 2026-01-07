# Implementation Plan: RAG Pipeline – Retrieval Validation & End-to-End Pipeline Testing

**Branch**: `006-rag-retrieval-validation` | **Date**: 2026-01-06 | **Spec**: [specs/006-rag-retrieval-validation/spec.md](/specs/006-rag-retrieval-validation/spec.md)
**Input**: Feature specification from `/specs/006-rag-retrieval-validation/spec.md`

## Summary

This plan outlines the implementation of a deterministic retrieval and validation layer for the RAG pipeline. The solution will connect to Qdrant to retrieve stored embeddings, execute similarity-based retrieval for given text queries, and validate the correctness, completeness, and determinism of the ingestion pipeline. The implementation will include automated testing to ensure reproducible, auditable results that prove the RAG foundation is sound.

## Technical Context

**Language/Version**: Python 3.11
**Primary Dependencies**: cohere, qdrant-client, python-dotenv, pytest
**Storage**: Qdrant Cloud (Free Tier) - vector database with stored embeddings and metadata
**Testing**: pytest for automated validation and regression testing
**Target Platform**: Linux server environment
**Project Type**: backend - focused on retrieval and validation logic
**Performance Goals**: Retrieval latency < 500ms for typical queries, ≥95% retrieval accuracy for known-content queries
**Constraints**: Must use same Cohere model as Spec-1, no modification of stored vectors, deterministic results
**Scale/Scope**: Single-file implementation with comprehensive validation coverage

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- **Accuracy First**: Implementation will use only indexed book content for retrieval, with proper source attribution
- **Spec-Driven Generation**: Implementation will strictly follow spec.md requirements for retrieval and validation
- **Clarity for Advanced Learners**: Code will include clear documentation and comments for maintainers
- **Modular Documentation**: Implementation will be self-contained but well-documented
- **Trustworthy, Source-Grounded AI Assistance**: Retrieval will be limited to indexed content with proper metadata
- **Zero Plagiarism Tolerance**: Implementation will be original code with no external knowledge leakage

## Project Structure

### Documentation (this feature)

```text
specs/006-rag-retrieval-validation/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
backend/
├── rag_pipeline/
│   ├── __init__.py
│   ├── config/
│   │   ├── __init__.py
│   │   └── settings.py
│   ├── embedding/
│   │   ├── __init__.py
│   │   └── query_embedding.py
│   ├── storage/
│   │   ├── __init__.py
│   │   └── qdrant_client.py
│   ├── retrieval/
│   │   ├── __init__.py
│   │   └── retriever.py
│   ├── validation/
│   │   ├── __init__.py
│   │   └── pipeline_validator.py
│   └── utils/
│       ├── __init__.py
│       └── helpers.py
├── retrieve.py          # Main entry point for retrieval and validation
└── tests/
    ├── __init__.py
    ├── test_retrieval.py
    ├── test_validation.py
    └── test_determinism.py
```

**Structure Decision**: Backend-focused structure with modular components for retrieval, validation, and testing. The main `retrieve.py` file serves as the entry point for the retrieval and validation functionality.

## Architecture Sketch

```
┌─────────────────┐    ┌──────────────────┐    ┌──────────────────┐
│   User Query    │───▶│ Query Embedding  │───▶│ Qdrant Vector DB │
│                 │    │ (Cohere model)   │    │                  │
└─────────────────┘    └──────────────────┘    └──────────────────┘
                                                    │
                                                    ▼
                                          ┌──────────────────┐
                                          │ Similarity       │
                                          │ Search & Ranking │
                                          └──────────────────┘
                                                    │
                                                    ▼
                                          ┌──────────────────┐
                                          │ Retrieved Chunks │
                                          │ (with metadata)  │
                                          └──────────────────┘
                                                    │
                                                    ▼
                                          ┌──────────────────┐
                                          │ Validation &     │
                                          │ Determinism Test │
                                          └──────────────────┘
                                                    │
                                                    ▼
                                          ┌──────────────────┐
                                          │ Results &        │
                                          │ Test Reports     │
                                          └──────────────────┘
```

## Pipeline Structure

### 1. Ingestion Verification Layer
- Verify Qdrant collection exists and contains expected embeddings
- Validate metadata completeness for each stored chunk
- Check that all expected book content has been ingested

### 2. Retrieval Logic Layer
- Generate query embeddings using the same Cohere model as Spec-1
- Execute top-K similarity search against Qdrant
- Apply deterministic ranking based on similarity scores
- Return chunks with complete metadata (source URL, module, chapter, chunk index)

### 3. Validation & Determinism Layer
- Execute known-content queries and verify correct content is returned
- Test for deterministic behavior by re-running identical queries
- Validate metadata completeness in all retrieved results
- Handle edge cases gracefully (empty queries, unrelated content, etc.)

## Validation Methodology

### 1. Retrieval Correctness
- **Known-content queries**: Use text derived from specific book sections to verify correct content retrieval
- **Accuracy measurement**: Track percentage of correctly retrieved content vs. expected content
- **Source attribution**: Verify metadata correctly links retrieved chunks to original sources

### 2. Metadata Completeness
- **Required fields check**: Ensure each retrieved chunk includes source URL, module, chapter, and chunk index
- **Metadata validation**: Verify all metadata fields are properly formatted and complete
- **Cross-reference verification**: Check that metadata aligns with original content

### 3. Regression Detection
- **Baseline establishment**: Create test cases with expected results from current working state
- **Regression tests**: Run comprehensive test suites to detect unintended changes
- **Automated alerts**: Flag any deviations from expected behavior

## Test Scenario Mapping

### User Story 1 – Similarity Retrieval Validation
- **Test Case 1.1**: Query derived from known book section → verify correct chapter content returned
- **Test Case 1.2**: Retrieve chunks with complete metadata (source URL, module, chapter, chunk index)
- **Test Case 1.3**: Repeat identical queries → verify consistent similarity scores

### User Story 2 – End-to-End Pipeline Verification
- **Test Case 2.1**: Execute comprehensive validation → verify ingested content can be retrieved without loss
- **Test Case 2.2**: Check for missing or orphaned embeddings → ensure complete pipeline integrity
- **Test Case 2.3**: Query unrelated to book content → verify graceful failure handling

### User Story 3 – Determinism & Regression Testing
- **Test Case 3.1**: Execute identical queries at different times → verify identical ranked results
- **Test Case 3.2**: Re-run test cases → verify consistent outputs
- **Test Case 3.3**: Run regression tests after code changes → detect unintended changes

## Key Architectural Decisions

### 1. Embedding Retrieval & Similarity Metrics
- **Decision**: Use cosine similarity for vector comparison
- **Rationale**: Cosine similarity is standard for embedding comparisons and works well with Cohere embeddings
- **Alternatives considered**: Euclidean distance, dot product

### 2. Deterministic Ranking Strategy
- **Decision**: Implement stable sorting with consistent tie-breaking
- **Rationale**: Ensures identical queries produce identical results across runs
- **Alternatives considered**: Random tie-breaking (rejected due to non-determinism)

### 3. Error Handling Approach
- **Decision**: Graceful degradation with meaningful error messages
- **Rationale**: Maintains system stability while providing diagnostic information
- **Alternatives considered**: Hard failures (rejected due to operational concerns)

### 4. Top-K Retrieval Strategy
- **Decision**: Configurable K value with default of 5
- **Rationale**: Balances relevance with performance, allows tuning for different use cases
- **Alternatives considered**: Fixed K value (rejected due to inflexibility)

## Implementation Phases

### Phase 0: Research & Setup
- Research Qdrant client capabilities and best practices
- Verify existing embeddings in Qdrant collection
- Set up environment and dependencies

### Phase 1: Core Retrieval Implementation
- Implement query embedding generation using Cohere
- Create Qdrant client connection and retrieval logic
- Add deterministic ranking functionality

### Phase 2: Validation & Testing
- Develop validation tests for retrieval correctness
- Implement determinism verification
- Create comprehensive test suite

### Phase 3: Edge Case Handling & Optimization
- Handle edge cases (empty queries, malformed input, etc.)
- Optimize performance for latency requirements
- Add error handling and logging

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| Multiple module structure | Maintainability and separation of concerns | Single file would be too complex to maintain |
| External dependencies | Required for vector operations and embeddings | Would need to implement from scratch |
