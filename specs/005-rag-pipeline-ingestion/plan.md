# Implementation Plan: RAG Pipeline – Website Content Ingestion, Embedding Generation, and Vector Storage

**Branch**: `005-rag-pipeline-ingestion` | **Date**: 2026-01-06 | **Spec**: [specs/005-rag-pipeline-ingestion/spec.md](specs/005-rag-pipeline-ingestion/spec.md)
**Input**: Feature specification from `/specs/005-rag-pipeline-ingestion/spec.md`

## Summary

Build a deterministic RAG pipeline that ingests Docusaurus website content, extracts clean text, generates Cohere embeddings, and stores them in Qdrant Cloud with metadata. The pipeline must be idempotent, handle errors gracefully, and maintain clear separation between crawling → parsing → chunking → embedding → storage phases.

## Technical Context

**Language/Version**: Python 3.11
**Primary Dependencies**: requests, beautifulsoup4, cohere, qdrant-client, python-dotenv
**Storage**: Qdrant Cloud (vector database), environment variables for configuration
**Testing**: pytest for unit and integration tests
**Target Platform**: Linux server environment
**Project Type**: Backend pipeline processing
**Performance Goals**: Process up to 1000 content chunks per run without performance degradation, complete typical book chapter (5-10 pages) within 5 minutes
**Constraints**: Must be deterministic and idempotent (no duplicate vectors), text chunking must be reproducible, configuration via environment variables
**Scale/Scope**: Handle up to 1000 content chunks per run, support multiple Docusaurus website URLs

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- **Accuracy First**: All implementation will use official APIs and documented methods for Docusaurus content extraction, Cohere embeddings, and Qdrant storage
- **Spec-Driven, Deterministic Generation**: Pipeline will follow deterministic chunking algorithms and idempotent storage patterns as specified
- **Clarity for Advanced Learners**: Code will include clear documentation and comments appropriate for CS/AI/Robotics practitioners
- **Modular, Extensible Documentation**: Implementation will use modular design with clear separation of concerns
- **Trustworthy, Source-Grounded AI Assistance**: Only content from specified Docusaurus websites will be ingested, no external knowledge sources
- **Zero Plagiarism Tolerance**: Code will be original implementation following the spec requirements

## Project Structure

### Documentation (this feature)
```text
specs/005-rag-pipeline-ingestion/
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
│   │   └── settings.py          # Environment variable configuration
│   ├── ingestion/
│   │   ├── __init__.py
│   │   ├── crawler.py           # URL fetching and navigation
│   │   └── parser.py            # Docusaurus content extraction
│   ├── processing/
│   │   ├── __init__.py
│   │   ├── chunker.py           # Text chunking with reproducible boundaries
│   │   └── cleaner.py           # Text cleaning and normalization
│   ├── embedding/
│   │   ├── __init__.py
│   │   └── generator.py         # Cohere embedding generation
│   ├── storage/
│   │   ├── __init__.py
│   │   └── vector_db.py         # Qdrant Cloud interaction
│   ├── models/
│   │   ├── __init__.py
│   │   └── data.py              # Data models for content chunks and embeddings
│   ├── utils/
│   │   ├── __init__.py
│   │   └── helpers.py           # Utility functions
│   └── main.py                  # Main pipeline execution
├── requirements.txt
├── pyproject.toml               # Project configuration for uv
└── .env.example                 # Example environment variables
```

**Structure Decision**: Backend pipeline structure selected to support the RAG ingestion pipeline with clear separation of concerns between crawling, parsing, chunking, embedding, and storage phases as required by the specification.

## Implementation Phases

### Phase 0: Research and Setup
- Initialize project structure using `uv` in `backend/` folder, create empty `__init__.py` files and `main.py`.
- Research Docusaurus content extraction techniques
- Investigate Cohere embedding models and API usage
- Explore Qdrant Cloud setup and Python client
- Define deterministic chunking algorithms


### Phase 1: Core Components Development
- Implement configuration management with environment variables
- Build web crawler for Docusaurus websites
- Create content parser to extract clean text from Docusaurus pages
- Develop deterministic text chunking algorithm
- Implement Cohere embedding generation
- Create Qdrant Cloud storage interface

### Phase 2: Pipeline Integration and Testing
- Integrate all components into end-to-end pipeline
- Implement idempotency checks to prevent duplicate vectors
- Add comprehensive error handling and logging
- Create unit and integration tests
- Add command-line interface for pipeline execution

## Key Design Decisions

1. **Deterministic Chunking**: Use consistent text boundaries based on semantic breaks (paragraphs, headers) with configurable maximum chunk size to ensure reproducible results across runs.

2. **Idempotency Strategy**: Generate unique identifiers for content chunks based on source URL + content hash to prevent duplicate storage in Qdrant.

3. **Error Resilience**: Implement retry mechanisms for API calls and network requests, with graceful degradation when individual pages fail.

4. **Modular Architecture**: Separate concerns into distinct modules to allow for independent testing and modification of each pipeline stage.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| Multiple Dependencies | Cohere API, Qdrant client, web scraping libraries needed for core functionality | Would limit functionality and prevent meeting spec requirements |
| Multi-module Architecture | Required by spec for clear separation between crawling → parsing → chunking → embedding → storage | Spec explicitly requires clear separation of pipeline phases |