# Research Findings: RAG Agent with OpenAI Agents SDK

## OpenAI Agents SDK Overview

The OpenAI Agents SDK provides a way to create and manage AI assistants that can use tools and engage in conversations. Key features relevant to our RAG agent:

- **Assistant**: Represents an AI agent with instructions and tools
- **Thread**: Represents a conversation thread with messages
- **Message**: Input/output messages in a thread
- **Run**: Represents the execution of an assistant on a thread
- **Tool**: Functions that the assistant can call to perform actions

## Existing Retrieval Pipeline (Spec-2/Spec-6)

Based on the `retrieve.py` file, the existing retrieval pipeline provides:

### Core Components:
- `RAGRetriever`: Main class for retrieval functionality
- `RetrievedChunk`: Data class representing retrieved content with text, score, and metadata
- `PipelineValidator`: Class for validation tests

### Key Methods:
- `generate_query_embedding(query: str)`: Creates embeddings using Cohere
- `retrieve(query: str, top_k: int = 5)`: Performs similarity search in Qdrant
- `validate_ingestion_completeness()`: Checks if data is properly ingested
- `test_determinism(query: str, runs: int = 3)`: Verifies deterministic results

### Metadata Structure:
- `source_url`: URL of the original document
- `module`: Module identifier
- `chapter`: Chapter identifier
- `chunk_index`: Index of the chunk
- `content`: The actual text content

## Deterministic Behavior Requirements

For the RAG agent to meet requirements:
- Identical queries with identical retrieval results must produce identical outputs
- Sorting of retrieved chunks must be deterministic (by score and ID)
- No temperature-based variability in responses
- Tool calls must return consistent results

## Integration Strategy

The RAG agent will:
1. Accept user queries via the OpenAI Agents SDK
2. Use a custom tool to interface with the existing `RAGRetriever`
3. Generate grounded responses based on retrieved content
4. Include proper citations in responses
5. Handle cases where no relevant content is found

## Key Decisions

### 1. Agent Instruction Strategy
- Use clear system instructions that mandate grounding in retrieved content
- Specify citation requirements explicitly
- Define refusal behavior for out-of-scope queries

### 2. Tool Schema Design
- Create a retrieval tool that accepts query and top_k parameters
- Return structured data with content, scores, and metadata
- Include error handling for retrieval failures

### 3. Response Synthesis
- Ensure responses only use information from retrieved chunks
- Format citations properly with source URLs and metadata
- Handle conflicting information gracefully

### 4. Determinism Enforcement
- Use deterministic sorting of retrieved results
- Avoid any randomness in response generation
- Cache tool responses for identical inputs during a single run