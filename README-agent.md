# RAG Agent with OpenAI Agents SDK

## Overview

This RAG (Retrieval-Augmented Generation) agent uses the OpenAI Agents SDK to answer questions based on book content retrieved from a vector store. The agent ensures all responses are grounded in retrieved content, includes proper citations, and safely refuses to answer when content is not found.

## Features

- **Grounded Responses**: Answers only from retrieved book content, no hallucinations
- **Citation Support**: Includes source citations (module, chapter, URL) for all answers
- **Safe Refusal**: Properly refuses when requested information is not in the book
- **Selected Text Mode**: Can restrict answers to only user-provided selected text
- **Deterministic Behavior**: Identical inputs produce identical outputs
- **Error Handling**: Comprehensive error handling and logging

## Architecture

The agent consists of:
- `RAGAgent`: Main agent class that orchestrates the interaction
- `RAGRetriever`: Interface to the existing retrieval system
- `UserQuery`: Data class for user queries
- `AgentResponse`: Data class for agent responses
- `RetrievedChunk`: Data class for retrieved content chunks

## Usage

### Basic Usage

```python
from backend.agent import RAGAgent, UserQuery

# Initialize the agent
agent = RAGAgent()

# Create a query
query = UserQuery(
    id="example_query_1",
    text="What are the fundamental principles discussed in the book?",
    selected_text_only=False
)

# Process the query
response = agent.query(query)

print(f"Response: {response.content}")
print(f"Citations: {response.citations}")
print(f"Deterministic hash: {response.deterministic_hash}")

# Clean up
agent.close()
```

### Selected Text Mode

```python
# Create a query with selected text only mode
selected_text = "Artificial intelligence is the simulation of human intelligence processes by computer systems."
query = UserQuery(
    id="selected_text_query_1",
    text="What is artificial intelligence?",
    selected_text_only=True,
    selected_text_content=selected_text
)

response = agent.query(query)
```

## Environment Variables

Create a `.env` file with the following variables:

```env
OPENAI_API_KEY=your_openai_api_key_here
COHERE_API_KEY=your_cohere_api_key_here
QDRANT_URL=your_qdrant_cloud_url_here
QDRANT_API_KEY=your_qdrant_api_key_here
QDRANT_COLLECTION_NAME=book_embeddings
COHERE_MODEL=embed-multilingual-v2.0
RETRIEVAL_THRESHOLD=0.3
```

## Requirements

- Python 3.9+
- OpenAI API key
- Cohere API key
- Qdrant Cloud URL and API key
- Book content already ingested into the vector store

## Testing

Run the comprehensive test suite:

```bash
python3 test_comprehensive.py
```

## Error Handling

The agent includes comprehensive error handling:
- Invalid input validation
- Network error handling
- Retrieval system error handling
- Graceful degradation when content is not found
- Detailed logging for debugging

## Determinism

The agent ensures deterministic behavior by:
- Using consistent parameters for LLM calls
- Calculating deterministic hashes for input/output pairs
- Maintaining consistent processing order

## Functional Requirements Satisfied

- **FR-001**: Query input handling
- **FR-002**: Tool usage for retrieval
- **FR-003**: Integration with existing retrieval system
- **FR-004**: Grounded responses in retrieved content
- **FR-005**: Proper citation inclusion
- **FR-006**: Safe refusal behavior
- **FR-007**: Full-book mode
- **FR-008**: Selected-text-only mode
- **FR-009**: Deterministic outputs
- **FR-010**: No external knowledge injection
- **FR-011**: Empty retrieval handling
- **FR-012**: Conflicting information handling
- **FR-013**: Testable functionality

## User Stories Implemented

1. **User Story 1**: Grounded QA with proper citations
2. **User Story 2**: Selected text only mode
3. **User Story 3**: Safe refusal and determinism