# Quickstart Guide: RAG Agent with OpenAI Agents SDK

## Overview
This guide will help you set up and run the RAG Agent that uses the OpenAI Agents SDK to answer questions based on book content retrieved from a vector store.

## Prerequisites
- Python 3.9+
- OpenAI API key
- Cohere API key
- Qdrant Cloud URL and API key
- Book content already ingested into the vector store (completed via Spec-1 and Spec-5)

## Environment Setup

### 1. Install Dependencies
```bash
pip install openai cohere qdrant-client python-dotenv
```

### 2. Configure Environment Variables
Create a `.env` file with the following variables:

```env
OPENAI_API_KEY=your_openai_api_key_here
COHERE_API_KEY=your_cohere_api_key_here
QDRANT_URL=your_qdrant_cloud_url_here
QDRANT_API_KEY=your_qdrant_api_key_here
QDRANT_COLLECTION_NAME=book_embeddings
COHERE_MODEL=embed-english-v3.0
RETRIEVAL_THRESHOLD=0.3
```

## Core Components

### 1. RAG Agent Initialization
The RAG agent is created with specific system instructions and tools:

```python
from openai import OpenAI
import json

client = OpenAI()

# Define the retrieval tool
retrieval_tool = {
    "type": "function",
    "function": {
        "name": "retrieve_content",
        "description": "Retrieve relevant content from the book based on the user's query",
        "parameters": {
            "type": "object",
            "properties": {
                "query": {
                    "type": "string",
                    "description": "The user's question or query to search for in the book content"
                },
                "top_k": {
                    "type": "integer",
                    "description": "Number of results to return (default: 5)",
                    "minimum": 1,
                    "maximum": 10
                },
                "selected_text_only": {
                    "type": "boolean",
                    "description": "Whether to restrict search to selected text only",
                    "default": false
                },
                "selected_text_content": {
                    "type": "string",
                    "description": "The specific text content to search within when selected_text_only is true"
                }
            },
            "required": ["query"]
        }
    }
}

# Create the assistant with instructions
assistant = client.beta.assistants.create(
    name="Book RAG Assistant",
    instructions="""
    You are a RAG (Retrieval-Augmented Generation) assistant for book content.

    Your primary function is to answer questions based solely on retrieved book content.
    You must:
    1. Only use information from the retrieved content chunks
    2. Cite the source of your information (module, chapter, URL)
    3. If the information is not found in the retrieved content, explicitly state "Answer not found in book"
    4. When in selected-text-only mode, only use the provided selected text for answers

    Do not use any external knowledge or make up information.
    """,
    model="gpt-4-turbo-preview",  # or another suitable model
    tools=[retrieval_tool]
)
```

### 2. Retrieval Tool Implementation
The retrieval tool must connect to the existing retrieval system:

```python
import cohere
from qdrant_client import QdrantClient
import os

class RAGRetriever:
    def __init__(self):
        # Initialize clients using environment variables
        self.cohere_client = cohere.Client(os.getenv("COHERE_API_KEY"))
        self.qdrant_client = QdrantClient(
            url=os.getenv("QDRANT_URL"),
            api_key=os.getenv("QDRANT_API_KEY")
        )
        self.collection_name = os.getenv("QDRANT_COLLECTION_NAME", "book_embeddings")

    def retrieve_content(self, query, top_k=5, selected_text_only=False, selected_text_content=None):
        """
        Retrieve content from the vector store based on the query.
        """
        if selected_text_only and selected_text_content:
            # In selected-text-only mode, search within the provided text
            # This would require a different search strategy
            # Implementation depends on how to handle selected text restriction
            pass
        else:
            # Standard retrieval from the full book content
            # Generate query embedding
            response = self.cohere_client.embed(
                texts=[query],
                model=os.getenv("COHERE_MODEL", "embed-english-v3.0"),
                input_type="search_document"
            )
            query_embedding = response.embeddings[0]

            # Perform similarity search
            search_results = self.qdrant_client.search(
                collection_name=self.collection_name,
                query_vector=query_embedding,
                limit=top_k,
                with_payload=True
            )

            # Format results
            chunks = []
            for result in search_results:
                chunks.append({
                    "text": result.payload.get("content", ""),
                    "score": result.score,
                    "metadata": {
                        "source_url": result.payload.get("metadata", {}).get("source_url", ""),
                        "module": result.payload.get("metadata", {}).get("module", ""),
                        "chapter": result.payload.get("metadata", {}).get("chapter", ""),
                        "chunk_index": result.payload.get("metadata", {}).get("chunk_index", "")
                    }
                })

            return {"chunks": chunks}
```

### 3. Running a Query
Here's how to run a query with the RAG agent:

```python
# Create a thread for the conversation
thread = client.beta.threads.create()

# Add a user message to the thread
message = client.beta.threads.messages.create(
    thread_id=thread.id,
    role="user",
    content="What are the fundamental principles of the book?"
)

# Run the assistant
run = client.beta.threads.runs.create(
    thread_id=thread.id,
    assistant_id=assistant.id
)

# Poll for completion
import time
while run.status in ["queued", "in_progress"]:
    time.sleep(1)
    run = client.beta.threads.runs.retrieve(thread_id=thread.id, run_id=run.id)

# Get the messages
messages = client.beta.threads.messages.list(thread_id=thread.id)
for msg in messages.data:
    print(f"{msg.role}: {msg.content[0].text.value}")
```

## Testing the Agent

### 1. Basic Query Test
```python
def test_basic_query():
    """Test the agent with a basic query."""
    # Implementation similar to the example above
    pass

def test_grounding_validation():
    """Validate that responses are grounded in retrieved content."""
    # Compare agent responses to retrieved chunks to ensure grounding
    pass

def test_citation_validation():
    """Validate that responses include proper citations."""
    # Check that responses contain module/chapter/URL citations
    pass

def test_refusal_behavior():
    """Test that the agent refuses when content is not found."""
    # Use queries that should result in "Answer not found in book"
    pass

def test_determinism():
    """Test that identical inputs produce identical outputs."""
    # Run the same query multiple times and compare results
    pass
```

## Configuration Options

### System Instructions
Customize the assistant's behavior by modifying the instructions:

```python
instructions = """
You are a RAG (Retrieval-Augmented Generation) assistant for book content.
[Customize behavior here]
"""
```

### Retrieval Parameters
Adjust retrieval behavior through the tool parameters:

```python
retrieval_tool = {
    # ... existing configuration ...
    "function": {
        "name": "retrieve_content",
        "parameters": {
            # Adjust these parameters as needed
            "top_k": 5,  # Number of results to retrieve
            "threshold": 0.3  # Minimum similarity score
        }
    }
}
```

## Troubleshooting

### Common Issues
1. **No API Keys**: Ensure all required environment variables are set
2. **Empty Results**: Verify that the book content has been properly ingested
3. **Connection Errors**: Check network connectivity to Qdrant and Cohere
4. **Rate Limits**: Implement retry logic if hitting API rate limits

### Validation Steps
1. Test retrieval system independently before integrating with the agent
2. Verify that the vector store contains the expected content
3. Check that all required metadata fields are present in retrieved chunks
4. Validate that the agent follows grounding and citation requirements