# Data Model: RAG Agent with OpenAI Agents SDK

## Core Entities

### 1. RAGAgent
**Description**: The AI agent built using the OpenAI Agents SDK that orchestrates question answering
- **Attributes**:
  - `id`: Unique identifier for the agent
  - `instructions`: System instructions defining agent behavior
  - `tools`: List of available tools (including retrieval tool)
  - `model`: The underlying LLM model used by the agent
  - `created_at`: Timestamp of agent creation

### 2. UserQuery
**Description**: The input question from the user that requires an answer based on book content
- **Attributes**:
  - `id`: Unique identifier for the query
  - `text`: The actual question text from the user
  - `selected_text_only`: Boolean flag indicating if retrieval should be constrained to selected text only
  - `selected_text_content`: Optional text content that should be the exclusive retrieval context
  - `timestamp`: When the query was submitted

### 3. RetrievedChunk
**Description**: The book content pieces retrieved from Qdrant + Cohere vector store that inform the answer
- **Attributes**:
  - `text`: The actual content text retrieved
  - `score`: Similarity score from the vector search
  - `source_url`: URL of the original document
  - `module`: Module identifier from metadata
  - `chapter`: Chapter identifier from metadata
  - `chunk_index`: Index of the chunk within the document
  - `relevance_score`: Additional relevance scoring if needed

### 4. AgentResponse
**Description**: The output from the RAG agent containing the answer and citations
- **Attributes**:
  - `id`: Unique identifier for the response
  - `content`: The actual response text
  - `citations`: List of source citations used in the response
  - `retrieved_chunks_used`: List of retrieved chunks that informed the response
  - `refusal_reason`: Optional reason if the agent refused to answer
  - `timestamp`: When the response was generated
  - `deterministic_hash`: Hash to verify determinism for identical inputs

### 5. RetrievalTool
**Description**: The agent tool that interfaces with the existing Spec-2 retrieval pipeline
- **Attributes**:
  - `id`: Unique identifier for the tool
  - `name`: Name of the tool ("retrieve_content")
  - `description`: Purpose of the tool
  - `parameters`: Schema defining input parameters
  - `function`: Reference to the retrieval function

### 6. ThreadContext
**Description**: Represents a single interaction context for the agent
- **Attributes**:
  - `id`: Unique identifier for the thread
  - `messages`: List of messages in the conversation
  - `created_at`: Timestamp of thread creation
  - `last_accessed`: Last time the thread was accessed

## Relationships

### RAGAgent → RetrievalTool
- One-to-many relationship
- A RAG agent can have multiple tools, including the retrieval tool

### UserQuery → ThreadContext
- Many-to-one relationship
- Multiple queries can be part of the same thread context

### ThreadContext → AgentResponse
- One-to-many relationship
- A thread can have multiple responses over time

### UserQuery → RetrievedChunk
- Many-to-many relationship (through the retrieval process)
- A query can retrieve multiple chunks, and chunks can be relevant to multiple queries

### RetrievedChunk → AgentResponse
- Many-to-many relationship
- A response can use multiple retrieved chunks, and chunks can inform multiple responses

## State Transitions

### AgentResponse States:
1. **Processing**: Response generation in progress
2. **Completed**: Response generated successfully
3. **Refused**: Agent refused to answer due to insufficient content
4. **Error**: Error occurred during response generation

## Validation Rules

### From Functional Requirements:
- **FR-004**: AgentResponse.content must be grounded exclusively in RetrievedChunk.text
- **FR-005**: AgentResponse.citations must not be empty for valid answers
- **FR-006**: AgentResponse.refusal_reason must be populated when refusing to answer
- **FR-008**: When UserQuery.selected_text_only is true, retrieval must only use UserQuery.selected_text_content
- **FR-010**: AgentResponse must not contain hallucinations or external knowledge
- **FR-012**: When RetrievedChunk contains conflicting information, both must be cited and reported