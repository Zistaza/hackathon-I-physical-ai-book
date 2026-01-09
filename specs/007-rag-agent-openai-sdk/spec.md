# Feature Specification: RAG Agent with OpenAI Agents SDK

**Feature Branch**: `007-rag-agent-openai-sdk`
**Created**: 2026-01-08
**Status**: Draft
**Input**: User description: "RAG Agent – OpenAI Agents SDK with Retrieval Integration

Project context:
This spec defines ONLY the agent layer of the RAG system. Ingestion and retrieval validation pipelines are complete and verified.

This spec introduces an AI agent built using the OpenAI Agents SDK that:
- Accepts user questions
- Performs retrieval over the existing Qdrant + Cohere vector store
- Generates grounded, source-cited answers strictly from retrieved book content

Target audience:
- AI / Robotics engineers
- Advanced CS students
- Contributors extending the RAG agent layer

Primary objective:
Design and implement a deterministic, source-grounded RAG agent that:
1. Uses the OpenAI Agents SDK as the sole orchestration layer (ChatKit explicitly out of scope)
2. Integrates the existing Spec-2 retrieval pipeline as an agent tool
3. Answers questions ONLY using retrieved book content
4. Supports:
   - full-book questions
   - user-selected-text-only questions (exclusive context)
5. Safely refuses when answers are not found in retrieved content

Success criteria:
- Answers are grounded exclusively in retrieved chunks
- Every answer cites module/chapter/URL metadata
- No hallucinations or external knowledge injection
- Selected text, when provided, is the only retrieval context
- Identical queries + retrieval results produce deterministic outputs
- Agent is fully testable headlessly (no UI)

Technical constraints:
- Agent framework: OpenAI Agents SDK
- ChatCompletion or single-prompt patterns are forbidden
- All LLM interaction must occur via agent runtime + tool calls
- Retrieval backend: existing Spec-2 logic (Qdrant + Cohere)
- Language: Python
- Configuration via environment variables
- No frontend UI, FastAPI server, ingestion, crawling, or embedding logic

Quality & governance:
- Must comply with constitution.md:
  - Accuracy First
  - Trustworthy, source-grounded AI assistance
  - Spec-driven, deterministic generation
  - Zero plagiarism tolerance
- Agent MUST:
  - Answer only from retrieved content
  - Cite sources or explicitly say "answer not found in book"
  - Prefer correctness over verbosity
- Clear separation between:
  - agent orchestration
  - retrieval tooling
  - response synthesis
  - citation handling

Deliverables:
- spec.md covering:
  - agent architecture
  - retrieval tool interfaces
  - system/prompt strategy
  - citation and refusal logic
- plan.md with implementation phases
- tasks.md with granular, testable tasks
- Minimal, correct, executable Python code for:
  - agent initialization
  - retrieval tool integration
  - grounded answer generation
  - deterministic output handling
  - agent-level validation tests

Explicit non-goals:
- Frontend or chat UI
- FastAPI / HTTP endpoints
- Streaming responses
- Conversation memory beyond one turn
- Personalization or user profiles
- External knowledge access
- LLM-based re-ranking beyond retrieved chunks

Assumptions:
- Spec-1 ingestion is complete and correct
- Spec-2 retrieval is validated and deterministic
- Qdrant contains book embeddings with full metadata
- Cohere and OpenAI API keys are available
- Content language is English

Agent execution model:
- Single-turn execution
- Zero or more tool calls
- Exactly one final answer or refusal
- No conversational memory across runs

Determinism strategy:
- Fixed system instructions
- Fixed tool schemas
- No temperature-based variability
- No hidden state
- Retrieval results passed verbatim to the agent

User Scenarios & Testing (mandatory):

User Story 1 – Grounded QA (P1):
- Correct answers for book-derived questions
- Answers cite module/chapter/URL
- No external facts introduced

User Story 2 – Selected Text Only (P2):
- Retrieval constrained strictly to selected text
- All other book content ignored
- Explicit insufficiency response when needed

User Story 3 – Safe Refusal & Determinism (P3):
- Clear refusal for unrelated queries
- Identical inputs produce identical outputs
- Conflicting chunks are reported and both cited
- Fully testable without UI

Edge cases:
- Empty retrieval results
- Conflicting chunks
- Short, ambiguous, or overly broad queries
- Missing metadata fields

Functional requirements:
- Retrieval results are the only knowledge source
- Every answer is cited or refused
- Selected-text-only mode is enforced
- Deterministic outputs for identical inputs
- Automated testability
- Zero hallucination or speculation

Measurable success criteria:
- 100% answer traceability to retrieved chunks
- 0% external knowledge leakage
- 0% uncited answers
- Fully deterministic behavior
- All refusal cases handled explicitly

Not building:
- UI or frontend widgets
- Backend API server
- Session memory
- Prompt experimentation tools
- Advanced ranking or summarization pipelines"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Grounded QA (Priority: P1)

As an AI/Robotics engineer or Advanced CS student, I want to ask questions about the book content so that I can get accurate, source-cited answers that are grounded exclusively in the retrieved book content.

**Why this priority**: This is the core functionality of the RAG agent - enabling users to ask questions and receive accurate, source-cited answers from the book content. Without this, the agent has no value.

**Independent Test**: The agent can be tested by providing user questions and verifying that the responses are grounded in retrieved content with proper citations to module/chapter/URL, and that no external knowledge is injected.

**Acceptance Scenarios**:

1. **Given** a user question about book content, **When** the RAG agent processes the question, **Then** the agent retrieves relevant book content and generates an answer citing the specific module/chapter/URL
2. **Given** a user question that cannot be answered from the book content, **When** the RAG agent processes the question, **Then** the agent responds with "answer not found in book" without introducing external knowledge

---

### User Story 2 - Selected Text Only Mode (Priority: P2)

As an AI/Robotics engineer or Advanced CS student, I want to constrain the agent's retrieval to only user-selected text so that answers are generated exclusively from the specified content without considering other book content.

**Why this priority**: This provides a specialized mode that allows users to get answers based on a specific subset of content, which is important for focused analysis or when working with particular sections of the book.

**Independent Test**: The agent can be tested by providing selected text along with a question, and verifying that retrieval is constrained strictly to the selected text while ignoring all other book content.

**Acceptance Scenarios**:

1. **Given** user-selected text and a related question, **When** the RAG agent processes the question in selected text mode, **Then** the agent retrieves only from the selected text and ignores all other book content
2. **Given** user-selected text that is insufficient to answer a question, **When** the RAG agent processes the question, **Then** the agent explicitly responds that the answer cannot be found in the selected text

---

### User Story 3 - Safe Refusal and Determinism (Priority: P3)

As an AI/Robotics engineer or Advanced CS student, I want the agent to safely refuse questions it cannot answer and produce deterministic outputs so that I can trust the agent's responses and reproduce identical results.

**Why this priority**: This ensures reliability and trustworthiness of the agent, which is essential for professional use. Determinism is crucial for testing and debugging.

**Independent Test**: The agent can be tested by providing identical inputs multiple times and verifying that identical outputs are produced, and by providing unrelated queries to verify safe refusal.

**Acceptance Scenarios**:

1. **Given** identical queries and retrieval results, **When** the RAG agent processes them, **Then** the agent produces identical outputs demonstrating determinism
2. **Given** an unrelated query that cannot be answered from book content, **When** the RAG agent processes the query, **Then** the agent safely refuses with a clear message without hallucinating

---

### Edge Cases

- What happens when retrieval results are empty?
- How does the system handle conflicting chunks that provide contradictory information?
- What occurs when queries are short, ambiguous, or overly broad?
- How does the system behave when metadata fields are missing from retrieved content?

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST accept user questions as input to the RAG agent
- **FR-002**: System MUST integrate with the existing Spec-2 retrieval pipeline as an agent tool
- **FR-003**: System MUST perform retrieval over the existing Qdrant + Cohere vector store when processing user questions
- **FR-004**: System MUST generate answers that are grounded exclusively in retrieved chunks from book content
- **FR-005**: System MUST cite module/chapter/URL metadata for every answer provided
- **FR-006**: System MUST refuse to answer questions when the answer is not found in retrieved content
- **FR-007**: System MUST support full-book question mode where retrieval occurs across all book content
- **FR-008**: System MUST support user-selected-text-only mode where retrieval is constrained to specified content
- **FR-009**: System MUST ensure identical queries with identical retrieval results produce identical outputs
- **FR-010**: System MUST ensure no hallucinations or external knowledge injection occurs in responses
- **FR-011**: System MUST handle empty retrieval results by providing appropriate responses
- **FR-012**: System MUST handle conflicting chunks by reporting and citing both when necessary
- **FR-013**: System MUST be fully testable headlessly without requiring a UI

### Key Entities

- **RAG Agent**: The AI agent built using the OpenAI Agents SDK that orchestrates question answering
- **Retrieval Tool**: The agent tool that interfaces with the existing Spec-2 retrieval pipeline
- **User Query**: The input question from the user that requires an answer based on book content
- **Retrieved Chunks**: The book content pieces retrieved from Qdrant + Cohere vector store that inform the answer
- **Citation Metadata**: The module/chapter/URL information that must be included in all answers

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: 100% of answers generated by the RAG agent are traceable to retrieved chunks from book content
- **SC-002**: 0% external knowledge leakage occurs in agent responses
- **SC-003**: 0% of answers are provided without proper citations to module/chapter/URL
- **SC-004**: 100% deterministic behavior where identical inputs produce identical outputs
- **SC-005**: All refusal cases are handled explicitly with clear "answer not found in book" messages
- **SC-006**: Selected-text-only mode successfully constrains retrieval to only specified content 100% of the time

## Assumptions

- Spec-1 ingestion is complete and correct
- Spec-2 retrieval is validated and deterministic
- Qdrant contains book embeddings with full metadata
- Cohere and OpenAI API keys are available
- Content language is English
- Existing retrieval pipeline provides reliable and consistent results

## Technical Constraints

- Agent framework: OpenAI Agents SDK
- ChatCompletion or single-prompt patterns are forbidden
- All LLM interaction must occur via agent runtime + tool calls
- Retrieval backend: existing Spec-2 logic (Qdrant + Cohere)
- Language: Python
- Configuration via environment variables
- No frontend UI, FastAPI server, ingestion, crawling, or embedding logic

## Quality & Governance Requirements

- Answers must be grounded exclusively in retrieved chunks
- Every answer must cite module/chapter/URL metadata
- No hallucinations or external knowledge injection
- Selected text, when provided, is the only retrieval context
- Identical queries + retrieval results produce deterministic outputs
- Agent is fully testable headlessly (no UI)
- Must comply with constitution.md: Accuracy First, Trustworthy source-grounded AI assistance, Spec-driven deterministic generation, Zero plagiarism tolerance
