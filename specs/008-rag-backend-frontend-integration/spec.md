# Feature Specification: RAG Backend Frontend Integration

**Feature Branch**: `008-rag-backend-frontend-integration`
**Created**: 2026-01-10
**Status**: Draft
**Input**: User description: "Integrate RAG Backend with Book Frontend using FastAPI

Target audience:
- Advanced CS / AI / Robotics learners using the Physical AI & Humanoid Robotics textbook
- Developers evaluating how RAG systems integrate with documentation platforms

Focus:
- Establish a local and production-ready integration between the existing RAG backend and the Docusaurus-based book frontend
- Expose RAG capabilities through a FastAPI service
- Enable the frontend to query the backend for answers grounded strictly in indexed book content
- Support both:
  1) General book-wide questions
  2) Questions constrained to user-selected text

Success criteria:
- FastAPI backend exposes clear, documented REST endpoints for:
  - Health/status check
  - RAG query with full-book context
  - RAG query constrained to user-selected text
- Frontend can successfully call backend endpoints locally (localhost) and in deployment
- RAG responses are generated exclusively from retrieved Qdrant content via the OpenAI Agent
- Backend returns:
  - Final answer
  - Retrieved source metadata (chapter/section identifiers)
  - Clear "not found in book" responses when applicable
- No direct model calls from the frontend (backend-only responsibility)
- System is deterministic, auditable, and reproducible

Constraints:
- Backend framework: FastAPI only
- Frontend: Existing Docusaurus site (no framework migration)
- Retrieval pipeline: Must reuse backend/rag-pipeline folder and backend/retrieve.py file outputs
- Agent logic: Must reuse backend/agent.py file for OpenAI Agents SDK implementation
- Vector store: Qdrant Cloud (Free Tier)
- Database (if needed): Neon Serverless Postgres (metadata/session tracking only)
- API keys loaded via environment variables (no hardcoding)
- Documentation-first approach: all endpoints and data contracts documented
- Local-first development (WSL-compatible), deployable later without redesign

Quality and governance requirements:
- All behavior must comply with constitution.md
- No external knowledge leakage beyond indexed book content
- Clear separation of concerns:
  - Frontend: UI and API calls only
  - Backend: retrieval, agent orchestration, response grounding
- Minimal, correct, executable code only
- No speculative features or undocumented endpoints

Deliverables:
- FastAPI application structure with clearly defined routers
- Request/response schemas for all endpoints
- Example frontend integration flow (API call sequence)
- Local run instructions (FastAPI + frontend)
- Validation checklist confirming end-to-end RAG flow

Not building:
- UI/UX redesign of the Docusaurus site
- Authentication, user accounts, or billing
- Streaming responses or WebSockets
- Fine-tuning, retraining, or new embedding pipelines
- Production-grade scaling, monitoring, or CI/CD"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Query Book Content with RAG (Priority: P1)

An advanced CS/AI/Robotics learner is reading the Physical AI & Humanoid Robotics textbook online and wants to ask complex questions about the content. The learner selects specific text on the page or asks a general question about the book. The system processes the query through the RAG backend and returns an answer grounded in the indexed book content with source references.

**Why this priority**: This is the core value proposition - enabling learners to get accurate, contextually relevant answers from the textbook content.

**Independent Test**: The system can be fully tested by submitting a question and verifying that the response is based on the book content with proper source attribution, delivering immediate value to the learner.

**Acceptance Scenarios**:

1. **Given** a learner is on the book website, **When** they submit a general question about book content, **Then** they receive an answer grounded in the indexed book content with source references
2. **Given** a learner has selected text on a book page, **When** they submit a question related to the selected text, **Then** they receive an answer constrained to information from that specific text with source references

---

### User Story 2 - Verify Answer Source Credibility (Priority: P2)

A developer evaluating RAG systems wants to understand how answers are generated and verify that responses are truly grounded in the book content. The system provides clear source metadata showing exactly which chapters/sections were used to generate the response.

**Why this priority**: Critical for trust and auditability - users need to verify that answers come from the expected sources.

**Independent Test**: Can be fully tested by submitting queries and verifying that source metadata is provided with each response, enabling verification of answer grounding.

**Acceptance Scenarios**:

1. **Given** a query has been processed, **When** the response is returned, **Then** it includes clear source metadata (chapter/section identifiers) for all information provided

---

### User Story 3 - Handle Cases Where Information is Not Available (Priority: P3)

A learner asks a question that cannot be answered based on the indexed book content. The system recognizes when the requested information is not available in the book and provides a clear response indicating this limitation.

**Why this priority**: Important for user experience - users should know when information is not available rather than receiving hallucinated responses.

**Independent Test**: Can be tested by submitting queries about topics not covered in the book and verifying that appropriate "not found in book" responses are returned.

**Acceptance Scenarios**:

1. **Given** a query that cannot be answered from book content, **When** the query is processed, **Then** the system returns a clear "not found in book" response

---

### Edge Cases

- What happens when the Qdrant vector store is temporarily unavailable?
- How does the system handle malformed queries or queries in languages not supported by the embeddings?
- What occurs when the OpenAI Agent service is experiencing high latency or errors?
- How does the system respond when the selected text is extremely large or contains special characters?

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST expose a FastAPI health check endpoint that confirms all dependencies (Qdrant, OpenAI Agent) are accessible
- **FR-002**: System MUST provide a RAG query endpoint that accepts general book-wide questions and returns answers grounded in indexed content
- **FR-003**: System MUST provide a RAG query endpoint that accepts questions constrained to user-selected text and returns answers based only on that text
- **FR-004**: System MUST return final answers that are exclusively generated from retrieved Qdrant content via the OpenAI Agent
- **FR-005**: System MUST include retrieved source metadata (chapter/section identifiers) with each response
- **FR-006**: System MUST return clear "not found in book" responses when requested information is not available in indexed content
- **FR-007**: System MUST prevent direct model calls from the frontend, ensuring all processing occurs on the backend
- **FR-008**: System MUST load API keys via environment variables with no hardcoding
- **FR-009**: System MUST be deterministic and reproducible, providing consistent responses for identical inputs
- **FR-010**: System MUST be compatible with Docusaurus frontend for local development and deployment

### Key Entities

- **Query Request**: Represents a user's question, potentially including selected text constraints and metadata
- **RAG Response**: Contains the final answer, source metadata (chapter/section identifiers), and confidence indicators
- **Source Document**: Indexed book content stored in Qdrant with chapter/section information
- **Session Context**: Optional metadata tracking for query context and user interaction patterns

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Users can submit questions and receive relevant answers grounded in book content within 5 seconds under normal conditions
- **SC-002**: 95% of queries return responses with proper source attribution (chapter/section identifiers)
- **SC-003**: System correctly identifies and responds with "not found in book" for 90% of queries about information not present in indexed content
- **SC-004**: Frontend successfully integrates with backend API endpoints with 99% success rate for API calls
- **SC-005**: All responses are deterministic - identical queries return identical answers with proper grounding
- **SC-006**: Local development environment can be set up and running within 10 minutes following provided instructions
