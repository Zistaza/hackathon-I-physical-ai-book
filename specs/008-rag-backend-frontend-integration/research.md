# Research: RAG Backend Frontend Integration

## Decision: FastAPI Framework Selection
**Rationale**: FastAPI chosen for its automatic API documentation, type hints support, async capabilities, and excellent integration with Pydantic models. It aligns perfectly with the spec requirements for a well-documented, type-safe backend service.

**Alternatives considered**:
- Flask: More familiar but lacks automatic documentation and type validation
- Django: Overkill for API-only service with simpler requirements
- Express.js: Would require changing to Node.js ecosystem

## Decision: API Endpoint Structure
**Rationale**: Following RESTful patterns with clear, predictable endpoints that serve the frontend needs:
- GET /health: System health and dependency availability check
- POST /rag/query: General book-wide questions
- POST /rag/query-selected-text: Questions constrained to user-selected text

**Alternatives considered**:
- GraphQL: More flexible but adds complexity for simple query use case
- Single endpoint with mode parameter: Less clear separation of concerns

## Decision: Request/Response Model Design
**Rationale**: Using Pydantic models for automatic validation, serialization, and clear API contracts. Models will include:
- QueryRequest: question text, selected text (optional), metadata
- QueryResponse: answer, citations, source metadata, refusal indicators
- HealthResponse: system status, dependency availability

## Decision: Agent Integration Approach
**Rationale**: Wrapping existing agent.py functionality in service layer to maintain separation of concerns while reusing proven logic. The RAGService class will handle the orchestration between frontend requests and backend agent/retrieval logic.

## Decision: Frontend Integration Pattern
**Rationale**: Creating a React-based chatbot component that can be embedded in Docusaurus pages. The component will handle:
- Query submission to backend API
- Loading states and error handling
- Display of answers with source citations
- Selected text integration capability

## Decision: Environment Configuration
**Rationale**: Using pydantic-settings for robust environment variable handling with validation. This ensures API keys are properly loaded from environment variables without hardcoding.

## Decision: Error Handling Strategy
**Rationale**: Comprehensive error handling at all levels:
- HTTP-level: Proper status codes (200, 400, 500)
- Application-level: Detailed error responses with context
- Client-level: Graceful degradation when backend unavailable

## Decision: Session Tracking (Optional)
**Rationale**: For Neon Postgres session tracking, using a lightweight approach with optional session IDs passed in requests. This enables conversation history without requiring authentication.