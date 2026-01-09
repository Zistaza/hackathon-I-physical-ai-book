# Implementation Plan: RAG Agent with OpenAI Agents SDK

**Branch**: `007-rag-agent-openai-sdk` | **Date**: 2026-01-08 | **Spec**: [link to spec.md]
**Input**: Feature specification from `/specs/007-rag-agent-openai-sdk/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Implementation of a RAG (Retrieval-Augmented Generation) agent using the OpenAI Agents SDK that integrates with the existing Spec-6 retrieval pipeline. The agent will accept user questions, perform retrieval over the Qdrant + Cohere vector store, and generate grounded, source-cited answers strictly from retrieved book content. The implementation emphasizes determinism, grounding, citation, and safe refusal behaviors as specified in the feature requirements.

## Technical Context

**Language/Version**: Python 3.11
**Primary Dependencies**: OpenAI Agents SDK, Cohere, Qdrant Client, python-dotenv
**Storage**: Qdrant Cloud (vector database), with metadata stored alongside embeddings
**Testing**: pytest for unit and integration tests, with deterministic validation tests
**Target Platform**: Linux server environment
**Project Type**: Single service (agent layer only, no UI)
**Performance Goals**: <500ms response time for typical queries, deterministic outputs for identical inputs
**Constraints**: Must use OpenAI Agents SDK exclusively (no ChatCompletion or single-prompt patterns), retrieval via existing Spec-6 pipeline, no UI components
**Scale/Scope**: Single-turn agent interactions, headless operation for testing

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

Based on the constitution.md file, the following gates must be satisfied:
1. **Accuracy First**: Agent must answer only from retrieved content, no hallucinations allowed
2. **Spec-Driven, Deterministic Generation**: Must follow spec.md requirements exactly, ensure deterministic outputs
3. **Trustworthy, Source-Grounded AI Assistance**: Must support user-selected text as exclusive context, cite relevant book sections, state when answers are not found
4. **Zero Plagiarism Tolerance**: Clear separation between concepts, architecture, and implementation

All gates are satisfied as the design strictly follows the spec requirements for grounding, citation, and deterministic behavior.

## Project Structure

### Documentation (this feature)

```text
specs/007-rag-agent-openai-sdk/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
│   └── rag-agent-contracts.md
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
src/
├── agents/
│   ├── __init__.py
│   ├── rag_agent.py         # Main RAG agent implementation
│   └── agent_factory.py     # Agent creation and configuration
├── tools/
│   ├── __init__.py
│   ├── retrieval_tool.py    # Interface to existing retrieval system
│   └── tool_registry.py     # Registry of available tools
├── models/
│   ├── __init__.py
│   ├── query.py             # Query data models
│   ├── response.py          # Response data models
│   └── chunk.py             # Retrieved chunk models
├── services/
│   ├── __init__.py
│   ├── rag_service.py       # Main RAG service orchestration
│   └── validation_service.py # Response validation
└── utils/
    ├── __init__.py
    ├── deterministic_hash.py # Determinism validation utilities
    └── citation_formatter.py # Citation formatting utilities

tests/
├── unit/
│   ├── test_agents/
│   ├── test_tools/
│   ├── test_models/
│   └── test_services/
├── integration/
│   └── test_agent_integration.py
└── contract/
    └── test_determinism.py
```

**Structure Decision**: Single service architecture chosen to implement the agent layer only, with clear separation of concerns between agents, tools, models, and services. This aligns with the requirement to implement only the agent layer without UI components.

## Architecture Sketch

### High-Level Agent Execution Flow (Single-Turn)
```
User Query
    ↓
Agent Orchestration Layer
    ↓
Retrieval Tool (interfaces with Spec-6 pipeline)
    ↓
Qdrant + Cohere Vector Store
    ↓
Retrieved Chunks with Metadata
    ↓
Response Synthesis & Citation Logic
    ↓
Grounded Answer with Citations OR Safe Refusal
```

### Clear Separation of Concerns:
- **Agent Orchestration**: Manages conversation flow via OpenAI Agents SDK
- **Retrieval Tool Interface**: Connects to existing Spec-6 retrieval pipeline (`backend/retrieve.py`)
- **Response Synthesis**: Combines retrieved content into coherent answers
- **Citation + Refusal Handling**: Ensures proper attribution and safe refusal when needed

## Phase Structure

### Phase 0: Research & SDK Alignment
- Researched OpenAI Agents SDK capabilities and limitations
- Analyzed existing Spec-6 retrieval pipeline (`backend/retrieve.py`)
- Identified integration points between agent and retrieval system
- Resolved all technical unknowns and dependencies

### Phase 1: Agent Foundation
- Initialize OpenAI client and create assistant with proper instructions
- Implement system instructions enforcing grounding, citation, and refusal requirements
- Establish determinism strategy using consistent parameters and deterministic processing
- Create agent factory for proper configuration and instantiation

### Phase 2: Retrieval Tool Integration
- Develop retrieval tool that interfaces with existing `RAGRetriever` class
- Implement tool schema compliant with OpenAI Agents SDK requirements
- Handle both full-book and selected-text-only modes as specified
- Ensure proper error handling and fallback behaviors

### Phase 3: Grounded Answer Synthesis & Citation Logic
- Implement response synthesis that strictly uses retrieved content
- Develop citation formatting following metadata structure from retrieval system
- Create logic to handle conflicting information in retrieved chunks
- Implement proper source attribution in all responses

### Phase 4: Safe Refusal Handling
- Implement detection of insufficient retrieval results
- Create proper refusal messages when content not found
- Handle edge cases like empty retrieval results and conflicting information
- Ensure graceful degradation without hallucinations

### Phase 5: Deterministic Validation & Headless Testing
- Implement deterministic validation tests ensuring identical inputs produce identical outputs
- Create comprehensive test suite mapping to User Stories (P1–P3) and Functional Requirements (FR-001 → FR-013)
- Develop headless testing framework for automated validation
- Verify grounding, citation, and determinism requirements

## Decisions Requiring Documentation

### 1. Agent Instruction Strategy and Constraints
- **Decision**: Use explicit system instructions that mandate grounding in retrieved content
- **Rationale**: Ensures the agent follows requirements without relying on model behavior alone
- **Alternatives Considered**: Post-processing validation vs. instruction-based enforcement
- **Chosen Approach**: Instruction-based with tool-assisted validation

### 2. Tool Schema Design for Retrieval Results
- **Decision**: Create structured tool that returns both content and metadata
- **Rationale**: Enables proper citation and validation of retrieved content
- **Alternatives Considered**: Simple text-only return vs. structured object with metadata
- **Chosen Approach**: Structured object with content, scores, and complete metadata

### 3. Citation Formatting and Conflict Reporting
- **Decision**: Format citations with module/chapter/URL as required by spec
- **Rationale**: Meets functional requirement FR-005 for proper attribution
- **Alternatives Considered**: Various citation formats and levels of detail
- **Chosen Approach**: Consistent format matching metadata structure from retrieval system

### 4. Determinism Enforcement Mechanisms
- **Decision**: Use deterministic sorting of retrieved results and consistent processing
- **Rationale**: Ensures FR-009 requirement of identical outputs for identical inputs
- **Alternatives Considered**: Random sampling vs. deterministic ordering
- **Chosen Approach**: Score-based ordering with ID tie-breaker

### 5. Refusal Criteria Thresholds
- **Decision**: Refuse when no relevant content found above similarity threshold
- **Rationale**: Prevents hallucinations when information is genuinely not in the book
- **Alternatives Considered**: Various confidence/similarity thresholds
- **Chosen Approach**: Use existing RETRIEVAL_THRESHOLD from retrieval system

## Testing & Validation Strategy

### Direct Mapping to Requirements:
- **User Story 1 (P1)**: Test correct answers with proper citations from book content
- **User Story 2 (P2)**: Test selected-text-only mode constraints and behavior
- **User Story 3 (P3)**: Test determinism and safe refusal capabilities
- **Functional Requirements FR-001 to FR-013**: Specific test cases for each requirement

### Explicit Validation for:
- **Grounding**: Verify all answers derive only from retrieved chunks
- **Citation Completeness**: Ensure every answer includes proper source citations
- **Selected-Text-Only Enforcement**: Validate that other content is ignored when in this mode
- **Empty/Conflicting Retrieval Results**: Proper handling of edge cases
- **Deterministic Output**: Identical inputs produce identical outputs consistently

### Headless, Automated Testability:
- All tests runnable without UI interaction
- Deterministic validation with hash comparison
- Comprehensive coverage of functional requirements
- Automated validation of grounding and citation requirements

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| N/A | N/A | N/A |