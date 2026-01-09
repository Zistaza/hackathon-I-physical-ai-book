# Tasks: RAG Agent with OpenAI Agents SDK

**Feature**: RAG Agent with OpenAI Agents SDK
**Branch**: `007-rag-agent-openai-sdk`
**Generated**: 2026-01-09
**Spec**: [spec.md](spec.md) | **Plan**: [plan.md](plan.md)

## Implementation Strategy

Implement a RAG agent using the OpenAI Agents SDK that integrates with the existing retrieval pipeline. Following the override requirement, all agent logic will be contained in a single `backend/agent.py` file instead of splitting across multiple files. The agent will accept user questions, perform retrieval over the Qdrant + Cohere vector store, and generate grounded, source-cited answers strictly from retrieved book content.

## Dependencies

- User Story 1 (P1) - Grounded QA: Requires OpenAI Agents SDK integration and retrieval tool
- User Story 2 (P2) - Selected Text Only Mode: Depends on User Story 1 completion
- User Story 3 (P3) - Safe Refusal and Determinism: Depends on User Story 1 and 2 completion

## Parallel Execution Opportunities

- [P] Setup tasks can be executed in parallel with environment configuration
- [P] Documentation and testing tasks can run alongside implementation
- [P] Individual user story tests can run independently after core implementation

## Phase 1: Setup

### Story Goal
Initialize project structure and dependencies for the RAG agent implementation.

### Independent Test Criteria
All setup tasks completed successfully with dependencies installed and environment configured.

### Tasks

- [x] T001 Create project structure per implementation plan
- [x] T002 [P] Install OpenAI Agents SDK and related dependencies
- [x] T003 [P] Configure environment variables for API keys
- [x] T004 [P] Set up initial project configuration files
- [x] T005 [P] Verify connection to existing retrieval system

## Phase 2: Foundational Tasks

### Story Goal
Establish core agent infrastructure and integration with existing retrieval system.

### Independent Test Criteria
Core agent can be instantiated and communicates with the existing retrieval system.

### Tasks

- [x] T006 [P] Create main agent.py file structure
- [x] T007 [P] Initialize OpenAI client and assistant configuration
- [x] T008 [P] Implement system instructions enforcing grounding requirements
- [x] T009 [P] Create retrieval tool that interfaces with existing RAGRetriever
- [x] T010 [P] Test basic communication between agent and retrieval system

## Phase 3: User Story 1 - Grounded QA (Priority: P1)

### Story Goal
As an AI/Robotics engineer or Advanced CS student, I want to ask questions about the book content so that I can get accurate, source-cited answers that are grounded exclusively in the retrieved book content.

### Independent Test Criteria
The agent can be tested by providing user questions and verifying that the responses are grounded in retrieved content with proper citations to module/chapter/URL, and that no external knowledge is injected.

### Tasks

- [x] T011 [P] [US1] Implement basic query processing in agent.py
- [x] T012 [P] [US1] Create retrieval tool with proper schema for OpenAI Agents SDK
- [x] T013 [P] [US1] Integrate retrieval tool with existing RAGRetriever class
- [x] T014 [P] [US1] Implement response synthesis using retrieved content only
- [x] T015 [P] [US1] Add citation formatting for module/chapter/URL metadata
- [x] T016 [US1] Test basic question answering with proper citations
- [x] T017 [US1] Validate that responses are grounded in retrieved chunks only
- [x] T018 [US1] Ensure no external knowledge is injected into responses
- [x] T019 [US1] Verify all Functional Requirements (FR-001 to FR-005) are met

## Phase 4: User Story 2 - Selected Text Only Mode (Priority: P2)

### Story Goal
As an AI/Robotics engineer or Advanced CS student, I want to constrain the agent's retrieval to only user-selected text so that answers are generated exclusively from the specified content without considering other book content.

### Independent Test Criteria
The agent can be tested by providing selected text along with a question, and verifying that retrieval is constrained strictly to the selected text while ignoring all other book content.

### Tasks

- [x] T020 [P] [US2] Extend retrieval tool to support selected-text-only mode
- [x] T021 [P] [US2] Implement logic to restrict retrieval to selected text content
- [x] T022 [P] [US2] Update agent instructions to handle selected text mode
- [x] T023 [US2] Test selected-text-only mode functionality
- [x] T024 [US2] Verify that other book content is ignored in selected mode
- [x] T025 [US2] Test scenarios where selected text is insufficient to answer
- [x] T026 [US2] Ensure proper refusal responses when selected text lacks required information
- [x] T027 [US2] Verify Functional Requirements FR-007 and FR-008 are met

## Phase 5: User Story 3 - Safe Refusal and Determinism (Priority: P3)

### Story Goal
As an AI/Robotics engineer or Advanced CS student, I want the agent to safely refuse questions it cannot answer and produce deterministic outputs so that I can trust the agent's responses and reproduce identical results.

### Independent Test Criteria
The agent can be tested by providing identical inputs multiple times and verifying that identical outputs are produced, and by providing unrelated queries to verify safe refusal.

### Tasks

- [x] T028 [P] [US3] Implement safe refusal logic for unanswered questions
- [x] T029 [P] [US3] Create logic to detect when answers are not found in retrieved content
- [x] T030 [P] [US3] Implement deterministic processing of identical inputs
- [x] T031 [P] [US3] Add deterministic validation for identical query results
- [x] T032 [US3] Test determinism by running identical queries multiple times
- [x] T033 [US3] Test safe refusal for unrelated or unanswerable queries
- [x] T034 [US3] Handle empty retrieval results appropriately
- [x] T035 [US3] Handle conflicting information in retrieved chunks
- [x] T036 [US3] Verify all Functional Requirements (FR-009 to FR-013) are met

## Phase 6: Testing & Validation

### Story Goal
Create comprehensive test suite to validate all functional requirements and user stories.

### Independent Test Criteria
All tests pass successfully and validate the agent meets the specified requirements.

### Tasks

- [x] T037 [P] Create unit tests for individual agent components
- [x] T038 [P] Create integration tests for end-to-end functionality
- [x] T039 [P] Create validation tests for grounding requirements
- [x] T040 [P] Create determinism validation tests
- [x] T041 [P] Create edge case tests for empty/conflicting results
- [x] T042 Run all tests and verify they pass
- [x] T043 Validate all user stories meet acceptance criteria
- [x] T044 Verify all functional requirements are satisfied

## Phase 7: Polish & Cross-Cutting Concerns

### Story Goal
Complete the implementation with proper error handling, documentation, and validation.

### Independent Test Criteria
Agent is production-ready with proper error handling and documentation.

### Tasks

- [x] T045 [P] Add comprehensive error handling and logging
- [x] T046 [P] Add input validation for all parameters
- [x] T047 [P] Add proper exception handling for network issues
- [x] T048 [P] Update documentation for the agent functionality
- [x] T049 [P] Add inline code documentation and comments
- [x] T050 Final validation of all requirements and user stories
- [x] T051 Clean up and optimize code for production use