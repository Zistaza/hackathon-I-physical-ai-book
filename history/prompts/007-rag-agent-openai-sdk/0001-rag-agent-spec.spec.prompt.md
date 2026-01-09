---
id: 0001
title: rag-agent-spec
stage: spec
date: 2026-01-08
surface: agent
model: Claude Sonnet 4.5
feature: 007-rag-agent-openai-sdk
branch: 007-rag-agent-openai-sdk
user: emizee
command: sp.specify
labels: ["rag", "agent", "openai", "specification"]
links:
  spec: /home/emizee/hackathon-I-physical-ai-book/specs/007-rag-agent-openai-sdk/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - /home/emizee/hackathon-I-physical-ai-book/specs/007-rag-agent-openai-sdk/spec.md
  - /home/emizee/hackathon-I-physical-ai-book/specs/007-rag-agent-openai-sdk/checklists/requirements.md
tests:
  - No specific tests added yet
---

## Prompt

RAG Agent – OpenAI Agents SDK with Retrieval Integration

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
- Advanced ranking or summarization pipelines

## Response snapshot

Created comprehensive specification for RAG Agent with OpenAI Agents SDK, including:
- User scenarios for grounded QA (P1), selected text mode (P2), and safe refusal/determinism (P3)
- 13 functional requirements covering core agent functionality
- Key entities including RAG Agent, Retrieval Tool, User Query, Retrieved Chunks, and Citation Metadata
- 6 measurable success outcomes
- Assumptions, technical constraints, and quality/governance requirements
- Quality validation checklist confirming specification completeness

## Outcome

- ✅ Impact: Created complete feature specification for RAG agent with OpenAI Agents SDK, enabling deterministic, source-grounded question answering
- 🧪 Tests: No specific tests added yet
- 📁 Files: Created spec.md and requirements checklist
- 🔁 Next prompts: Ready for planning phase with /sp.plan
- 🧠 Reflection: Specification covers all required elements with clear user stories, functional requirements, and success criteria

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): Not applicable
- Prompt variant (if applicable): Not applicable
- Next experiment (smallest change to try): Proceed with planning phase
