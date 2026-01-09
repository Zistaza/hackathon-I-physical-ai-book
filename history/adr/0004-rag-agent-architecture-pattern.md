# ADR-0004: RAG Agent Architecture Pattern

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2026-01-08
- **Feature:** 007-rag-agent-openai-sdk
- **Context:** Need to establish clear architectural boundaries for the RAG agent to ensure separation of concerns between orchestration, retrieval, synthesis, and citation handling.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security?
     2) Alternatives: Multiple viable options considered with tradeoffs?
     3) Scope: Cross-cutting concern (not an isolated detail)?
     If any are false, prefer capturing as a PHR note instead of an ADR. -->

## Decision

Implement a clear separation of concerns with the following architectural pattern:
- Agent Orchestration Layer: Manages conversation flow via OpenAI Agents SDK
- Retrieval Tool Interface: Connects to existing Spec-6 retrieval pipeline
- Response Synthesis: Combines retrieved content into coherent answers
- Citation + Refusal Handling: Ensures proper attribution and safe refusal when needed

This pattern ensures each component has a single responsibility while working together as an integrated system.

## Consequences

### Positive

- Clear boundaries between different functional concerns
- Easier to test and maintain individual components
- Enables independent evolution of each layer
- Facilitates proper grounding and citation enforcement
- Supports both full-book and selected-text-only modes cleanly

### Negative

- More complex initial architecture with multiple interacting components
- Requires careful coordination between layers
- Additional overhead from component interactions
- May be over-engineered for simple use cases

## Alternatives Considered

Alternative Pattern A: Monolithic approach where all logic is in agent instructions
- Why rejected: Would not provide clear separation, harder to enforce grounding, difficult to test

Alternative Pattern B: Direct integration without tool abstraction
- Why rejected: Would create tight coupling between agent and retrieval, making future changes difficult

Alternative Pattern C: Server-based API calls instead of tools
- Why rejected: Would not meet requirement to use OpenAI Agents SDK tools, adds network complexity

## References

- Feature Spec: /specs/007-rag-agent-openai-sdk/spec.md
- Implementation Plan: /specs/007-rag-agent-openai-sdk/plan.md
- Related ADRs: ADR-0003, ADR-0005, ADR-0006, ADR-0007
- Evaluator Evidence: /history/prompts/007-rag-agent-openai-sdk/001-create-implementation-plan.plan.prompt.md
