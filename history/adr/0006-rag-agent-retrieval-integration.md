# ADR-0006: RAG Agent Retrieval Integration

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2026-01-08
- **Feature:** 007-rag-agent-openai-sdk
- **Context:** Need to integrate the RAG agent with the existing Spec-6 retrieval pipeline to ensure consistent access to book content while maintaining proper separation of concerns.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security?
     2) Alternatives: Multiple viable options considered with tradeoffs?
     3) Scope: Cross-cutting concern (not an isolated detail)?
     If any are false, prefer capturing as a PHR note instead of an ADR. -->

## Decision

Integrate with the existing Spec-6 retrieval pipeline through a dedicated retrieval tool that:
- Interfaces with the existing `RAGRetriever` class in `backend/retrieve.py`
- Implements tool schema compliant with OpenAI Agents SDK requirements
- Handles both full-book and selected-text-only modes
- Returns structured data with content, scores, and complete metadata
- Includes proper error handling and fallback behaviors

This maintains clear separation between agent orchestration and retrieval concerns.

## Consequences

### Positive

- Leverages existing, validated retrieval pipeline without duplication
- Maintains consistency with the established retrieval architecture
- Enables both full-book and selected-text-only modes as required
- Proper separation of concerns between agent and retrieval layers
- Reduces maintenance overhead by reusing existing infrastructure

### Negative

- Tight coupling to existing retrieval implementation
- Dependency on external backend components
- Potential performance bottlenecks if retrieval is slow
- Requires coordination between agent and retrieval teams

## Alternatives Considered

Alternative Integration A: Duplicate retrieval logic inside the agent
- Why rejected: Would violate DRY principle and create maintenance burden

Alternative Integration B: Direct database access from agent
- Why rejected: Would bypass existing retrieval infrastructure and create tight coupling

Alternative Integration C: Separate API service for retrieval
- Why rejected: Would add unnecessary complexity when direct integration is possible

## References

- Feature Spec: /specs/007-rag-agent-openai-sdk/spec.md
- Implementation Plan: /specs/007-rag-agent-openai-sdk/plan.md
- Related ADRs: ADR-0003, ADR-0004, ADR-0005, ADR-0007
- Evaluator Evidence: /history/prompts/007-rag-agent-openai-sdk/001-create-implementation-plan.plan.prompt.md
