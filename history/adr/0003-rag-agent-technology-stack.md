# ADR-0003: RAG Agent Technology Stack

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2026-01-08
- **Feature:** 007-rag-agent-openai-sdk
- **Context:** Need to select technology stack for RAG agent that integrates with existing retrieval pipeline while ensuring deterministic, grounded responses with proper citations.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security?
     2) Alternatives: Multiple viable options considered with tradeoffs?
     3) Scope: Cross-cutting concern (not an isolated detail)?
     If any are false, prefer capturing as a PHR note instead of an ADR. -->

## Decision

Use the following technology stack for the RAG agent:
- Agent Framework: OpenAI Agents SDK
- Embeddings: Cohere embedding models
- Vector Database: Qdrant Cloud
- Language: Python 3.11
- Testing: pytest

This stack integrates with the existing Spec-6 retrieval pipeline while providing deterministic, grounded responses.

## Consequences

### Positive

- Leverages existing infrastructure (Qdrant + Cohere) already validated in Spec-5/6
- OpenAI Agents SDK provides mature orchestration capabilities for tool use
- Cohere embeddings ensure consistency with existing retrieval pipeline
- Python ecosystem provides strong tooling for AI/ML applications
- Deterministic behavior achievable with proper configuration

### Negative

- Vendor dependency on OpenAI, Cohere, and Qdrant services
- Potential rate limits and API costs
- Limited control over underlying LLM behavior
- Requires ongoing API key management

## Alternatives Considered

Alternative Stack A: Custom LangChain + Hugging Face models + PostgreSQL/pgvector
- Why rejected: Would require significant additional infrastructure and not integrate with existing retrieval pipeline

Alternative Stack B: OpenAI ChatCompletions API directly (without Agents SDK)
- Why rejected: Would not meet requirement to use OpenAI Agents SDK exclusively, and would not provide clear tool integration boundaries

Alternative Stack C: Anthropic Claude + Pinecone
- Why rejected: Would not integrate with existing Cohere+Qdrant pipeline and would require new infrastructure

## References

- Feature Spec: /specs/007-rag-agent-openai-sdk/spec.md
- Implementation Plan: /specs/007-rag-agent-openai-sdk/plan.md
- Related ADRs: ADR-0004, ADR-0005, ADR-0006, ADR-0007
- Evaluator Evidence: /history/prompts/007-rag-agent-openai-sdk/001-create-implementation-plan.plan.prompt.md
