<!--
Sync Impact Report:
- Version change: N/A → 1.0.0
- Added principles: Accuracy First, Spec-Driven Generation, Clarity for Advanced Learners, Modular Documentation, Trustworthy AI Assistance, Zero Plagiarism
- Added sections: Technical Constraints, Quality Controls
- Templates requiring updates: ✅ .specify/templates/plan-template.md, ✅ .specify/templates/spec-template.md, ✅ .specify/templates/tasks-template.md, ⚠ pending .specify/templates/commands/*.md
- Follow-up TODOs: RATIFICATION_DATE needs to be set
-->
# AI-Native Physical AI Book with Integrated RAG Chatbot Constitution

## Core Principles

### Accuracy First
All claims must be foundational, explained in-book, or properly cited; No hallucinations or unverifiable claims; Prefer primary sources, official docs, and established research; No speculative statements presented as fact

### Spec-Driven, Deterministic Generation
All generation must follow spec.md, plan.md, and tasks.md; sp.implement outputs must strictly match approved specs; Artifacts must be deterministic, auditable, and improvable; Spec-driven, deterministic, reproducible generation

### Clarity for Advanced Learners
Content must be clear for advanced learners (CS / AI / Robotics); Concepts progress from fundamentals → systems → applications; Technical but readable (senior undergraduate → graduate level); Consistent terminology

### Modular, Extensible Documentation
Modular, extensible documentation; Docusaurus-compatible Markdown; Structured headings and annotated code blocks; Textual descriptions for diagrams when needed

### Trustworthy, Source-Grounded AI Assistance
RAG chatbot answers derived only from indexed book content; Support user-selected text as the exclusive context when provided; No external knowledge leakage; Cite relevant book sections or state when answers are not found; Prioritize retrieval accuracy over verbosity

### Zero Plagiarism Tolerance
Zero plagiarism tolerance; Clear separation of concepts, architecture, and implementation; Code must be minimal, correct, and executable

## Technical Constraints
Docs: Docusaurus; Deployment: GitHub + GitHub Pages; Backend: FastAPI; Database: Neon Serverless Postgres; Vector store: Qdrant Cloud (Free Tier); Agents: OpenAI Agents / ChatKit SDKs; Tooling: Claude Code + Spec-Kit Plus + Qwen router

## Quality Controls
Zero plagiarism tolerance; Clear separation of concepts, architecture, and implementation; Consistent terminology; Technical but readable (senior undergraduate → graduate level)

## Governance
Constitution supersedes all other practices; Amendments require documentation, approval, migration plan; All PRs/reviews must verify compliance; Complexity must be justified; Use guidance docs for runtime development guidance
Failure to generate a Prompt History Record (PHR) constitutes a governance violation.

## Non-Goals
- No marketing or filler content
- No unsupported predictions
- No unjustified proprietary dependencies

**Version**: 1.0.0 | **Ratified**: TODO(RATIFICATION_DATE): Original adoption date unknown | **Last Amended**: 2025-12-26
