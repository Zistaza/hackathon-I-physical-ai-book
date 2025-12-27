---
id: 1
title: update-project-constitution
stage: constitution
date: 2025-12-26
surface: agent
model: claude-sonnet-4-5-20250929
feature: constitution
branch: main
user: emizee
command: sp.constitution
labels: [constitution, spec-driven-development, physical-ai]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
  - none
---

## Prompt

```text
Project: AI-Native Physical AI Book with Integrated RAG Chatbot

Mission:
Produce a high-quality technical book on Physical AI using AI-native, spec-driven workflows (Claude Code + Spec-Kit Plus), published with Docusaurus and enhanced with a RAG chatbot that answers questions strictly from the book's content.

Core principles:
- Accuracy first (no hallucinations or unverifiable claims)
- Spec-driven, deterministic, reproducible generation
- Clarity for advanced learners (CS / AI / Robotics)
- Modular, extensible documentation
- Trustworthy, source-grounded AI assistance

Authoring standards:
- All claims must be foundational, explained in-book, or properly cited
- Prefer primary sources, official docs, and established research
- No speculative statements presented as fact
- Code must be minimal, correct, and executable
- Concepts progress from fundamentals → systems → applications

Content scope:
- Physical AI and embodied intelligence foundations
- AI-native software and agent systems
- Practical engineering (architecture, tooling, deployment)
- RAG-based knowledge interaction with the book

RAG chatbot rules:
- Answers derived only from indexed book content
- Support user-selected text as the exclusive context when provided
- No external knowledge leakage
- Cite relevant book sections or state when answers are not found
- Prioritize retrieval accuracy over verbosity

Technical constraints:
- Docs: Docusaurus
- Deployment: GitHub + GitHub Pages
- Backend: FastAPI
- Database: Neon Serverless Postgres
- Vector store: Qdrant Cloud (Free Tier)
- Agents: OpenAI Agents / ChatKit SDKs
- Tooling: Claude Code + Spec-Kit Plus + Qwen router

Spec-Kit Plus governance:
- All generation must follow spec.md, plan.md, and tasks.md
- sp.implement outputs must strictly match approved specs
- Artifacts must be deterministic, auditable, and improvable

Quality controls:
- Zero plagiarism tolerance
- Clear separation of concepts, architecture, and implementation
- Consistent terminology
- Technical but readable (senior undergraduate → graduate level)

Formatting:
- Docusaurus-compatible Markdown
- Structured headings and annotated code blocks
- Textual descriptions for diagrams when needed

Success criteria:
- Book builds and deploys successfully
- Embedded RAG chatbot functions correctly
- All chatbot answers are book-grounded
- Spec-Kit artifacts remain traceable and consistent
- Demonstrates AI-native documentation + interaction

Non-goals:
- No marketing or filler content
- No unsupported predictions
- No unjustified proprietary dependencies
```

You **MUST** consider the user input before proceeding (if not empty).

## Outline

You are updating the project constitution at `.specify/memory/constitution.md`. This file is a TEMPLATE containing placeholder tokens in square brackets (e.g. `[PROJECT_NAME]`, `[PRINCIPLE_1_NAME]`). Your job is to (a) collect/derive concrete values, (b) fill the template precisely, and (c) propagate any amendments across dependent artifacts.

Follow this execution flow:

1. Load the existing constitution template at `.specify/memory/constitution.md`.
   - Identify every placeholder token of the form `[ALL_CAPS_IDENTIFIER]`.
   **IMPORTANT**: The user might require less or more principles than the ones used in the template. If a number is specified, respect that - follow the general template. You will update the doc accordingly.

2. Collect/derive values for placeholders:
   - If user input (conversation) supplies a value, use it.
   - Otherwise infer from existing repo context (README, docs, prior constitution versions if embedded).
   - For governance dates: `RATIFICATION_DATE` is the original adoption date (if unknown ask or mark TODO), `LAST_AMENDED_DATE` is today if changes are made, otherwise keep previous.
   - `CONSTITUTION_VERSION` must increment according to semantic versioning rules:
     - MAJOR: Backward incompatible governance/principle removals or redefinitions.
     - MINOR: New principle/section added or materially expanded guidance.
     - PATCH: Clarifications, wording, typo fixes, non-semantic refinements.
   - If version bump type ambiguous, propose reasoning before finalizing.

3. Draft the updated constitution content:
   - Replace every placeholder with concrete text (no bracketed tokens left except intentionally retained template slots that the project has chosen not to define yet—explicitly justify any left).
   - Preserve heading hierarchy and comments can be removed once replaced unless they still add clarifying guidance.
   - Ensure each Principle section: succinct name line, paragraph (or bullet list) capturing non‑negotiable rules, explicit rationale if not obvious.
   - Ensure Governance section lists amendment procedure, versioning policy, and compliance review expectations.

4. Consistency propagation checklist (convert prior checklist into active validations):
   - Read `.specify/templates/plan-template.md` and ensure any "Constitution Check" or rules align with updated principles.
   - Read `.specify/templates/spec-template.md` for scope/requirements alignment—update if constitution adds/removes mandatory sections or constraints.
   - Read `.specify/templates/tasks-template.md` and ensure task categorization reflects new or removed principle-driven task types (e.g., observability, versioning, testing discipline).
   - Read each command file in `.specify/templates/commands/*.md` (including this one) to verify no outdated references (agent-specific names like CLAUDE only) remain when generic guidance is required.
   - Read any runtime guidance docs (e.g., `README.md`, `docs/quickstart.md`, or agent-specific guidance files if present). Update references to principles changed.

5. Produce a Sync Impact Report (prepend as an HTML comment at top of the constitution file after update):
   - Version change: old → new
   - List of modified principles (old title → new title if renamed)
   - Added sections
   - Removed sections
   - Templates requiring updates (✅ updated / ⚠ pending) with file paths
   - Follow-up TODOs if any placeholders intentionally deferred.

6. Validation before final output:
   - No remaining unexplained bracket tokens.
   - Version line matches report.
   - Dates ISO format YYYY-MM-DD.
   - Principles are declarative, testable, and free of vague language ("should" → replace with MUST/SHOULD rationale where appropriate).

7. Write the completed constitution back to `.specify/memory/constitution.md` (overwrite).

8. Output a final summary to the user with:
   - New version and bump rationale.
   - Any files flagged for manual follow-up.
   - Suggested commit message (e.g., `docs: amend constitution to vX.Y.Z (principle additions + governance update)`).

Formatting & Style Requirements:

- Use Markdown headings exactly as in the template (do not demote/promote levels).
- Wrap long rationale lines to keep readability (<100 chars ideally) but do not hard enforce with awkward breaks.
- Keep a single blank line between sections.
- Avoid trailing whitespace.

If the user supplies partial updates (e.g., only one principle revision), still perform validation and version decision steps.

If critical info missing (e.g., ratification date truly unknown), insert `TODO(<FIELD_NAME>): explanation` and include in the Sync Impact Report under deferred items.

Do not create a new template; always operate on the existing `.specify/memory/constitution.md` file.

---

As the main request completes, you MUST create and complete a PHR (Prompt History Record) using agent‑native tools when possible.

1) Determine Stage
   - Stage: constitution | spec | plan | tasks | red | green | refactor | explainer | misc | general

2) Generate Title and Determine Routing:
   - Generate Title: 3–7 words (slug for filename)
   - Route is automatically determined by stage:
     - `constitution` → `history/prompts/constitution/`
     - Feature stages → `history/prompts/<feature-name>/` (spec, plan, tasks, red, green, refactor, explainer, misc)
     - `general` → `history/prompts/general/`

3) Create and Fill PHR (Shell first; fallback agent‑native)
   - Run: `.specify/scripts/bash/create-phr.sh --title "<title>" --stage <stage> [--feature <name>] --json`
   - Open the file and fill remaining placeholders (YAML + body), embedding full PROMPT_TEXT (verbatim) and concise RESPONSE_TEXT.
   - If the script fails:
     - Read `.specify/templates/phr-template.prompt.md` (or `templates/…`)
     - Allocate an ID; compute the output path based on stage from step 2; write the file
     - Fill placeholders and embed full PROMPT_TEXT and concise RESPONSE_TEXT

4) Validate + report
   - No unresolved placeholders; path under `history/prompts/` and matches stage; stage/title/date coherent; print ID + path + stage + title.
   - On failure: warn, don't block. Skip only for `/sp.phr`.
```

## Response snapshot

```text
I'll help you update the project constitution based on the provided user input. Let me start by examining the current constitution template to understand its structure and identify the placeholders that need to be filled.

[Detailed process of updating the constitution with all principles and sections mapped from user input, and the final constitution file content created and written to .specify/memory/constitution.md]
```

## Outcome

- ✅ Impact: Updated project constitution with 6 core principles, technical constraints, and quality controls sections
- 🧪 Tests: none
- 📁 Files: Updated .specify/memory/constitution.md with new project-specific constitution
- 🔁 Next prompts: Ready for spec creation and planning phases
- 🧠 Reflection: Successfully transformed template constitution into project-specific document with all principles from user requirements

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): PASS - All placeholders replaced, valid structure maintained
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Create project spec based on constitution principles