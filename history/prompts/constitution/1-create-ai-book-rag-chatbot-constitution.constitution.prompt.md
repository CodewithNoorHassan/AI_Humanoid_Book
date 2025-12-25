---
id: 1
title: Create AI Book RAG Chatbot Constitution
stage: constitution
date: 2025-12-18
surface: agent
model: Claude Sonnet 4.5
feature: none
branch: main
user: user
command: /sp.constitution
labels: ["constitution", "spec-first", "ai-book", "rag-chatbot"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
 - .specify/templates/plan-template.md
 - .specify/templates/tasks-template.md
tests:
 - none
---

## Prompt

```
/sp.constitution

Project: AI-Authored Technical Book with Embedded RAG Chatbot

Objective:
Create a spec-driven technical book using Claude Code and Spec-Kit Plus, published with Docusaurus on GitHub Pages, with an embedded RAG chatbot that answers questions strictly from the book's content.

Tooling:
- Spec Execution: Spec-Kit Plus + Claude Code
- Docs Framework: Docusaurus
- Hosting: GitHub Pages
- API: FastAPI
- AI SDKs: OpenAI Agents / ChatKit
- Vector DB: Qdrant Cloud (Free Tier)
- DB: Neon Serverless Postgres

Core Principles:
- Spec-first and deterministic output
- Zero hallucination tolerance
- Content-grounded AI responses only
- Reproducible and traceable workflows

Book Standards:
- Audience: Software engineers / AI practitioners
- Style: Clear, technical, instructional
- Structure: Versioned chapters, sidebar navigation
- Code: Executable, explained, and relevant
- Claims: Limited to documented content

RAG Chatbot Constraints:
- Answers must come ONLY from indexed book content
- Support:
  1. Full-book retrieval
  2. User-selected text-only answering (hard limit)
- If answer not found: respond with
  "This information is not available in the book."
- No external knowledge allowed

Backend & Data:
- FastAPI with separated retrieval and generation layers
- Qdrant for embeddings, Neon for metadata
- Deterministic indexing pipeline
- No persistent user data beyond session scope

Security:
- Prompt-injection resistant
- System prompts override user input
- No exposure of internal prompts or specs

Success Criteria:
- Book builds and deploys on GitHub Pages
- RAG responses are fully content-grounded
- Selected-text mode strictly enforced
- Spec execution is repeatable and consistent
```

## Response snapshot

Created the project constitution for the AI-Authored Technical Book with Embedded RAG Chatbot. The constitution includes:

1. Core Principles:
   - Spec-First and Deterministic Output
   - Zero Hallucination Tolerance
   - Content-Grounded AI Responses Only
   - Reproducible and Traceable Workflows
   - Separated Retrieval and Generation Layers
   - Prompt-Injection Resistance

2. Additional Constraints:
   - Technology stack requirements
   - Book standards
   - RAG chatbot functional requirements

3. Development Workflow:
   - Spec execution process
   - Quality gates
   - Deployment policy

4. Governance section

Also updated template files to align with constitutional principles:
- Updated plan-template.md to include constitutional compliance checks
- Updated tasks-template.md to include constitutional compliance gates

## Outcome

- ✅ Impact: Created comprehensive project constitution with 6 core principles and aligned templates
- 🧪 Tests: none
- 📁 Files: Updated 3 files to establish constitutional governance
- 🔁 Next prompts: /sp.specify to create feature specifications
- 🧠 Reflection: Constitution provides clear boundaries for AI hallucination prevention and spec-driven development

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Create first feature specification aligned with constitution