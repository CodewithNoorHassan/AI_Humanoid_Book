<!-- SYNC IMPACT REPORT
Version change: N/A (initial) → 1.0.0
Modified principles: None (new constitution)
Added sections: All sections (new document)
Removed sections: None
Templates requiring updates:
  - .specify/templates/plan-template.md ✅ updated
  - .specify/templates/spec-template.md ✅ updated
  - .specify/templates/tasks-template.md ✅ updated
  - .specify/templates/commands/*.md ⚠ pending
Follow-up TODOs: None
-->

# AI-Authored Technical Book with Embedded RAG Chatbot Constitution

## Core Principles

### Spec-First and Deterministic Output
Every feature and change begins with a specification that defines expected behavior and outcomes; All outputs must be reproducible and traceable to the specification; Clear success criteria required before implementation begins.

### Zero Hallucination Tolerance
The RAG chatbot must only respond with information grounded in the book's content; If information is not available in the book, respond with "This information is not available in the book."; No external knowledge or assumptions allowed in AI responses.

### Content-Grounded AI Responses Only
All chatbot answers must come exclusively from indexed book content; Support both full-book retrieval and user-selected text-only answering modes; Strict enforcement of content boundaries to prevent hallucinations.

### Reproducible and Traceable Workflows
All development processes must be repeatable and version-controlled; Every change must be tracked with clear audit trails; Documentation and code generation must follow deterministic patterns.

### Separated Retrieval and Generation Layers
Backend architecture must maintain clear separation between content retrieval and response generation; FastAPI services handle retrieval separately from LLM generation; Qdrant handles embeddings while Neon manages metadata.

### Prompt-Injection Resistance
System prompts must override user input to prevent manipulation; Security measures must protect against prompt injection attacks; Internal prompts and specifications must not be exposed to users.

## Additional Constraints

### Technology Stack Requirements
- Frontend: Docusaurus for documentation framework
- Hosting: GitHub Pages for static site hosting
- Backend: FastAPI for API services
- Vector Database: Qdrant Cloud (Free Tier) for embeddings
- Metadata Database: Neon Serverless Postgres for metadata storage
- AI SDKs: OpenAI Agents / ChatKit for AI functionality

### Book Standards
- Target Audience: Software engineers and AI practitioners
- Writing Style: Clear, technical, and instructional
- Structure: Versioned chapters with sidebar navigation
- Code Examples: Executable, explained, and relevant to content
- Claims: Limited to documented content within the book

### RAG Chatbot Functional Requirements
- Full-book retrieval capability for comprehensive queries
- User-selected text-only answering mode with hard enforcement
- Strict content grounding with zero tolerance for hallucinations
- Session-limited user data persistence (no persistent user profiles)

## Development Workflow

### Spec Execution Process
- Use Spec-Kit Plus and Claude Code for specification-driven development
- Follow the sequence: Constitution → Spec → Plan → Tasks → Implementation
- All changes must be spec-driven and validated against requirements

### Quality Gates
- All code must pass automated testing before merging
- Content must be validated for accuracy and grounding in book material
- RAG responses must undergo hallucination testing
- Security reviews required for AI and database components

### Deployment Policy
- GitHub Actions automate deployment to GitHub Pages
- Staging environment required for content validation
- Rollback procedures must be tested and documented
- Versioning follows semantic versioning principles

## Governance

This constitution governs all development activities for the AI-Authored Technical Book with Embedded RAG Chatbot project. All team members must comply with these principles and constraints. Amendments to this constitution require documentation of changes, approval from project stakeholders, and a migration plan for existing implementations. All pull requests and code reviews must verify compliance with constitutional principles. Development guidance follows the Spec-Kit Plus methodology as outlined in project documentation.

**Version**: 1.0.0 | **Ratified**: 2025-12-18 | **Last Amended**: 2025-12-18