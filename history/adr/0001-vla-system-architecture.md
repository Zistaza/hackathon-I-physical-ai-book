# ADR-0001: VLA System Architecture

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-28
- **Feature:** 004-vla-physical-ai
- **Context:** Need to design an architecture for Vision-Language-Action (VLA) systems that integrates vision processing, language understanding, and action execution components to enable robots to respond to natural language commands. The system must be suitable for educational purposes while maintaining technical accuracy and practical applicability.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security?
     2) Alternatives: Multiple viable options considered with tradeoffs?
     3) Scope: Cross-cutting concern (not an isolated detail)?
     If any are false, prefer capturing as a PHR note instead of an ADR. -->

## Decision

The VLA system will be implemented using a modular three-component architecture with clear separation of concerns:

- **Vision Processing Module**: Handles visual input from cameras and sensors, performs object detection and scene understanding
- **Language Processing Module**: Processes natural language input using OpenAI Whisper and LLMs for command interpretation
- **Action Execution Module**: Executes physical actions on the robot based on processed commands using ROS 2
- **Integration Layer**: Coordinates between all modules with a shared world state representation
- **Technology Stack**: Docusaurus documentation framework, OpenAI Whisper, ROS 2, NVIDIA Isaac SDK, Isaac Sim

## Consequences

### Positive

- Clear separation of concerns enables independent development and testing of each component
- Modular design allows for component replacement or upgrades without affecting the entire system
- Educational value is enhanced by the transparent architecture that clearly demonstrates each system function
- Integration with industry-standard tools (ROS 2, NVIDIA Isaac) ensures practical relevance
- Shared world state enables consistent system behavior across all components

### Negative

- Increased complexity compared to monolithic approaches
- Potential communication overhead between components
- Dependency on multiple external frameworks and tools
- Learning curve for users unfamiliar with ROS 2 and NVIDIA Isaac ecosystem
- Additional infrastructure requirements for simulation and testing

## Alternatives Considered

**Alternative 1: End-to-end Neural Network Architecture**
- Approach: Single neural network that maps directly from vision and language inputs to action outputs
- Why rejected: Less explainable, harder to debug, difficult to maintain, and less suitable for educational purposes where understanding component interactions is crucial

**Alternative 2: Custom Middleware Instead of ROS 2**
- Approach: Develop custom communication middleware instead of using ROS 2
- Why rejected: ROS 2 provides mature, well-documented, and widely adopted solutions for robotics communication, making custom development unnecessary and risky

**Alternative 3: Cloud-Based Processing Instead of Local Integration**
- Approach: Host components separately in cloud services with API-based communication
- Why rejected: Would introduce latency issues critical for real-time robotics, require persistent network connectivity, and reduce system reliability

## References

- Feature Spec: specs/004-vla-physical-ai/spec.md
- Implementation Plan: specs/004-vla-physical-ai/plan.md
- Related ADRs: None
- Evaluator Evidence: specs/004-vla-physical-ai/research.md
