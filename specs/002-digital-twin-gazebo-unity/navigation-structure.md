# Navigation Structure for Digital Twin Module

## Chapter Sequence

The digital twin module follows this sequence:

1. Chapter 1: Digital Twins in Physical AI
2. Chapter 2: Physics Simulation with Gazebo
3. Chapter 3: High-Fidelity Environments with Unity
4. Chapter 4: Sensor Simulation for Humanoid Robots

## Navigation Implementation

### Sidebar Navigation
- The sidebar (sidebars.ts) provides category-based navigation for the module
- Chapters are organized under "Module 2: Digital Twin (Gazebo & Unity)"
- Each chapter has a clear position in the navigation hierarchy

### Document-Level Navigation
Each chapter document should include navigation links at the bottom:

```markdown
## Next Steps

- **Previous**: [Link to previous chapter in the sequence]
- **Next**: [Link to next chapter in the sequence]
- **Module Index**: [Link back to module overview or main page]
```

### Navigation Patterns

#### Chapter 1: Digital Twins in Physical AI
- Previous: None (first chapter)
- Next: Chapter 2 - Physics Simulation with Gazebo

#### Chapter 2: Physics Simulation with Gazebo
- Previous: Chapter 1 - Digital Twins in Physical AI
- Next: Chapter 3 - High-Fidelity Environments with Unity
- References Chapter 1 concepts for foundational knowledge

#### Chapter 3: High-Fidelity Environments with Unity
- Previous: Chapter 2 - Physics Simulation with Gazebo
- Next: Chapter 4 - Sensor Simulation for Humanoid Robots
- References Chapter 1 concepts for foundational knowledge

#### Chapter 4: Sensor Simulation for Humanoid Robots
- Previous: Chapter 3 - High-Fidelity Environments with Unity
- Next: Module conclusion or next module
- References Chapter 1 concepts and Chapter 2 physics simulation

## Cross-Chapter References

### Dependencies to Consider
- Chapter 2 builds on concepts from Chapter 1 (digital twin foundations)
- Chapter 4 connects to Chapter 2 (physics simulation affects sensor behavior)
- All chapters reference foundational concepts from Chapter 1

## Implementation Notes

When creating the actual chapter files, include navigation elements using Docusaurus' built-in features:

```markdown
import DocCardList from '@theme/DocCardList';
import {useCurrentSidebarCategory} from '@docusaurus/theme-common';

## Next Steps

<DocCardList items={useCurrentSidebarCategory().items} />
```

Or use the pagination feature if available in the Docusaurus configuration.