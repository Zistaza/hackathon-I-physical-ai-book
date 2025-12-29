---
id: 001
title: Homepage Update for Physical AI Course
stage: spec
date: 2025-12-29
surface: agent
model: Claude Sonnet 4.5
feature: 001-homepage-update
branch: 001-homepage-update
user: emizee
command: sp.specify
labels: ["homepage", "ui", "ux", "docusaurus", "course"]
links:
  spec: specs/001-homepage-update/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/001-homepage-update/spec.md
 - specs/001-homepage-update/checklists/requirements.md
tests:
 -
---

## Prompt

Task: Update the homepage at `my-website/src/pages/index.tsx` to create a modern, visually appealing, fully styled landing page for the "Physical AI & Humanoid Robotics" course, and generate a Prompt History Record (PHR) for this update.

Requirements:

1. Hero Section:
- Title: "Physical AI & Humanoid Robotics"
- Tagline: "Bridging Digital Brain to Physical Body"
- "Start Learning" button linking to `/docs/chapters/intro-ros2`
- Gradient background, modern typography, subtle animation
- Optional hero image or icon
- Fully responsive

2. Modules Section:
- 4 module cards: ROS 2, Digital Twin, AI-Robot Brain, Vision-Language-Action
- Each card shows 4 subchapter titles
- Cards link to correct docs pages
- Colored cards with shadows, rounded corners, hover effects
- Include icons for each module
- Grid layout (col--3), responsive, consistent spacing

3. Hardware Section:
- 3 cards: Digital Twin Workstation, Physical AI Edge Kit, Robot Lab
- Each card shows description and approximate price
- Colored cards with shadows, hover effects, icons
- Use Card/CardGroup, responsive layout (col--4 desktop, stacked mobile)

4. Design & Theme:
- Enhance default Docusaurus theme
- Gradient Hero, colorful cards, shadows, hover effects, icons
- Modern readable typography
- Fully responsive and visually appealing

5. Technical Safety:
- Do not modify any markdown files in `/docs`
- Preserve existing content
- Output a complete TSX file ready to replace `src/pages/index.tsx`
- Maintain correct doc links

6. Bonus:
- Add subtle hover/transition animations
- Harmonious color palette, clean maintainable code

Output:
- Complete TSX code for homepage
- Prompt History Record (PHR) documenting the update

## Response snapshot

Created a comprehensive specification for the homepage update feature for the Physical AI & Humanoid Robotics course. The specification includes:

- User scenarios with prioritized user stories for viewing the modern homepage, exploring course modules, and reviewing hardware requirements
- Detailed functional requirements (FR-001 through FR-012) covering all aspects of the homepage design and functionality
- Success criteria with measurable outcomes (SC-001 through SC-008) that are technology-agnostic and measurable
- Key entities identification
- Edge cases consideration

The specification is complete and ready for the planning phase. It follows all requirements specified by the user and adheres to the template structure with all mandatory sections filled out properly.

## Outcome

- ✅ Impact: Created complete feature specification for homepage redesign with user stories, requirements, and success criteria
- 🧪 Tests: Specification validated against quality checklist with all items passing
- 📁 Files: Created spec.md and requirements checklist in specs/001-homepage-update/
- 🔁 Next prompts: Ready for planning phase with /sp.plan
- 🧠 Reflection: Specification is comprehensive and ready for architectural planning

## Evaluation notes (flywheel)

- Failure modes observed: None - all requirements properly captured and documented
- Graders run and results (PASS/FAIL): PASS - All checklist items validated successfully
- Prompt variant (if applicable): Standard specification creation
- Next experiment (smallest change to try): Proceed to architectural planning phase