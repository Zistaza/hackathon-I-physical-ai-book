---
id: 1
title: "Homepage Update for Physical AI & Humanoid Robotics Course"
stage: general
date_iso: 2025-12-29
surface: agent
model: claude-sonnet-4-5-20250929
feature: homepage-update
branch: 001-homepage-update
user: emizee
command: "Update the existing homepage at my-website/src/pages/index.tsx to create a modern, responsive landing page for my \"Physical AI & Humanoid Robotics\" course."
labels: ["homepage", "ui", "docusaurus", "landing-page"]
spec: null
ticket: null
adr: null
pr: null
files_yaml: |
  - my-website/src/pages/index.tsx
tests_yaml: |
  - Build test: npm run build
prompt_text: |
  Update the existing homepage at my-website/src/pages/index.tsx to create a modern, responsive landing page for my "Physical AI & Humanoid Robotics" course.

  1. **Hero Section**
  - Add a hero section at the top with:
    - Course title: "Physical AI & Humanoid Robotics"
    - Tagline: "Bridging Digital Brain to Physical Body"
    - A "Start Learning" button linking to /docs/chapters/01-intro-ros2

  2. **Modules Section**
  - Add a modules section showing 4 modules as cards:
    1. ROS 2
    2. Digital Twin
    3. AI-Robot Brain
    4. Vision-Language-Action
  - Each module card should display its 4 subchapter titles.
  - Each card should link to the corresponding main doc page.
  - Use a clean grid layout and Docusaurus Card/CardGroup components.

  3. **Hardware Requirements Section**
  - Add a hardware section showing 3 cards:
    1. Digital Twin Workstation
    2. Physical AI Edge Kit
    3. Robot Lab
  - Each card should have a short description and approximate price.
  - Use Docusaurus CardGroup for layout.

  4. **Design & Responsiveness**
  - Ensure the homepage is responsive for desktop and mobile.
  - Use Docusaurus default theme styling.
  - Keep spacing, headings, and typography consistent.

  5. **Safety**
  - Do NOT modify any markdown files in /docs.
  - Preserve all existing homepage content outside of the added sections.

  6. **PHR (Project Home Report)**
  - After updating the homepage, generate a brief PHR summarizing:
    - Sections added (Hero, Modules, Hardware)
    - Number of modules and subchapters linked
    - Hardware items added with descriptions
    - Any notes on responsiveness or styling applied
  - Format the PHR as a markdown report.

  Output:
  - Complete TSX code ready to update my-website/src/pages/index.tsx
  - Markdown PHR summarizing homepage updates.
response_text: |
  Successfully updated the homepage at my-website/src/pages/index.tsx with a modern, responsive landing page for the "Physical AI & Humanoid Robotics" course. The update included:

  - Hero Section: Added course title "Physical AI & Humanoid Robotics", tagline "Bridging Digital Brain to Physical Body", and "Start Learning" button linking to /docs/chapters/intro-ros2
  - Modules Section: Created 4 module cards (ROS 2, Digital Twin, AI-Robot Brain, Vision-Language-Action) with their 4 subchapters each, using Docusaurus Card components with responsive grid layout
  - Hardware Requirements Section: Added 3 hardware cards (Digital Twin Workstation, Physical AI Edge Kit, Robot Lab) with descriptions and approximate prices
  - Responsive Design: Used Docusaurus default styling with responsive grid classes (col--3, col--4) that adapt to desktop and mobile
  - All links were verified to work correctly with the documentation structure
outcome: |
  The homepage has been successfully transformed into a modern landing page that clearly presents the course structure and hardware requirements. The build process completed without errors, confirming all links are valid.
evaluation: |
  The implementation successfully addresses all requirements with a clean, responsive design using Docusaurus components. The page is visually appealing and provides clear navigation paths for users to explore different modules of the course.
---

# Homepage Update for Physical AI & Humanoid Robotics Course

## Summary

Successfully updated the homepage at my-website/src/pages/index.tsx with a modern, responsive landing page for the "Physical AI & Humanoid Robotics" course.

## Changes Made

### 1. Hero Section
- Added course title: "Physical AI & Humanoid Robotics"
- Added tagline: "Bridging Digital Brain to Physical Body"
- Added "Start Learning" button linking to `/docs/chapters/intro-ros2`

### 2. Modules Section
- Created 4 module cards with responsive grid layout:
  1. **ROS 2**
     - Introduction to ROS 2
     - Nodes & Topics
     - Services & Actions
     - URDF Robot Description
     - Links to: `/docs/chapters/intro-ros2`

  2. **Digital Twin**
     - Digital Twins in Physical AI
     - Physics Simulation with Gazebo
     - High-Fidelity Environments with Unity
     - Sensor Simulation for Humanoid Robots
     - Links to: `/docs/modules/digital-twin/chapter-1-digital-twins-in-physical-ai`

  3. **AI-Robot Brain**
     - Introduction to Isaac Sim
     - Isaac ROS & Visual SLAM
     - Navigation 2 Path Planning
     - Integration & Deployment
     - Links to: `/docs/module3-ai-robot-brain/intro-isaac-sim`

  4. **Vision-Language-Action**
     - Introduction to VLA
     - Voice to Action
     - Cognitive Planning with LLMs
     - Capstone: Autonomous Humanoid
     - Links to: `/docs/module4-vla-physical-ai/intro-vla`

### 3. Hardware Requirements Section
- Added 3 hardware cards with descriptions and prices:
  1. **Digital Twin Workstation**
     - Description: High-performance computing setup for running digital twin simulations and development environments
     - Price: ~$2,000-$5,000

  2. **Physical AI Edge Kit**
     - Description: Complete hardware kit with sensors, actuators, and edge computing modules for physical AI applications
     - Price: ~$1,500-$3,000

  3. **Robot Lab**
     - Description: Complete laboratory setup with humanoid robots, safety equipment, and development tools
     - Price: ~$10,000-$50,000

### 4. Design & Responsiveness
- Implemented responsive design using Docusaurus grid classes (col--3 for modules, col--4 for hardware)
- Used Docusaurus Card components for consistent styling
- Maintained consistent spacing and typography throughout
- All links validated and tested to work correctly with the documentation structure

## Verification
- The build process completed successfully with `npm run build`
- All links have been verified to point to existing documentation files
- The page is responsive and adapts to different screen sizes
- No modifications were made to any markdown files in /docs as required