# Data Model: Homepage Update for Physical AI & Humanoid Robotics Course

## Overview

This document defines the data structures and entities for the homepage update feature. Since this is primarily a UI/UX enhancement with no complex data storage requirements, the data model focuses on the content structures and component interfaces.

## Component Interfaces

### ModuleCardProps
**Purpose**: Defines the interface for module cards displayed on the homepage

**Fields**:
- `title` (string): The title of the course module (e.g., "ROS 2", "Digital Twin", "AI-Robot Brain", "Vision-Language-Action")
- `subchapters` (string[]): Array of subchapter titles for the module
- `link` (string): URL path to the corresponding documentation page

**Validation**:
- `title` must be non-empty
- `subchapters` must contain 1-8 items (for visual balance)
- `link` must be a valid relative URL starting with `/docs/`

**Example**:
```typescript
{
  title: "ROS 2",
  subchapters: [
    "Introduction to ROS 2",
    "Nodes & Topics",
    "Services & Actions",
    "URDF Robot Description"
  ],
  link: "/docs/chapters/intro-ros2"
}
```

### HardwareCardProps
**Purpose**: Defines the interface for hardware cards displayed on the homepage

**Fields**:
- `title` (string): The title of the hardware component (e.g., "Digital Twin Workstation", "Physical AI Edge Kit", "Robot Lab")
- `description` (string): Detailed description of the hardware component
- `price` (string): Approximate price range for the hardware component

**Validation**:
- `title` must be non-empty
- `description` must be 10-200 characters
- `price` must follow format "~$X-X" or "~$X" or "Variable"

**Example**:
```typescript
{
  title: "Digital Twin Workstation",
  description: "High-performance computing setup for running digital twin simulations and development environments.",
  price: "~$2,000-$5,000"
}
```

## Content Entities

### Module
**Purpose**: Represents a course module with its associated content

**Fields**:
- `id` (string): Unique identifier for the module
- `title` (string): Display title of the module
- `subchapters` (string[]): List of subchapter titles within the module
- `documentationPath` (string): Path to the module's documentation
- `icon` (string): Icon identifier for the module (to be implemented in UI)

**Relationships**:
- Contains multiple Subchapter entities

### HardwareItem
**Purpose**: Represents a hardware component required for the course

**Fields**:
- `id` (string): Unique identifier for the hardware item
- `title` (string): Display title of the hardware component
- `description` (string): Detailed description of the component
- `priceRange` (string): Approximate price information
- `category` (string): Category of the hardware (e.g., "Development", "Robotics", "Lab")

**Relationships**:
- Belongs to the Hardware Requirements section

## Page Structure

### HomepageLayout
**Purpose**: Defines the overall structure of the homepage

**Sections**:
1. **Hero Section**
   - title: string ("Physical AI & Humanoid Robotics")
   - tagline: string ("Bridging Digital Brain to Physical Body")
   - ctaButton: {text: string, link: string}

2. **Modules Section**
   - title: string ("Course Modules")
   - modules: Module[]

3. **Hardware Section**
   - title: string ("Hardware Requirements")
   - hardwareItems: HardwareItem[]

## State Transitions

### Card Interaction States
- **Default**: Card appears in normal state
- **Hover**: Card shows enhanced visual effects (elevation, color change)
- **Active**: Card shows when clicked (navigation occurs)

## Validation Rules

### UI Content Validation
- All links must be valid relative paths to documentation
- Module cards must display exactly 4 subchapters as specified in requirements
- Hardware cards must display both description and price information
- All content must be properly formatted for accessibility

### Responsive Behavior
- On mobile: Cards stack vertically
- On tablet: Cards display in 2-column grid
- On desktop: Cards display in 3-column grid (Hardware) or 4-column grid (Modules)