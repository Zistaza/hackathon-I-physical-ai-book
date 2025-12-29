# Quickstart Guide: Homepage Update for Physical AI & Humanoid Robotics Course

## Overview

This guide provides a quick start for implementing the homepage update for the Physical AI & Humanoid Robotics course. The update includes a modern Hero section, Module cards with subchapters, and Hardware requirement cards.

## Prerequisites

- Node.js 18+ installed
- Docusaurus CLI installed (`npm install -g @docusaurus/cli`)
- Project dependencies installed (`npm install` in the `my-website` directory)

## Setup and Installation

1. **Navigate to the website directory**:
   ```bash
   cd my-website
   ```

2. **Install dependencies** (if not already installed):
   ```bash
   npm install
   ```

3. **Start the development server**:
   ```bash
   npm run start
   ```

## Implementation Steps

### 1. Update the Homepage Component

Replace the content of `src/pages/index.tsx` with the updated implementation that includes:

- Enhanced Hero section with gradient background
- 4 Module cards with hover effects and subchapter lists
- 3 Hardware cards with descriptions and pricing

### 2. Update the CSS Module

Enhance `src/pages/index.module.css` with:

- Gradient background styles for the Hero section
- Card styling with shadows, rounded corners, and hover effects
- Responsive grid layouts for different screen sizes
- CSS transitions for interactive elements

### 3. Key Features to Implement

#### Hero Section
- Title: "Physical AI & Humanoid Robotics"
- Tagline: "Bridging Digital Brain to Physical Body"
- Gradient background with modern typography
- "Start Learning" button linking to `/docs/chapters/intro-ros2`
- Subtle animation effects on hover

#### Module Cards
- 4 cards: ROS 2, Digital Twin, AI-Robot Brain, Vision-Language-Action
- Each card shows 4 subchapter titles
- Cards link to correct documentation pages
- Colored backgrounds with shadows and rounded corners
- Hover effects with elevation and color changes
- Grid layout (4 columns on desktop)

#### Hardware Cards
- 3 cards: Digital Twin Workstation, Physical AI Edge Kit, Robot Lab
- Each card shows description and approximate price
- Colored backgrounds with shadows
- Hover effects with elevation
- Grid layout (3 columns on desktop, stacked on mobile)

## Running and Testing

1. **Start the development server**:
   ```bash
   cd my-website
   npm run start
   ```

2. **Open your browser** to `http://localhost:3000`

3. **Test responsiveness** by resizing the browser window or using browser developer tools

4. **Verify all links** work correctly by clicking on module and hardware cards

## Common Customizations

### Color Palette
Update the CSS variables in `index.module.css` to change the color scheme:

```css
:root {
  --primary-color: #your-color;
  --secondary-color: #your-color;
  --card-shadow: 0 4px 6px rgba(0, 0, 0, 0.1);
}
```

### Card Hover Effects
Modify the transition properties in the card CSS:

```css
.card:hover {
  transform: translateY(-5px);
  box-shadow: 0 10px 20px rgba(0, 0, 0, 0.2);
  transition: all 0.3s ease;
}
```

## Troubleshooting

### Cards Not Displaying Properly
- Check that the grid classes are correctly applied (`col--3` for modules, `col--4` for hardware)
- Verify that the container divs have the proper Docusaurus classes

### Links Not Working
- Ensure all link paths start with `/docs/` and match existing documentation files
- Check for typos in the path strings

### Responsive Issues
- Verify that mobile-first CSS classes are properly implemented
- Test on multiple screen sizes using browser developer tools

## Deployment

When ready to deploy:

1. Build the site:
   ```bash
   npm run build
   ```

2. The built site will be in the `build/` directory
3. Deploy according to your hosting platform's instructions (typically GitHub Pages for this project)