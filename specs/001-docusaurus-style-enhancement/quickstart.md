# Quickstart Guide: Docusaurus Style Enhancement

## Overview
This guide provides instructions for implementing the visual style enhancements to your Docusaurus-based textbook. The enhancements include improved typography, color schemes, code block styling, and responsive design elements.

## Prerequisites
- Docusaurus 3.x project
- Node.js and npm installed
- Basic understanding of CSS

## Installation Steps

### 1. Create the CSS File
Create a new CSS file at `my-website/src/css/custom.css` with the following content:

```css
/**
 * Docusaurus Style Enhancement
 *
 * Global CSS file for enhanced typography, colors, and layout
 * Import this file in your Docusaurus configuration to apply styles
 */

/* Typography improvements */
:root {
  --font-sans: 'Inter', 'Roboto', system-ui, -apple-system, sans-serif;
}

/* Apply clean sans-serif font globally */
body {
  font-family: var(--font-sans);
  line-height: 1.7;
}

/* Heading hierarchy */
h1, h2 {
  color: #2b3137;
  font-weight: 600;
}

h3 {
  color: #2b3137;
}

/* Paragraph styling */
p {
  color: #4a4a4a;
  line-height: 1.7;
}

/* Code block styling */
code {
  background-color: #f5f5f5;
  padding: 0.2rem 0.4rem;
  border-radius: 4px;
  font-family: monospace;
}

pre code {
  background: none;
  padding: 0;
  border-radius: 0;
}

pre {
  background-color: #f5f5f5;
  padding: 1rem;
  border-radius: 6px;
  overflow-x: auto;
}

/* List styling */
ul, ol {
  margin: 1rem 0;
}

ul li, ol li {
  color: #4a4a4a;
  margin: 0.5rem 0;
}

ul li::marker, ol li::marker {
  color: #666;
}

/* Note and tip block styling */
.note, .tip {
  border-left: 4px solid #667eea;
  background-color: #f0f4ff;
  border-radius: 4px;
  padding: 1rem;
  margin: 1rem 0;
}

/* Spacing improvements */
article {
  padding: 0 1rem;
}

section {
  margin: 1.5rem 0;
}

/* Responsive design */
@media (max-width: 768px) {
  h1 {
    font-size: 1.8rem;
  }

  h2 {
    font-size: 1.5rem;
  }

  h3 {
    font-size: 1.2rem;
  }

  p {
    font-size: 0.95rem;
  }

  pre {
    padding: 0.8rem;
    font-size: 0.85rem;
  }
}

@media (max-width: 480px) {
  h1 {
    font-size: 1.6rem;
  }

  h2 {
    font-size: 1.3rem;
  }

  p {
    font-size: 0.9rem;
  }
}
```

### 2. Update Docusaurus Configuration
Add the CSS file to your Docusaurus configuration in `docusaurus.config.js`:

```javascript
module.exports = {
  // ... other config
  stylesheets: [
    {
      href: '/css/custom.css',
      type: 'text/css',
    },
  ],
  // ... rest of config
};
```

### 3. Alternative Configuration Method
If the above doesn't work, you can also import the CSS file in your main layout by creating or updating `src/pages/index.js` or a layout component:

```javascript
import '../css/custom.css';

// Your component code here
```

## Usage

### Special Blocks
To use the special note and tip styling, add the appropriate classes to your markdown:

```markdown
<div class="note">

This is a note block with special styling.

</div>

<div class="tip">

This is a tip block with special styling.

</div>
```

## Testing
1. Run your Docusaurus site locally: `npm run start`
2. Navigate through different pages to verify:
   - Typography improvements
   - Color scheme application
   - Code block styling
   - List styling
   - Special block styling
   - Responsive behavior on different screen sizes

## Troubleshooting
- If styles don't appear, verify the CSS file path in your configuration
- If mobile styles don't work, check that your viewport meta tag is properly set
- For specific element overrides, you may need to increase CSS specificity