# Docusaurus Markdown Standards for Digital Twin Module

## Document Structure

### Frontmatter
Every document must include proper frontmatter with the following required fields:
```yaml
---
title: [Document title]
description: [Brief description of the content]
sidebar_position: [Number for ordering in sidebar]
---
```

### Headings
- Use proper heading hierarchy: H1 for title (automatically generated from frontmatter), H2 for main sections, H3 for subsections, H4 for further subdivisions
- Use sentence case for headings (capitalize only the first word and proper nouns)
- Use descriptive headings that clearly indicate the content

### Content Formatting

#### Text
- Use standard Markdown syntax for formatting
- Use **bold** for emphasis on important terms
- Use *italics* for book titles, emphasis, or to highlight new terms
- Use `inline code` for technical terms, file names, and code snippets

#### Code Blocks
- Use proper language identifiers for syntax highlighting
- Example: ```markdown
  ```python
  def example_function():
      return "Hello, World!"
  ```
  ```

#### Lists
- Use hyphens (-) for unordered lists
- Use numbers (1., 2., 3.) for ordered lists
- Maintain consistent indentation (2 spaces)

#### Links
- Use Markdown link syntax: `[link text](path/to/file)`
- For internal links, use relative paths from the docs directory
- For external links, use full URLs

#### Images
- Use Markdown image syntax: `![alt text](path/to/image)`
- Store images in appropriate directories under `static/`
- Always include descriptive alt text

## Technical Content Standards

### Mathematics
- Use LaTeX-style math rendering where needed: `$inline math$` or `$$display math$$`
- Use clear, consistent notation throughout
- Explain mathematical concepts in plain language alongside formulas

### Code Examples
- Include meaningful, practical examples
- Use realistic variable names that reflect the context
- Include comments to explain complex code sections
- Ensure all code examples are valid and executable in context

### Diagrams and Visuals
- Use Docusaurus admonitions for notes, tips, and warnings:
  ```markdown
  :::note
  This is a note.
  :::
  ```
- Use appropriate admonition types: `note`, `tip`, `caution`, `danger`

## Consistency Guidelines

### Terminology
- Use consistent terminology as defined in the robotics-terminology.md document
- Define new terms when first used in each document
- Link to related concepts when appropriate

### Cross-References
- Link between related sections within the module
- Use relative paths for internal links
- Include context when making cross-references

## Quality Standards

### Readability
- Use clear, concise language appropriate for advanced learners
- Break complex concepts into digestible sections
- Use examples to illustrate abstract concepts
- Include practical applications where relevant

### Accessibility
- Write alt text for all images
- Use descriptive link text
- Maintain good heading structure for screen readers
- Use sufficient contrast for text and backgrounds

## Review Checklist
- [ ] Frontmatter is properly formatted with all required fields
- [ ] Heading hierarchy is logical and consistent
- [ ] All code blocks have appropriate language identifiers
- [ ] Links work correctly and point to valid destinations
- [ ] Images have descriptive alt text
- [ ] Terminology is consistent with the reference document
- [ ] Content is appropriate for the target audience level
- [ ] Mathematical notation is clear and properly formatted
- [ ] Examples are practical and meaningful