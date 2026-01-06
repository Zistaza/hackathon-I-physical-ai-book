from bs4 import BeautifulSoup
from typing import List, Optional, Dict, Any
import re
from urllib.parse import urlparse
from ...models.data import ContentChunk


class DocusaurusParser:
    """
    Parser for Docusaurus websites that extracts clean content from HTML
    """
    def __init__(self):
        """
        Initialize the parser
        """
        # CSS selectors for Docusaurus main content areas
        self.main_content_selectors = [
            'main article',  # Primary content area
            'article',       # Alternative content area
            '.main-wrapper', # Common Docusaurus wrapper
            '.container',    # Container that might hold content
            '.markdown',     # Markdown content areas
            '.theme-doc-markdown',  # Docusaurus specific class
            '.doc-content',  # Documentation content area
            '.docs-content'  # Alternative documentation class
        ]

        # CSS selectors for elements to exclude (navigation, headers, footers, etc.)
        self.exclude_selectors = [
            'nav',           # Navigation elements
            'header',        # Header sections
            'footer',        # Footer sections
            '.navbar',       # Navigation bars
            '.menu',         # Menu items
            '.sidebar',      # Side navigation
            '.toc',          # Table of contents
            '.pagination',   # Pagination controls
            '.footer',       # Footer elements
            '.header',       # Header elements
            '.search',       # Search components
            '.admonition',   # Note/warning boxes that may not be content
        ]

    def extract_content(self, html: str, source_url: str = "") -> str:
        """
        Extract main content from HTML, removing navigation and UI chrome

        Args:
            html: HTML content to parse
            source_url: Source URL for context (used for metadata extraction)

        Returns:
            Clean text content extracted from the HTML
        """
        soup = BeautifulSoup(html, 'html.parser')

        # Remove elements that should be excluded
        for selector in self.exclude_selectors:
            for element in soup.select(selector):
                element.decompose()  # Remove the element and its children

        # Try to find main content using selectors
        main_content = None
        for selector in self.main_content_selectors:
            main_content = soup.select_one(selector)
            if main_content:
                break

        # If no main content found, use the body or entire document
        if not main_content:
            main_content = soup.find('body') or soup

        # Extract text content, preserving paragraph structure
        content_parts = []
        for element in main_content.find_all(['p', 'h1', 'h2', 'h3', 'h4', 'h5', 'h6', 'li', 'td', 'div']):
            text = element.get_text(strip=True)
            if text:
                content_parts.append(text)

        # Join content with newlines to preserve structure
        clean_content = '\n\n'.join(content_parts)
        return clean_content

    def extract_metadata(self, html: str, source_url: str = "") -> Dict[str, Any]:
        """
        Extract metadata from HTML such as title, module, chapter, etc.

        Args:
            html: HTML content to parse
            source_url: Source URL for metadata extraction

        Returns:
            Dictionary containing extracted metadata
        """
        soup = BeautifulSoup(html, 'html.parser')
        metadata = {}

        # Extract title from various possible locations
        title_tag = soup.find('title')
        if title_tag:
            metadata['title'] = title_tag.get_text(strip=True)
        else:
            # Try h1 as title
            h1_tag = soup.find('h1')
            if h1_tag:
                metadata['title'] = h1_tag.get_text(strip=True)

        # Extract description from meta tags
        description_tag = soup.find('meta', attrs={'name': 'description'})
        if description_tag:
            metadata['description'] = description_tag.get('content', '')

        # Extract keywords from meta tags
        keywords_tag = soup.find('meta', attrs={'name': 'keywords'})
        if keywords_tag:
            metadata['keywords'] = keywords_tag.get('content', '')

        # Extract module/chapter information from URL
        if source_url:
            parsed_url = urlparse(source_url)
            path_parts = [part for part in parsed_url.path.split('/') if part]
            if path_parts:
                # Use the first path segment as module, if available
                metadata['module'] = path_parts[0] if len(path_parts) > 0 else ""
                # Use the last path segment as document name
                metadata['document'] = path_parts[-1] if path_parts else ""

        # Additional metadata extraction can be added here
        return metadata

    def parse_page(self, html: str, source_url: str = "") -> ContentChunk:
        """
        Parse a complete page and return a ContentChunk with extracted content and metadata

        Args:
            html: HTML content to parse
            source_url: Source URL for the content

        Returns:
            ContentChunk with extracted content and metadata
        """
        # Extract the clean content
        content = self.extract_content(html, source_url)

        # Extract metadata
        metadata = self.extract_metadata(html, source_url)

        # Create a ContentChunk instance
        chunk = ContentChunk(
            content=content,
            source_url=source_url,
            module=metadata.get('module'),
            chunk_index=0,  # For a single page, this would be 0
            id="",  # Will be auto-generated
            content_hash=""  # Will be auto-generated
        )

        return chunk

    def parse_multiple_pages(self, html_contents: List[tuple[str, str]]) -> List[ContentChunk]:
        """
        Parse multiple pages and return ContentChunks

        Args:
            html_contents: List of tuples (url, html_content) to parse

        Returns:
            List of ContentChunk objects
        """
        chunks = []
        for url, html in html_contents:
            chunk = self.parse_page(html, url)
            chunks.append(chunk)

        return chunks