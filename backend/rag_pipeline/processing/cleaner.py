import re
from typing import List, Union
from backend.rag_pipeline.utils.helpers import clean_text


class TextCleaner:
    """
    Class to clean and normalize text content for the RAG pipeline
    """
    def __init__(self):
        """
        Initialize the text cleaner
        """
        # Common patterns to remove or replace
        self.patterns_to_remove = [
            r'<script[^>]*>.*?</script>',  # JavaScript
            r'<style[^>]*>.*?</style>',    # CSS
            r'<[^>]+>',                    # HTML tags
            r'&[a-zA-Z]+;',               # HTML entities
        ]

        # Patterns to normalize
        self.patterns_to_normalize = [
            (r'\s+', ' '),                # Multiple whitespace to single space
            (r'\n\s*\n', '\n\n'),        # Multiple newlines to double newline
        ]

    def clean_text(self, text: str) -> str:
        """
        Clean text by removing extraneous whitespace and normalizing

        Args:
            text: The text to clean

        Returns:
            Cleaned text
        """
        if not text:
            return ""

        # Remove HTML patterns
        for pattern in self.patterns_to_remove:
            text = re.sub(pattern, '', text, flags=re.DOTALL | re.IGNORECASE)

        # Normalize whitespace patterns
        for pattern, replacement in self.patterns_to_normalize:
            text = re.sub(pattern, replacement, text)

        # Use the utility function for additional cleaning
        text = clean_text(text)

        return text

    def normalize_content(self, content: Union[str, List[str]]) -> Union[str, List[str]]:
        """
        Normalize text content (or list of content) for standard text processing

        Args:
            content: Text content or list of text content to normalize

        Returns:
            Normalized content (same type as input)
        """
        if isinstance(content, list):
            return [self.clean_text(item) for item in content]
        else:
            return self.clean_text(content)

    def remove_special_characters(self, text: str, keep_chars: str = ".,!?;:'\"()[]{}") -> str:
        """
        Remove special characters from text, keeping only alphanumeric and specified characters

        Args:
            text: The text to process
            keep_chars: Characters to keep in the text

        Returns:
            Text with special characters removed
        """
        if not text:
            return ""

        # Create a pattern that matches all characters except alphanumeric and keep_chars
        pattern = f'[^a-zA-Z0-9{re.escape(keep_chars)}\s]'
        cleaned_text = re.sub(pattern, ' ', text)

        # Clean up multiple spaces that might have been created
        cleaned_text = re.sub(r'\s+', ' ', cleaned_text).strip()

        return cleaned_text

    def remove_extra_whitespace(self, text: str) -> str:
        """
        Remove extra whitespace while preserving paragraph structure

        Args:
            text: The text to process

        Returns:
            Text with extra whitespace removed
        """
        if not text:
            return ""

        # Replace multiple spaces with single space
        text = re.sub(r'[ \t]+', ' ', text)

        # Replace multiple newlines with maximum 2 newlines (preserve paragraphs)
        text = re.sub(r'\n{3,}', '\n\n', text)

        # Strip leading/trailing whitespace
        text = text.strip()

        return text

    def standardize_line_endings(self, text: str) -> str:
        """
        Standardize line endings to use \n

        Args:
            text: The text to process

        Returns:
            Text with standardized line endings
        """
        if not text:
            return ""

        # Replace all types of line endings with \n
        text = re.sub(r'\r\n|\r|\n', '\n', text)

        return text

    def clean_and_normalize(self, text: str) -> str:
        """
        Perform complete cleaning and normalization of text

        Args:
            text: The text to process

        Returns:
            Fully cleaned and normalized text
        """
        if not text:
            return ""

        # Apply all cleaning steps in sequence
        text = self.standardize_line_endings(text)
        text = self.remove_special_characters(text)
        text = self.remove_extra_whitespace(text)
        text = self.clean_text(text)

        return text