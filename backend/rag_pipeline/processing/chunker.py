from typing import List, Tuple
from backend.rag_pipeline.models.data import ContentChunk
from backend.rag_pipeline.utils.helpers import generate_chunk_id
import re


class ContentChunker:
    """
    Class to split text content into deterministic chunks with reproducible boundaries
    """
    def __init__(self, max_chunk_size: int = 1000, overlap: int = 100):
        """
        Initialize the content chunker

        Args:
            max_chunk_size: Maximum number of characters per chunk
            overlap: Number of characters to overlap between chunks
        """
        if max_chunk_size <= 0:
            raise ValueError("max_chunk_size must be positive")
        if overlap < 0:
            raise ValueError("overlap must be non-negative")
        if overlap >= max_chunk_size:
            raise ValueError("overlap must be less than max_chunk_size")

        self.max_chunk_size = max_chunk_size
        self.overlap = overlap

    def chunk_content(self, content: str, source_url: str = "", module: str = "") -> List[ContentChunk]:
        """
        Split text content into deterministic chunks with reproducible boundaries

        Args:
            content: The content to chunk
            source_url: Source URL for the content (used for ID generation)
            module: Module identifier for the content

        Returns:
            List of ContentChunk objects
        """
        if not content:
            return []

        # Split content into chunks based on semantic boundaries
        chunks = self._semantic_chunking(content)

        # Ensure chunks are within size limits
        final_chunks = []
        chunk_index = 0

        for chunk_text in chunks:
            # If a chunk is still too large, split it by character count
            sub_chunks = self._split_large_chunk(chunk_text)

            for sub_chunk_text in sub_chunks:
                # Create ContentChunk with proper metadata
                chunk = ContentChunk(
                    content=sub_chunk_text,
                    source_url=source_url,
                    module=module,
                    chunk_index=chunk_index,
                    id="",  # Will be auto-generated
                    content_hash=""  # Will be auto-generated
                )
                final_chunks.append(chunk)
                chunk_index += 1

        return final_chunks

    def _semantic_chunking(self, content: str) -> List[str]:
        """
        Split content based on semantic boundaries like paragraphs, headers, etc.

        Args:
            content: The content to split

        Returns:
            List of content chunks
        """
        if not content:
            return []

        # Split by double newlines (paragraph boundaries)
        paragraphs = content.split('\n\n')

        # Process paragraphs into chunks that respect the size limit
        chunks = []
        current_chunk = ""

        for paragraph in paragraphs:
            # If adding this paragraph would exceed the limit
            if len(current_chunk) + len(paragraph) > self.max_chunk_size:
                # If current chunk is not empty, save it
                if current_chunk.strip():
                    chunks.append(current_chunk.strip())

                # If the paragraph itself is too large, split it
                if len(paragraph) > self.max_chunk_size:
                    sub_chunks = self._split_large_paragraph(paragraph)
                    chunks.extend(sub_chunks)
                    current_chunk = ""
                else:
                    # Start a new chunk with this paragraph
                    current_chunk = paragraph
            else:
                # Add paragraph to current chunk
                if current_chunk:
                    current_chunk += "\n\n" + paragraph
                else:
                    current_chunk = paragraph

        # Add the last chunk if it exists
        if current_chunk.strip():
            chunks.append(current_chunk.strip())

        return chunks

    def _split_large_paragraph(self, paragraph: str) -> List[str]:
        """
        Split a large paragraph into smaller chunks based on sentences.

        Args:
            paragraph: The large paragraph to split

        Returns:
            List of smaller chunks
        """
        # Split by sentence endings
        sentences = re.split(r'(?<=[.!?])\s+', paragraph)

        chunks = []
        current_chunk = ""

        for sentence in sentences:
            # If adding this sentence would exceed the limit
            if len(current_chunk) + len(sentence) > self.max_chunk_size:
                # If current chunk is not empty, save it
                if current_chunk.strip():
                    chunks.append(current_chunk.strip())

                # If the sentence itself is too large, split it by character count
                if len(sentence) > self.max_chunk_size:
                    char_chunks = self._split_large_chunk(sentence)
                    chunks.extend(char_chunks)
                    current_chunk = ""
                else:
                    # Start a new chunk with this sentence
                    current_chunk = sentence
            else:
                # Add sentence to current chunk
                if current_chunk:
                    current_chunk += " " + sentence
                else:
                    current_chunk = sentence

        # Add the last chunk if it exists
        if current_chunk.strip():
            chunks.append(current_chunk.strip())

        return chunks

    def _split_large_chunk(self, chunk: str) -> List[str]:
        """
        Split a chunk that is still too large into smaller pieces.

        Args:
            chunk: The chunk to split

        Returns:
            List of smaller chunks
        """
        if len(chunk) <= self.max_chunk_size:
            return [chunk]

        chunks = []
        start = 0

        while start < len(chunk):
            end = start + self.max_chunk_size

            # If we're not at the end, try to break at a word boundary
            if end < len(chunk):
                # Look for the last space within the chunk to avoid breaking words
                space_pos = chunk.rfind(' ', start, end)

                if space_pos != -1 and space_pos > start:
                    # Break at the space
                    end = space_pos

            chunks.append(chunk[start:end])
            start = end - self.overlap if self.overlap > 0 else end

        return chunks

    def generate_chunk_id(self, source_url: str, content: str, chunk_index: int) -> str:
        """
        Generate a unique ID for a chunk based on source URL, content, and chunk index

        Args:
            source_url: The source URL of the content
            content: The content text
            chunk_index: The index of this chunk within the document

        Returns:
            Unique identifier for the chunk
        """
        return generate_chunk_id(source_url, content, chunk_index)

    def chunk_with_metadata(self, content: str, source_url: str = "", module: str = "",
                          chunk_size: int = None, overlap: int = None) -> List[ContentChunk]:
        """
        Chunk content with the ability to override chunk size and overlap

        Args:
            content: The content to chunk
            source_url: Source URL for the content
            module: Module identifier for the content
            chunk_size: Optional override for max chunk size
            overlap: Optional override for overlap

        Returns:
            List of ContentChunk objects
        """
        # Save original values
        original_size = self.max_chunk_size
        original_overlap = self.overlap

        # Apply overrides if provided
        if chunk_size is not None:
            self.max_chunk_size = chunk_size
        if overlap is not None:
            self.overlap = overlap

        # Generate chunks
        chunks = self.chunk_content(content, source_url, module)

        # Restore original values
        self.max_chunk_size = original_size
        self.overlap = original_overlap

        return chunks