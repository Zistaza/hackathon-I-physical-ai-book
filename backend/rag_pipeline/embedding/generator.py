import cohere
from typing import List, Union
from ...models.data import EmbeddingVector, ContentChunk
from ...config.settings import load_config


class CohereEmbedder:
    """
    Class to generate embeddings using Cohere API
    """
    def __init__(self, api_key: str = None, model_name: str = "embed-multilingual-v2.0"):
        """
        Initialize the embedder

        Args:
            api_key: Cohere API key (if not provided, will load from config)
            model_name: Name of the Cohere model to use for embeddings
        """
        if api_key is None:
            config = load_config()
            api_key = config.cohere_api_key

        if not api_key:
            raise ValueError("Cohere API key is required")

        self.client = cohere.Client(api_key)
        self.model_name = model_name

    def generate_embedding(self, text: str) -> EmbeddingVector:
        """
        Generate embedding vector using Cohere API

        Args:
            text: Text to generate embedding for

        Returns:
            EmbeddingVector with the generated embedding
        """
        if not text.strip():
            raise ValueError("Text cannot be empty")

        # Generate embedding using Cohere
        response = self.client.embed(
            texts=[text],
            model=self.model_name,
            input_type="search_document"  # Using search_document as input type for document embeddings
        )

        # Extract the embedding vector
        embedding_vector = response.embeddings[0]

        # Create and return EmbeddingVector object
        return EmbeddingVector(
            chunk_id="",  # Will be set when associated with a ContentChunk
            vector=embedding_vector,
            model_name=self.model_name,
            model_version=None,  # Cohere doesn't typically provide version in response
            created_at=None  # Will be set by model validation
        )

    def batch_generate_embeddings(self, texts: List[str]) -> List[EmbeddingVector]:
        """
        Generate multiple embeddings in a batch

        Args:
            texts: List of texts to generate embeddings for

        Returns:
            List of EmbeddingVector objects
        """
        if not texts:
            return []

        # Filter out empty texts
        non_empty_texts = [text for text in texts if text.strip()]
        if not non_empty_texts:
            return []

        # Generate embeddings in batch
        response = self.client.embed(
            texts=non_empty_texts,
            model=self.model_name,
            input_type="search_document"
        )

        # Create EmbeddingVector objects for each embedding
        embeddings = []
        for i, embedding_vector in enumerate(response.embeddings):
            embedding = EmbeddingVector(
                chunk_id="",  # Will be set when associated with a ContentChunk
                vector=embedding_vector,
                model_name=self.model_name,
                model_version=None,
                created_at=None  # Will be set by model validation
            )
            embeddings.append(embedding)

        return embeddings

    def embed_content_chunks(self, content_chunks: List[ContentChunk]) -> List[EmbeddingVector]:
        """
        Generate embeddings for a list of ContentChunk objects

        Args:
            content_chunks: List of ContentChunk objects to embed

        Returns:
            List of EmbeddingVector objects with proper chunk_id references
        """
        if not content_chunks:
            return []

        # Extract texts from content chunks
        texts = [chunk.content for chunk in content_chunks]

        # Generate embeddings
        embeddings = self.batch_generate_embeddings(texts)

        # Associate each embedding with the corresponding chunk ID
        for i, embedding in enumerate(embeddings):
            embedding.chunk_id = content_chunks[i].id

        return embeddings

    def embed_single_chunk(self, content_chunk: ContentChunk) -> EmbeddingVector:
        """
        Generate embedding for a single ContentChunk object

        Args:
            content_chunk: ContentChunk object to embed

        Returns:
            EmbeddingVector object with proper chunk_id reference
        """
        embedding = self.generate_embedding(content_chunk.content)
        embedding.chunk_id = content_chunk.id
        return embedding