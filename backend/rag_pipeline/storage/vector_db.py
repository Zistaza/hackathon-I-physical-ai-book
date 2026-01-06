from qdrant_client import QdrantClient
from qdrant_client.http import models
from typing import List, Optional, Dict, Any
from ...models.data import EmbeddingVector, ContentChunk
from ...config.settings import load_config
import hashlib


class QdrantStorage:
    """
    Class to store embeddings in Qdrant Cloud with appropriate metadata
    """
    def __init__(self, url: str = None, api_key: str = None, collection_name: str = "rag_pipeline_chunks"):
        """
        Initialize the Qdrant storage

        Args:
            url: Qdrant Cloud URL (if not provided, will load from config)
            api_key: Qdrant API key (if not provided, will load from config)
            collection_name: Name of the Qdrant collection to store vectors
        """
        if url is None or api_key is None:
            config = load_config()
            if url is None:
                url = config.qdrant_url
            if api_key is None:
                api_key = config.qdrant_api_key

        if not url or not api_key:
            raise ValueError("Both Qdrant URL and API key are required")

        self.client = QdrantClient(url=url, api_key=api_key)
        self.collection_name = collection_name

        # Create collection if it doesn't exist
        self._ensure_collection_exists()

    def _ensure_collection_exists(self):
        """
        Ensure the collection exists in Qdrant, create it if it doesn't
        """
        try:
            # Try to get collection info
            self.client.get_collection(self.collection_name)
        except Exception:
            # Collection doesn't exist, create it
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=models.VectorParams(
                    size=4096,  # Cohere embeddings are typically 4096 dimensions for some models
                    distance=models.Distance.COSINE
                )
            )

    def store_embedding(self, embedding: EmbeddingVector, content_chunk: ContentChunk) -> bool:
        """
        Store a single embedding in Qdrant with appropriate metadata

        Args:
            embedding: EmbeddingVector to store
            content_chunk: Associated ContentChunk for metadata

        Returns:
            True if successful, False otherwise
        """
        try:
            # Check if this chunk already exists to maintain idempotency
            if self.check_duplicate(content_chunk.id):
                print(f"Chunk with ID {content_chunk.id} already exists in Qdrant, skipping...")
                return True  # Consider this a success since it already exists

            # Prepare the point to insert
            point = models.PointStruct(
                id=content_chunk.id,  # Use the chunk ID as the point ID
                vector=embedding.vector,
                payload={
                    "chunk_id": content_chunk.id,
                    "content": content_chunk.content,
                    "source_url": content_chunk.source_url,
                    "module": content_chunk.module,
                    "chunk_index": content_chunk.chunk_index,
                    "content_hash": content_chunk.content_hash,
                    "model_name": embedding.model_name,
                    "created_at": content_chunk.created_at.isoformat(),
                    "embedding_created_at": embedding.created_at.isoformat()
                }
            )

            # Insert the point into the collection
            self.client.upsert(
                collection_name=self.collection_name,
                points=[point]
            )

            return True
        except Exception as e:
            print(f"Error storing embedding: {e}")
            return False

    def store_batch(self, embeddings: List[EmbeddingVector], content_chunks: List[ContentChunk]) -> Dict[str, Any]:
        """
        Store multiple embeddings in Qdrant

        Args:
            embeddings: List of EmbeddingVector objects to store
            content_chunks: List of associated ContentChunk objects

        Returns:
            Dictionary with results including counts of successful and failed operations
        """
        if len(embeddings) != len(content_chunks):
            raise ValueError("Number of embeddings must match number of content chunks")

        results = {
            "successful": 0,
            "failed": 0,
            "errors": []
        }

        # Prepare points for insertion
        points = []
        for embedding, chunk in zip(embeddings, content_chunks):
            try:
                # Check if this chunk already exists to maintain idempotency
                if self.check_duplicate(chunk.id):
                    print(f"Chunk with ID {chunk.id} already exists in Qdrant, skipping...")
                    results["successful"] += 1
                    continue

                point = models.PointStruct(
                    id=chunk.id,  # Use the chunk ID as the point ID
                    vector=embedding.vector,
                    payload={
                        "chunk_id": chunk.id,
                        "content": chunk.content,
                        "source_url": chunk.source_url,
                        "module": chunk.module,
                        "chunk_index": chunk.chunk_index,
                        "content_hash": chunk.content_hash,
                        "model_name": embedding.model_name,
                        "created_at": chunk.created_at.isoformat(),
                        "embedding_created_at": embedding.created_at.isoformat()
                    }
                )
                points.append(point)
            except Exception as e:
                results["failed"] += 1
                results["errors"].append(f"Error preparing point for chunk {chunk.id}: {e}")

        # Insert all points in a batch if any are ready
        if points:
            try:
                self.client.upsert(
                    collection_name=self.collection_name,
                    points=points
                )
                results["successful"] += len(points)
            except Exception as e:
                results["failed"] += len(points)
                results["errors"].append(f"Batch insert failed: {e}")

        return results

    def check_duplicate(self, chunk_id: str) -> bool:
        """
        Check if a chunk already exists in storage

        Args:
            chunk_id: ID of the chunk to check

        Returns:
            True if chunk exists, False otherwise
        """
        try:
            # Try to retrieve the point by ID
            records = self.client.retrieve(
                collection_name=self.collection_name,
                ids=[chunk_id]
            )
            return len(records) > 0
        except Exception:
            # If there's an error (e.g., collection doesn't exist), assume it doesn't exist
            return False

    def search_similar(self, query_embedding: List[float], limit: int = 5) -> List[Dict[str, Any]]:
        """
        Search for similar content in the vector database

        Args:
            query_embedding: Embedding vector to search for similar items
            limit: Maximum number of results to return

        Returns:
            List of similar content with metadata
        """
        try:
            results = self.client.search(
                collection_name=self.collection_name,
                query_vector=query_embedding,
                limit=limit
            )

            # Format results to include content and metadata
            formatted_results = []
            for result in results:
                formatted_results.append({
                    "chunk_id": result.id,
                    "content": result.payload.get("content", ""),
                    "source_url": result.payload.get("source_url", ""),
                    "module": result.payload.get("module", ""),
                    "score": result.score,
                    "metadata": {k: v for k, v in result.payload.items() if k not in ["content"]}
                })

            return formatted_results
        except Exception as e:
            print(f"Error searching for similar content: {e}")
            return []

    def get_chunk_by_id(self, chunk_id: str) -> Optional[Dict[str, Any]]:
        """
        Retrieve a specific chunk by its ID

        Args:
            chunk_id: ID of the chunk to retrieve

        Returns:
            Chunk data with metadata or None if not found
        """
        try:
            records = self.client.retrieve(
                collection_name=self.collection_name,
                ids=[chunk_id]
            )

            if records:
                record = records[0]
                return {
                    "chunk_id": record.id,
                    "content": record.payload.get("content", ""),
                    "source_url": record.payload.get("source_url", ""),
                    "module": record.payload.get("module", ""),
                    "metadata": {k: v for k, v in record.payload.items() if k not in ["content"]}
                }
            return None
        except Exception as e:
            print(f"Error retrieving chunk {chunk_id}: {e}")
            return None

    def delete_chunk(self, chunk_id: str) -> bool:
        """
        Delete a chunk from the vector database

        Args:
            chunk_id: ID of the chunk to delete

        Returns:
            True if successful, False otherwise
        """
        try:
            self.client.delete(
                collection_name=self.collection_name,
                points_selector=models.PointIdsList(
                    points=[chunk_id]
                )
            )
            return True
        except Exception as e:
            print(f"Error deleting chunk {chunk_id}: {e}")
            return False

    def get_collection_stats(self) -> Dict[str, Any]:
        """
        Get statistics about the collection

        Returns:
            Dictionary with collection statistics
        """
        try:
            collection_info = self.client.get_collection(self.collection_name)
            return {
                "vectors_count": collection_info.vectors_count,
                "indexed_vectors_count": collection_info.indexed_vectors_count,
                "points_count": collection_info.points_count
            }
        except Exception as e:
            print(f"Error getting collection stats: {e}")
            return {}