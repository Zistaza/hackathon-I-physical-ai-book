from qdrant_client import QdrantClient
from qdrant_client.http import models
from typing import List, Optional, Dict, Any
from backend.rag_pipeline.models.data import EmbeddingVector, ContentChunk
from backend.rag_pipeline.config.settings import load_config
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
            collection_info = self.client.get_collection(self.collection_name)
            # If collection exists but has wrong vector size, we need to recreate it
            if collection_info.config.params.vectors.size != 768:
                print(f"Collection has wrong vector size ({collection_info.config.params.vectors.size}), recreating...")
                self.client.delete_collection(self.collection_name)
                self.client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=models.VectorParams(
                        size=768,  # Updated to match Cohere's actual embedding dimensions (768 for the model being used)
                        distance=models.Distance.COSINE
                    )
                )
        except Exception:
            # Collection doesn't exist, create it
            # Using 768 dimensions based on the actual embedding size returned by the API
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=models.VectorParams(
                    size=768,  # Actual embedding dimension being returned
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
            # Use the content_chunk.id as a reference in payload but generate a valid Qdrant ID
            # Qdrant requires IDs to be either unsigned integers or UUIDs, so we'll use a hash
            # that we convert to an integer or use as UUID

            # Convert the chunk ID to a valid Qdrant ID (use the last 16 hex chars as int)
            # Since chunk.id is a SHA256 hash, we can convert part of it to int
            try:
                # Take the first 16 hex characters and convert to int
                qdrant_id = int(content_chunk.id[:16], 16) % (2**63)  # Ensure it fits in signed 64-bit int
            except ValueError:
                # If conversion fails, generate a numeric ID
                import hashlib
                qdrant_id = int(hashlib.md5(content_chunk.id.encode()).hexdigest()[:16], 16) % (2**63)

            # Check if this chunk already exists to maintain idempotency
            if self.check_duplicate_by_content_hash(content_chunk.content_hash):
                print(f"Chunk with content hash {content_chunk.content_hash} already exists in Qdrant, skipping...")
                return True  # Consider this a success since it already exists

            # Prepare the point to insert
            point = models.PointStruct(
                id=qdrant_id,  # Use a valid Qdrant ID
                vector=embedding.vector,
                payload={
                    "chunk_id": content_chunk.id,  # Original chunk ID in payload
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
                if self.check_duplicate_by_content_hash(chunk.content_hash):
                    print(f"Chunk with content hash {chunk.content_hash} already exists in Qdrant, skipping...")
                    results["successful"] += 1
                    continue

                # Convert the chunk ID to a valid Qdrant ID
                try:
                    # Take the first 16 hex characters and convert to int
                    qdrant_id = int(chunk.id[:16], 16) % (2**63)  # Ensure it fits in signed 64-bit int
                except ValueError:
                    # If conversion fails, generate a numeric ID
                    import hashlib
                    qdrant_id = int(hashlib.md5(chunk.id.encode()).hexdigest()[:16], 16) % (2**63)

                point = models.PointStruct(
                    id=qdrant_id,  # Use a valid Qdrant ID
                    vector=embedding.vector,
                    payload={
                        "chunk_id": chunk.id,  # Original chunk ID in payload
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

    def check_duplicate_by_content_hash(self, content_hash: str) -> bool:
        """
        Check if a chunk with the given content hash already exists in storage

        Args:
            content_hash: Hash of the content to check

        Returns:
            True if chunk exists, False otherwise
        """
        try:
            # Use scroll to find records with the given content hash in the payload
            records, _ = self.client.scroll(
                collection_name=self.collection_name,
                scroll_filter=models.Filter(
                    must=[
                        models.FieldCondition(
                            key="content_hash",
                            match=models.MatchValue(value=content_hash)
                        )
                    ]
                ),
                limit=1
            )
            return len(records) > 0
        except Exception:
            # If there's an error, assume it doesn't exist
            return False

    def check_duplicate(self, chunk_id: str) -> bool:
        """
        Check if a chunk already exists in storage (by original chunk ID)

        Args:
            chunk_id: Original chunk ID to check

        Returns:
            True if chunk exists, False otherwise
        """
        try:
            # Convert chunk_id to the Qdrant ID format to check if it exists
            try:
                qdrant_id = int(chunk_id[:16], 16) % (2**63)
            except ValueError:
                import hashlib
                qdrant_id = int(hashlib.md5(chunk_id.encode()).hexdigest()[:16], 16) % (2**63)

            # Try to retrieve the point by the converted ID
            records = self.client.retrieve(
                collection_name=self.collection_name,
                ids=[qdrant_id]
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