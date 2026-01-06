from typing import List, Tuple
import time
from .config.settings import load_config, PipelineConfig
from .models.data import ContentChunk, EmbeddingVector, ProcessingResult
from .ingestion.crawler import DocusaurusCrawler
from .ingestion.parser import DocusaurusParser
from .processing.cleaner import TextCleaner
from .processing.chunker import ContentChunker
from .embedding.generator import CohereEmbedder
from .storage.vector_db import QdrantStorage
from .utils.helpers import format_elapsed_time
from datetime import datetime


class PipelineOrchestrator:
    """
    Main orchestrator for the RAG pipeline that coordinates all components
    """
    def __init__(self, config: PipelineConfig = None):
        """
        Initialize the pipeline orchestrator

        Args:
            config: Pipeline configuration (if not provided, will load from environment)
        """
        self.config = config or load_config()
        self.crawler = DocusaurusCrawler()
        self.parser = DocusaurusParser()
        self.cleaner = TextCleaner()
        self.chunker = ContentChunker(
            max_chunk_size=self.config.chunk_size,
            overlap=self.config.chunk_overlap
        )
        self.embedder = CohereEmbedder(
            api_key=self.config.cohere_api_key,
            model_name=self.config.cohere_model
        )
        self.storage = QdrantStorage(
            url=self.config.qdrant_url,
            api_key=self.config.qdrant_api_key,
            collection_name=self.config.qdrant_collection_name
        )

    def run_pipeline(self) -> ProcessingResult:
        """
        Execute the full pipeline: crawl -> parse -> clean -> chunk -> embed -> store

        Returns:
            ProcessingResult with statistics about the run
        """
        start_time = datetime.now()
        print(f"Starting RAG pipeline at {start_time.strftime('%Y-%m-%d %H:%M:%S')}")

        processed_chunks_count = 0
        failed_chunks_count = 0
        error_details = []

        try:
            # Step 1: Crawl the Docusaurus URLs
            print("Step 1: Crawling Docusaurus websites...")
            crawled_results = self.crawler.fetch_all_pages(self.config.docusaurus_urls)

            # Process crawled results
            successful_crawls = []
            for url, content, error in crawled_results:
                if error:
                    print(f"Failed to crawl {url}: {error}")
                    failed_chunks_count += 1
                    error_details.append(f"Failed to crawl {url}: {error}")
                else:
                    successful_crawls.append((url, content))

            # Step 2: Parse the HTML content
            print("Step 2: Parsing HTML content...")
            parsed_chunks = []
            for url, html in successful_crawls:
                try:
                    chunk = self.parser.parse_page(html, url)
                    parsed_chunks.append(chunk)
                except Exception as e:
                    print(f"Failed to parse {url}: {e}")
                    failed_chunks_count += 1
                    error_details.append(f"Failed to parse {url}: {e}")

            # Step 3: Clean and normalize content
            print("Step 3: Cleaning and normalizing content...")
            cleaned_chunks = []
            for chunk in parsed_chunks:
                try:
                    cleaned_content = self.cleaner.clean_and_normalize(chunk.content)
                    # Create a new chunk with cleaned content
                    cleaned_chunk = ContentChunk(
                        content=cleaned_content,
                        source_url=chunk.source_url,
                        module=chunk.module,
                        chunk_index=chunk.chunk_index,
                        id=chunk.id,
                        content_hash=chunk.content_hash
                    )
                    cleaned_chunks.append(cleaned_chunk)
                except Exception as e:
                    print(f"Failed to clean content from {chunk.source_url}: {e}")
                    failed_chunks_count += 1
                    error_details.append(f"Failed to clean content from {chunk.source_url}: {e}")

            # Step 4: Chunk the content
            print("Step 4: Chunking content...")
            all_chunks = []
            for chunk in cleaned_chunks:
                try:
                    # Chunk the content
                    chunked = self.chunker.chunk_content(
                        chunk.content,
                        source_url=chunk.source_url,
                        module=chunk.module
                    )
                    all_chunks.extend(chunked)
                except Exception as e:
                    print(f"Failed to chunk content from {chunk.source_url}: {e}")
                    failed_chunks_count += 1
                    error_details.append(f"Failed to chunk content from {chunk.source_url}: {e}")

            # Step 5: Generate embeddings
            print("Step 5: Generating embeddings...")
            embedding_results = []
            for chunk in all_chunks:
                try:
                    embedding = self.embedder.embed_single_chunk(chunk)
                    embedding_results.append((embedding, chunk))
                except Exception as e:
                    print(f"Failed to embed chunk from {chunk.source_url}: {e}")
                    failed_chunks_count += 1
                    error_details.append(f"Failed to embed chunk from {chunk.source_url}: {e}")

            # Step 6: Store embeddings in Qdrant
            print("Step 6: Storing embeddings in Qdrant...")
            for embedding, chunk in embedding_results:
                try:
                    success = self.storage.store_embedding(embedding, chunk)
                    if success:
                        processed_chunks_count += 1
                        print(f"Successfully stored chunk {chunk.id} from {chunk.source_url}")
                    else:
                        failed_chunks_count += 1
                        error_details.append(f"Failed to store chunk {chunk.id} from {chunk.source_url}")
                except Exception as e:
                    print(f"Failed to store chunk {chunk.id} from {chunk.source_url}: {e}")
                    failed_chunks_count += 1
                    error_details.append(f"Failed to store chunk {chunk.id} from {chunk.source_url}: {e}")

        except Exception as e:
            print(f"Pipeline failed with error: {e}")
            error_details.append(f"Pipeline failed: {e}")
            failed_chunks_count += len(all_chunks)  # Count remaining chunks as failed

        end_time = datetime.now()
        total_time = (end_time - start_time).total_seconds()

        # Determine status based on results
        if failed_chunks_count == 0 and processed_chunks_count > 0:
            status = "success"
        elif failed_chunks_count > 0 and processed_chunks_count > 0:
            status = "partial"
        else:
            status = "failed"

        # Create and return processing result
        result = ProcessingResult(
            id="",  # Will be auto-generated
            status=status,
            processed_chunks_count=processed_chunks_count,
            failed_chunks_count=failed_chunks_count,
            start_time=start_time,
            end_time=end_time,
            error_details="; ".join(error_details) if error_details else None
        )

        print(f"\nPipeline completed at {end_time.strftime('%Y-%m-%d %H:%M:%S')}")
        print(f"Total time: {format_elapsed_time(total_time)}")
        print(f"Processed: {processed_chunks_count}, Failed: {failed_chunks_count}")
        print(f"Status: {status}")
        if error_details:
            print(f"Errors: {len(error_details)}")

        return result

    def run_stage(self, stage: str, *args, **kwargs) -> any:
        """
        Execute a specific pipeline stage

        Args:
            stage: Name of the stage to run ('crawl', 'parse', 'clean', 'chunk', 'embed', 'store')
            *args, **kwargs: Arguments specific to the stage

        Returns:
            Result of the stage execution
        """
        if stage.lower() == 'crawl':
            urls = args[0] if args else self.config.docusaurus_urls
            return self.crawler.fetch_all_pages(urls)
        elif stage.lower() == 'parse':
            html = args[0] if args else ""
            source_url = kwargs.get('source_url', '')
            return self.parser.parse_page(html, source_url)
        elif stage.lower() == 'clean':
            content = args[0] if args else ""
            return self.cleaner.clean_and_normalize(content)
        elif stage.lower() == 'chunk':
            content = args[0] if args else ""
            source_url = kwargs.get('source_url', '')
            module = kwargs.get('module', '')
            return self.chunker.chunk_content(content, source_url, module)
        elif stage.lower() == 'embed':
            content_chunk = args[0] if args else None
            if content_chunk:
                return self.embedder.embed_single_chunk(content_chunk)
            else:
                return None
        elif stage.lower() == 'store':
            embedding = args[0] if args else None
            content_chunk = args[1] if len(args) > 1 else None
            if embedding and content_chunk:
                return self.storage.store_embedding(embedding, content_chunk)
            else:
                return False
        else:
            raise ValueError(f"Unknown stage: {stage}")

    def test_connection(self) -> bool:
        """
        Test connections to all required services

        Returns:
            True if all connections are successful, False otherwise
        """
        print("Testing connections...")
        try:
            # Test Cohere connection by making a simple API call
            print("Testing Cohere connection...")
            test_embedding = self.embedder.generate_embedding("test")
            print("✓ Cohere connection successful")

            # Test Qdrant connection by getting collection stats
            print("Testing Qdrant connection...")
            stats = self.storage.get_collection_stats()
            print(f"✓ Qdrant connection successful - Collection has {stats.get('points_count', 0)} points")

            # Test that we can load config
            print("✓ Configuration loaded successfully")

            return True
        except Exception as e:
            print(f"Connection test failed: {e}")
            return False

    def close(self):
        """
        Close all resources used by the pipeline
        """
        self.crawler.close()


def main():
    """
    Main function to run the pipeline from command line
    """
    import sys
    import argparse

    parser = argparse.ArgumentParser(description='RAG Pipeline for Docusaurus Content Ingestion')
    parser.add_argument('--stage', type=str, help='Run a specific stage (crawl, parse, clean, chunk, embed, store, full)')
    parser.add_argument('--test', action='store_true', help='Test connections only')
    parser.add_argument('--config', type=str, help='Path to config file (not used, loads from environment)')

    args = parser.parse_args()

    # Load configuration
    config = load_config()

    # Create orchestrator
    orchestrator = PipelineOrchestrator(config)

    try:
        if args.test:
            # Test connections only
            success = orchestrator.test_connection()
            sys.exit(0 if success else 1)
        elif args.stage and args.stage.lower() == 'full':
            # Run full pipeline
            result = orchestrator.run_pipeline()
            print(f"\nFinal Result: {result.status}")
            print(f"Processed: {result.processed_chunks_count}, Failed: {result.failed_chunks_count}")
            sys.exit(0 if result.status != 'failed' else 1)
        elif args.stage:
            # Run specific stage (for testing purposes)
            print(f"Running stage: {args.stage}")
            # For now, just run the full pipeline if a specific stage isn't implemented separately
            result = orchestrator.run_pipeline()
            sys.exit(0 if result.status != 'failed' else 1)
        else:
            # Default: run full pipeline
            result = orchestrator.run_pipeline()
            print(f"\nFinal Result: {result.status}")
            print(f"Processed: {result.processed_chunks_count}, Failed: {result.failed_chunks_count}")
            sys.exit(0 if result.status != 'failed' else 1)
    finally:
        orchestrator.close()


if __name__ == "__main__":
    main()