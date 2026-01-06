# Internal API Contracts: RAG Pipeline

## Module: rag_pipeline.config.settings

### Interface: Settings
**Purpose**: Configuration management using environment variables

**Methods**:
- `load_config() -> PipelineConfig`: Load configuration from environment variables
- `validate_config(config: PipelineConfig) -> bool`: Validate configuration parameters

**Input/Output**:
- Input: Environment variables (COHERE_API_KEY, QDRANT_URL, etc.)
- Output: PipelineConfig object with validated settings

## Module: rag_pipeline.ingestion.crawler

### Interface: DocusaurusCrawler
**Purpose**: Fetch content from Docusaurus website URLs

**Methods**:
- `fetch_page(url: str) -> str`: Fetch HTML content from a URL
- `fetch_all_pages(urls: List[str]) -> Dict[str, str]`: Fetch content from multiple URLs

**Input/Output**:
- Input: URL string(s)
- Output: HTML content string(s)

## Module: rag_pipeline.ingestion.parser

### Interface: DocusaurusParser
**Purpose**: Extract clean text content from Docusaurus HTML

**Methods**:
- `extract_content(html: str) -> str`: Extract main content from HTML
- `extract_metadata(html: str, url: str) -> Dict[str, str]`: Extract metadata (module, chapter, etc.)

**Input/Output**:
- Input: HTML content string and URL
- Output: Clean text content and metadata dictionary

## Module: rag_pipeline.processing.chunker

### Interface: ContentChunker
**Purpose**: Split content into deterministic chunks

**Methods**:
- `chunk_content(text: str, max_size: int, overlap: int) -> List[ContentChunk]`: Split text into chunks
- `generate_chunk_id(source_url: str, content: str) -> str`: Generate unique ID for a chunk

**Input/Output**:
- Input: Text content, max chunk size, overlap
- Output: List of ContentChunk objects

## Module: rag_pipeline.processing.cleaner

### Interface: TextCleaner
**Purpose**: Clean and normalize text content

**Methods**:
- `clean_text(text: str) -> str`: Remove extraneous whitespace and normalize text
- `normalize_content(text: str) -> str`: Apply standard text normalization

**Input/Output**:
- Input: Raw text content
- Output: Clean, normalized text

## Module: rag_pipeline.embedding.generator

### Interface: CohereEmbedder
**Purpose**: Generate embeddings using Cohere API

**Methods**:
- `generate_embedding(text: str, model: str) -> List[float]`: Generate embedding vector
- `batch_generate_embeddings(texts: List[str], model: str) -> List[List[float]]`: Generate multiple embeddings

**Input/Output**:
- Input: Text content and model name
- Output: Embedding vector (list of floats)

## Module: rag_pipeline.storage.vector_db

### Interface: QdrantStorage
**Purpose**: Store embeddings in Qdrant Cloud

**Methods**:
- `store_embedding(chunk: ContentChunk, embedding: List[float]) -> bool`: Store a single embedding
- `store_batch(chunks: List[ContentChunk], embeddings: List[List[float]]) -> Dict[str, bool]`: Store multiple embeddings
- `check_duplicate(chunk_id: str) -> bool`: Check if chunk already exists in storage

**Input/Output**:
- Input: ContentChunk object and embedding vector
- Output: Success status (boolean) or batch results

## Module: rag_pipeline.main

### Interface: PipelineOrchestrator
**Purpose**: Coordinate the entire pipeline execution

**Methods**:
- `run_pipeline(config: PipelineConfig) -> ProcessingResult`: Execute full pipeline
- `run_stage(stage: str, config: PipelineConfig) -> Any`: Execute specific pipeline stage

**Input/Output**:
- Input: PipelineConfig object
- Output: ProcessingResult object with execution summary