# Quickstart: RAG Pipeline – Website Content Ingestion, Embedding Generation, and Vector Storage

## Prerequisites

- Python 3.11 or higher
- uv package manager
- Access to Cohere API (API key)
- Access to Qdrant Cloud (URL and API key)
- List of Docusaurus website URLs to process

## Setup

1. **Clone the repository** (if not already done):
   ```bash
   git clone <repository-url>
   cd <repository-name>
   ```

2. **Install uv** (if not already installed):
   ```bash
   pip install uv
   ```

3. **Create and activate virtual environment**:
   ```bash
   uv venv
   source .venv/bin/activate  # On Windows: .venv\Scripts\activate
   ```

4. **Install dependencies**:
   ```bash
   uv pip install -r backend/requirements.txt
   ```

5. **Set up environment variables**:
   ```bash
   cp backend/.env.example .env
   ```

   Then edit `.env` to add your:
   - `COHERE_API_KEY`: Your Cohere API key
   - `QDRANT_URL`: Your Qdrant Cloud URL
   - `QDRANT_API_KEY`: Your Qdrant Cloud API key
   - `DOCUSAURUS_URLS`: Comma-separated list of Docusaurus URLs to process

## Usage

### Run the full pipeline:
```bash
cd backend
python -m rag_pipeline.main
```

### Run with specific configuration:
```bash
cd backend
python -m rag_pipeline.main --config-path /path/to/config.json
```

### Run specific pipeline stages:
```bash
# Just crawl and extract content
python -m rag_pipeline.main --stage crawl

# Just chunk the content
python -m rag_pipeline.main --stage chunk

# Just generate embeddings
python -m rag_pipeline.main --stage embed

# Just store in vector database
python -m rag_pipeline.main --stage store
```

## Configuration

The pipeline can be configured via environment variables in `.env` or via a JSON config file. Key configuration options:

- `DOCUSAURUS_URLS`: List of URLs to process
- `CHUNK_SIZE`: Maximum character count per chunk (default: 1000)
- `CHUNK_OVERLAP`: Character overlap between chunks (default: 200)
- `COHERE_MODEL`: Cohere model to use (default: embed-multilingual-v3.0)
- `QDRANT_COLLECTION`: Name of Qdrant collection (default: rag_content)

## Example

```bash
# Set up environment variables
export COHERE_API_KEY="your-cohere-api-key"
export QDRANT_URL="https://your-cluster.europe-west3-0.cloud.qdrant.io:6333"
export QDRANT_API_KEY="your-qdrant-api-key"
export DOCUSAURUS_URLS="https://your-book-site.com/docs/intro,https://your-book-site.com/docs/setup"

# Run the pipeline
cd backend
python -m rag_pipeline.main
```

## Output

The pipeline will:
1. Fetch content from specified Docusaurus URLs
2. Extract clean text content, removing navigation and UI elements
3. Chunk the content using deterministic boundaries
4. Generate embeddings using Cohere
5. Store embeddings in Qdrant with metadata (source URL, module, chunk index)
6. Provide a summary of processing results

## Troubleshooting

- **API Rate Limits**: If you encounter rate limits from Cohere or Qdrant, consider adding delays between requests or upgrading your plan.
- **Content Extraction Issues**: If content isn't being extracted properly, check that the Docusaurus CSS selectors in the parser match your site's structure.
- **Memory Issues**: For large sites, consider processing URLs in batches or increasing available memory.