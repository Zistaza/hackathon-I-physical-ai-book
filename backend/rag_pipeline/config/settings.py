from pydantic import BaseModel, Field, validator
from typing import List, Optional
from dotenv import load_dotenv
import os

load_dotenv()


class PipelineConfig(BaseModel):
    """
    Configuration settings for the RAG pipeline
    """
    docusaurus_urls: List[str] = Field(
        default_factory=list,
        description="List of Docusaurus website URLs to process"
    )
    chunk_size: int = Field(
        default=1000,
        ge=100,
        le=10000,
        description="Maximum number of characters per content chunk"
    )
    chunk_overlap: int = Field(
        default=100,
        ge=0,
        description="Number of characters to overlap between chunks"
    )
    cohere_model: str = Field(
        default="embed-multilingual-v2.0",
        description="Name of the Cohere model to use for embeddings"
    )
    qdrant_collection_name: str = Field(
        default="rag_pipeline_chunks",
        description="Name of the Qdrant collection to store vectors"
    )
    qdrant_url: str = Field(
        default=...,
        description="URL of the Qdrant Cloud instance"
    )
    qdrant_api_key: str = Field(
        default=...,
        description="API key for Qdrant Cloud"
    )
    cohere_api_key: str = Field(
        default=...,
        description="API key for Cohere"
    )

    @validator('docusaurus_urls')
    def validate_urls(cls, v):
        """Validate that all URLs are properly formatted"""
        import re
        url_pattern = re.compile(
            r'^https?://'  # http:// or https://
            r'(?:(?:[A-Z0-9](?:[A-Z0-9-]{0,61}[A-Z0-9])?\.)+[A-Z]{2,6}\.?|'  # domain...
            r'localhost|'  # localhost...
            r'\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3})'  # ...or ip
            r'(?::\d+)?'  # optional port
            r'(?:/?|[/?]\S+)$', re.IGNORECASE)
        for url in v:
            if not url_pattern.match(url):
                raise ValueError(f'Invalid URL: {url}')
        return v

    @validator('chunk_overlap')
    def validate_chunk_overlap(cls, v, values):
        """Validate that chunk overlap is less than chunk size"""
        if 'chunk_size' in values and v >= values['chunk_size']:
            raise ValueError('chunk_overlap must be less than chunk_size')
        return v


def load_config() -> PipelineConfig:
    """
    Load configuration from environment variables
    """
    docusaurus_urls = os.getenv('DOCUSAURUS_URLS', '')
    urls_list = [url.strip() for url in docusaurus_urls.split(',') if url.strip()]

    config = PipelineConfig(
        docusaurus_urls=urls_list,
        chunk_size=int(os.getenv('CHUNK_SIZE', '1000')),
        chunk_overlap=int(os.getenv('CHUNK_OVERLAP', '100')),
        cohere_model=os.getenv('COHERE_MODEL', 'embed-multilingual-v2.0'),
        qdrant_collection_name=os.getenv('QDRANT_COLLECTION_NAME', 'rag_pipeline_chunks'),
        qdrant_url=os.getenv('QDRANT_URL', ''),
        qdrant_api_key=os.getenv('QDRANT_API_KEY', ''),
        cohere_api_key=os.getenv('COHERE_API_KEY', ''),
    )
    return config


def validate_config(config: PipelineConfig) -> bool:
    """
    Validate configuration parameters
    """
    try:
        config.validate()
        return True
    except Exception as e:
        print(f"Configuration validation error: {e}")
        return False