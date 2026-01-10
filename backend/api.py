"""
FastAPI Application for RAG Backend Frontend Integration
"""

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from contextlib import asynccontextmanager
import logging
import os
from dotenv import load_dotenv  # <-- add this

# Load .env variables at startup
load_dotenv()  # This reads .env at project root

# Optional: check that keys are loaded
logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO)
logger.info("OPENROUTER_API_KEY loaded: %s", "set" if os.getenv("OPENROUTER_API_KEY") else "missing")
logger.info("OPENAI_API_KEY loaded: %s", "set" if os.getenv("OPENAI_API_KEY") else "missing")
logger.info("COHERE_API_KEY loaded: %s", "set" if os.getenv("COHERE_API_KEY") else "missing")

# Import routers
from .routers import health, rag

@asynccontextmanager
async def lifespan(app: FastAPI):
    logger.info("Starting up RAG Backend API...")
    yield
    logger.info("Shutting down RAG Backend API...")

# Create FastAPI app
app = FastAPI(
    title="RAG Backend API",
    description="API for RAG backend integration with book frontend",
    version="1.0.0",
    lifespan=lifespan
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include routers
app.include_router(health.router, prefix="", tags=["health"])
app.include_router(rag.router, prefix="/rag", tags=["rag"])

# Root endpoint
@app.get("/")
async def root():
    return {"message": "RAG Backend API is running", "version": "1.0.0"}

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
