"""
Hybrid Free-Tier Safe + Full RAG Agent
======================================

- Supports citations and multi-step retrieval
- Limits tokens to avoid 402 errors on OpenRouter free tier
- Truncates chunks to fit token budget
- Handles selected-text-only mode
- Deterministic responses
"""

import os
import json
import hashlib
import time
import logging
from typing import Dict, List, Optional, Any
from dataclasses import dataclass
from openai import OpenAI
import cohere
from qdrant_client import QdrantClient
from dotenv import load_dotenv

# Logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Load env
load_dotenv()

# Free-tier safe configs
MAX_TOKENS = int(os.getenv("MAX_TOKENS", "300"))
OPENAI_MODEL = os.getenv("AGENT_MODEL", "openai/gpt-4o-mini")  # cheaper model
TOP_K = int(os.getenv("TOP_K", "3"))
CHUNK_TRUNCATE = int(os.getenv("CHUNK_TRUNCATE", "500"))

AGENT_INSTRUCTIONS = """
You are a RAG assistant for book content.
Answer only based on retrieved content.
Cite sources (module, chapter, URL).
Do not hallucinate.
Be deterministic.
"""

@dataclass
class RetrievedChunk:
    text: str
    score: float
    source_url: str
    module: str
    chapter: str
    chunk_index: int

@dataclass
class UserQuery:
    id: str
    text: str
    selected_text_only: bool = False
    selected_text_content: Optional[str] = None
    timestamp: Optional[float] = None

@dataclass
class AgentResponse:
    id: str
    content: str
    citations: List[Dict[str, str]]
    retrieved_chunks_used: List[RetrievedChunk]
    refusal_reason: Optional[str] = None
    timestamp: Optional[float] = None
    deterministic_hash: Optional[str] = None

# -------------------- Retriever --------------------
class RAGRetriever:
    def __init__(self):
        self.cohere_client = cohere.Client(os.getenv("COHERE_API_KEY"))
        self.qdrant_client = QdrantClient(
            url=os.getenv("QDRANT_URL"),
            api_key=os.getenv("QDRANT_API_KEY")
        )
        self.collection_name = os.getenv("QDRANT_COLLECTION_NAME", "book_embeddings")
        self.retrieval_threshold = float(os.getenv("RETRIEVAL_THRESHOLD", "0.3"))

    def retrieve_content(
        self, query: str, top_k: int = TOP_K, selected_text_only: bool = False,
        selected_text_content: Optional[str] = None
    ) -> Dict[str, Any]:
        try:
            if selected_text_only and selected_text_content:
                return {
                    "chunks": [{
                        "text": selected_text_content[:CHUNK_TRUNCATE],
                        "score": 1.0,
                        "metadata": {
                            "source_url": "selected_text",
                            "module": "selected",
                            "chapter": "selected",
                            "chunk_index": 0
                        }
                    }],
                    "empty_retrieval": False,
                    "conflicting_info": False
                }

            embed = self.cohere_client.embed(
                texts=[query],
                model=os.getenv("COHERE_MODEL", "embed-english-v3.0"),
                input_type="search_document"
            )

            results = self.qdrant_client.search(
                collection_name=self.collection_name,
                query_vector=embed.embeddings[0],
                limit=top_k,
                with_payload=True
            )

            chunks = []
            for r in results[:top_k]:
                if r.score >= self.retrieval_threshold:
                    text = r.payload.get("content", "")[:CHUNK_TRUNCATE]
                    metadata = r.payload.get("metadata", {})
                    chunks.append({"text": text, "score": r.score, "metadata": metadata})

            return {"chunks": chunks, "empty_retrieval": len(chunks) == 0, "conflicting_info": False}

        except Exception as e:
            logger.error(f"Retrieval error: {e}")
            return {"chunks": [], "empty_retrieval": True, "conflicting_info": False, "error": str(e)}

# -------------------- Agent --------------------
class RAGAgent:
    def __init__(self):
        self.client = OpenAI(
            api_key=os.getenv("OPENROUTER_API_KEY"),
            base_url="https://openrouter.ai/api/v1"  # important for sk-or keys
        )
        self.retriever = RAGRetriever()


        self.retrieval_tool = {
            "type": "function",
            "function": {
                "name": "retrieve_content",
                "description": "Retrieve relevant book content",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "query": {"type": "string"},
                        "top_k": {"type": "integer"},
                        "selected_text_only": {"type": "boolean"},
                        "selected_text_content": {"type": "string"}
                    },
                    "required": ["query"]
                }
            }
        }

    def query(self, user_query: UserQuery) -> AgentResponse:
        messages = [
            {"role": "system", "content": AGENT_INSTRUCTIONS},
            {"role": "user", "content": user_query.text}
        ]

        try:
            # Initial model call with free-tier safe max tokens
            response = self.client.chat.completions.create(
                model=OPENAI_MODEL,
                messages=messages,
                tools=[self.retrieval_tool],
                tool_choice="auto",
                max_tokens=MAX_TOKENS,
                temperature=0.0,
                top_p=1.0
            )

            msg = response.choices[0].message
            tool_calls = msg.tool_calls or []
            tool_results = []

            # Execute retrieval if requested
            if tool_calls:
                messages.append({"role": "assistant", "content": msg.content, "tool_calls": tool_calls})
                for call in tool_calls:
                    args = json.loads(call.function.arguments)
                    result = self.retriever.retrieve_content(**args)
                    messages.append({"role": "tool", "tool_call_id": call.id, "content": json.dumps(result)})
                    tool_results.append(result)

                # Final response using retrieved content
                final = self.client.chat.completions.create(
                    model=OPENAI_MODEL,
                    messages=messages,
                    tool_choice="none",
                    max_tokens=MAX_TOKENS,
                    temperature=0.0,
                    top_p=1.0
                )
                content = final.choices[0].message.content
            else:
                content = msg.content

            # Extract citations
            citations = []
            retrieved_chunks_used = []
            for result in tool_results:
                for chunk in result.get("chunks", []):
                    retrieved_chunks_used.append(
                        RetrievedChunk(
                            text=chunk["text"],
                            score=chunk["score"],
                            source_url=chunk["metadata"].get("source_url", ""),
                            module=chunk["metadata"].get("module", ""),
                            chapter=chunk["metadata"].get("chapter", ""),
                            chunk_index=chunk["metadata"].get("chunk_index", 0)
                        )
                    )
                    citation = {
                        "source_url": chunk["metadata"].get("source_url", ""),
                        "module": chunk["metadata"].get("module", ""),
                        "chapter": chunk["metadata"].get("chapter", "")
                    }
                    if citation not in citations:
                        citations.append(citation)

            refusal_reason = None
            if "not found in book" in content.lower():
                refusal_reason = "No relevant content found in the book."

            return AgentResponse(
                id=f"resp_{abs(hash(user_query.text))}",
                content=content,
                citations=citations,
                retrieved_chunks_used=retrieved_chunks_used,
                refusal_reason=refusal_reason,
                timestamp=time.time(),
                deterministic_hash=self._hash(user_query.text, content)
            )

        except Exception as e:
            logger.error(e)
            if "402" in str(e):
                return AgentResponse(
                    id=f"resp_{abs(hash(user_query.text))}",
                    content="⚠️ Token limit exceeded. Try a shorter question.",
                    citations=[],
                    retrieved_chunks_used=[],
                    refusal_reason="OpenRouter credit limit exceeded",
                    timestamp=time.time(),
                    deterministic_hash="credit_error"
                )
            return AgentResponse(
                id="error",
                content="An error occurred.",
                citations=[],
                retrieved_chunks_used=[],
                refusal_reason=str(e),
                timestamp=time.time(),
                deterministic_hash=self._hash(user_query.text, "error")
            )

    def _hash(self, q: str, r: str) -> str:
        return hashlib.sha256(f"{q}||{r}".encode()).hexdigest()

    def close(self):
        pass

# -------------------- Example Run --------------------
if __name__ == "__main__":
    agent = RAGAgent()
    query = UserQuery(id="q1", text="Explain ROS 2 fundamentals")
    response = agent.query(query)
    print(response.content)
    if response.citations:
        print("Citations:", response.citations)
