# API Contracts: RAG Agent with OpenAI Agents SDK

## Overview
This document defines the API contracts for the RAG Agent built with the OpenAI Agents SDK. Since the agent operates through the OpenAI platform, the contracts define the interface between the agent and the retrieval system.

## Agent Configuration Contract

### System Instructions Schema
```json
{
  "type": "string",
  "description": "System instructions for the RAG agent",
  "required": [
    "grounding_requirement",
    "citation_requirement",
    "refusal_behavior",
    "selected_text_mode"
  ],
  "properties": {
    "grounding_requirement": {
      "type": "string",
      "description": "Must answer only from retrieved content",
      "const": "Answer only using information from the retrieved book content. Do not use any external knowledge."
    },
    "citation_requirement": {
      "type": "string",
      "description": "Must cite sources or refuse",
      "const": "Every answer must cite module/chapter/URL metadata or explicitly state 'answer not found in book'."
    },
    "refusal_behavior": {
      "type": "string",
      "description": "Safe refusal when content not found",
      "const": "Refuse to answer when the requested information is not available in the retrieved content."
    },
    "selected_text_mode": {
      "type": "string",
      "description": "Handle selected text only mode",
      "const": "When in selected-text-only mode, only use the provided selected text for answers."
    }
  }
}
```

## Tool Schema Contracts

### Retrieval Tool Contract
```json
{
  "type": "object",
  "description": "Tool for retrieving content from the book vector store",
  "required": ["type", "function"],
  "properties": {
    "type": {
      "type": "string",
      "const": "function"
    },
    "function": {
      "type": "object",
      "required": ["name", "description", "parameters"],
      "properties": {
        "name": {
          "type": "string",
          "const": "retrieve_content"
        },
        "description": {
          "type": "string",
          "description": "Retrieve relevant content from the book based on the user query",
          "const": "Search the book content for information relevant to the user's query and return the most relevant chunks with their metadata."
        },
        "parameters": {
          "type": "object",
          "required": ["type", "properties"],
          "properties": {
            "type": {
              "type": "string",
              "const": "object"
            },
            "properties": {
              "type": "object",
              "required": ["query"],
              "properties": {
                "query": {
                  "type": "string",
                  "description": "The user's question or query to search for in the book content"
                },
                "top_k": {
                  "type": "integer",
                  "description": "Number of results to return (default: 5)",
                  "minimum": 1,
                  "maximum": 10,
                  "default": 5
                },
                "selected_text_only": {
                  "type": "boolean",
                  "description": "Whether to restrict search to selected text only",
                  "default": false
                },
                "selected_text_content": {
                  "type": "string",
                  "description": "The specific text content to search within when selected_text_only is true"
                }
              }
            }
          }
        }
      }
    }
  }
}
```

### Retrieved Content Response Schema
```json
{
  "type": "object",
  "description": "Response from the retrieval tool",
  "required": ["chunks"],
  "properties": {
    "chunks": {
      "type": "array",
      "items": {
        "type": "object",
        "required": ["text", "score", "metadata"],
        "properties": {
          "text": {
            "type": "string",
            "description": "The retrieved content text"
          },
          "score": {
            "type": "number",
            "description": "Similarity score for the retrieved chunk",
            "minimum": 0,
            "maximum": 1
          },
          "metadata": {
            "type": "object",
            "required": ["source_url", "module", "chapter"],
            "properties": {
              "source_url": {
                "type": "string",
                "description": "URL of the original document"
              },
              "module": {
                "type": "string",
                "description": "Module identifier"
              },
              "chapter": {
                "type": "string",
                "description": "Chapter identifier"
              },
              "chunk_index": {
                "type": "integer",
                "description": "Index of the chunk within the document"
              }
            }
          }
        }
      }
    },
    "empty_retrieval": {
      "type": "boolean",
      "description": "Indicates if no content was retrieved",
      "default": false
    },
    "conflicting_info": {
      "type": "boolean",
      "description": "Indicates if conflicting information was found in retrieved chunks",
      "default": false
    }
  }
}
```

## Agent Interaction Contracts

### Query Processing Contract
```json
{
  "type": "object",
  "description": "Contract for processing user queries",
  "required": ["query", "mode"],
  "properties": {
    "query": {
      "type": "string",
      "description": "The user's question to be answered"
    },
    "mode": {
      "type": "string",
      "enum": ["full_book", "selected_text_only"],
      "description": "The retrieval mode to use"
    },
    "selected_text": {
      "type": "string",
      "description": "Text to constrain retrieval to when mode is selected_text_only"
    }
  }
}
```

### Response Generation Contract
```json
{
  "type": "object",
  "description": "Contract for agent response generation",
  "required": ["status", "content"],
  "properties": {
    "status": {
      "type": "string",
      "enum": ["success", "refused", "error"],
      "description": "Status of the response generation"
    },
    "content": {
      "type": "string",
      "description": "The response content"
    },
    "citations": {
      "type": "array",
      "items": {
        "type": "object",
        "required": ["source_url", "module", "chapter"],
        "properties": {
          "source_url": {
            "type": "string",
            "description": "URL of the cited source"
          },
          "module": {
            "type": "string",
            "description": "Module of the cited source"
          },
          "chapter": {
            "type": "string",
            "description": "Chapter of the cited source"
          }
        }
      },
      "description": "List of citations used in the response"
    },
    "refusal_reason": {
      "type": "string",
      "description": "Reason for refusal if status is 'refused'"
    },
    "deterministic": {
      "type": "boolean",
      "description": "Whether the response is deterministic",
      "default": true
    }
  }
}
```

## Determinism Contract

### Input-Output Consistency
```json
{
  "type": "object",
  "description": "Contract ensuring deterministic behavior",
  "required": ["input_hash", "output_hash", "consistent"],
  "properties": {
    "input_hash": {
      "type": "string",
      "description": "Hash of the input query and retrieval results"
    },
    "output_hash": {
      "type": "string",
      "description": "Hash of the generated response"
    },
    "consistent": {
      "type": "boolean",
      "description": "Whether identical inputs produce identical outputs"
    },
    "validation_timestamp": {
      "type": "string",
      "description": "Timestamp of determinism validation",
      "format": "date-time"
    }
  }
}
```

## Error Handling Contracts

### Retrieval Error Contract
```json
{
  "type": "object",
  "description": "Contract for handling retrieval errors",
  "required": ["error_type", "message", "fallback_action"],
  "properties": {
    "error_type": {
      "type": "string",
      "enum": ["connection_error", "empty_results", "service_unavailable", "invalid_query"]
    },
    "message": {
      "type": "string",
      "description": "Human-readable error message"
    },
    "fallback_action": {
      "type": "string",
      "enum": ["refuse_answer", "retry", "use_default_response"],
      "description": "Action to take when error occurs"
    }
  }
}
```

## Validation Requirements

### Functional Requirement Mapping
- **FR-001**: Query input must conform to Query Processing Contract
- **FR-002**: Agent must utilize Retrieval Tool Contract
- **FR-003**: Retrieval must use Qdrant + Cohere as specified
- **FR-004**: Response Generation Contract enforces grounding in retrieved chunks
- **FR-005**: Citations must follow Citation Contract schema
- **FR-006**: Refusal behavior follows Response Generation Contract
- **FR-007**: Full-book mode uses default retrieval parameters
- **FR-008**: Selected-text-only mode uses selected_text_only parameter
- **FR-009**: Determinism Contract ensures identical inputs produce identical outputs
- **FR-010**: Response Generation Contract prevents external knowledge injection
- **FR-011**: Handles empty retrieval results via Empty Retrieval flag
- **FR-012**: Conflicting information detection via Conflicting Info flag
- **FR-013**: All functionality is testable via defined contracts