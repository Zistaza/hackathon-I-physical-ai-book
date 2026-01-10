# Quickstart Guide: RAG Backend Frontend Integration

## Prerequisites

- Python 3.11+
- Node.js 18+ (for frontend development)
- Access to Qdrant Cloud (Free Tier)
- API keys for OpenAI/Anthropic/OpenRouter and Cohere
- Git

## Environment Setup

### 1. Clone the repository
```bash
git clone <repository-url>
cd hackathon-I-physical-ai-book
```

### 2. Set up Python virtual environment
```bash
python -m venv .venv
source .venv/bin/activate  # On Windows: .venv\Scripts\activate
pip install --upgrade pip
```

### 3. Install backend dependencies
```bash
cd backend
pip install -r requirements.txt
# If no requirements.txt exists yet, install FastAPI and related packages:
pip install fastapi uvicorn python-dotenv pydantic cohere qdrant-client openai
```

### 4. Configure environment variables
Create a `.env` file in the backend directory with the following variables:
```env
# API Keys
OPENAI_API_KEY=your_openai_api_key
OPENROUTER_API_KEY=your_openrouter_api_key  # Optional, for OpenRouter
COHERE_API_KEY=your_cohere_api_key

# Qdrant Configuration
QDRANT_URL=your_qdrant_cluster_url
QDRANT_API_KEY=your_qdrant_api_key
QDRANT_COLLECTION_NAME=book_embeddings

# Cohere Model Configuration
COHERE_MODEL=embed-english-v3.0

# Agent Configuration
AGENT_MODEL=gpt-4o
OPENROUTER_BASE_URL=https://openrouter.ai/api/v1

# Retrieval Configuration
RETRIEVAL_THRESHOLD=0.3
```

## Backend Setup

### 1. Create the backend structure
```bash
cd backend
mkdir -p models routers services
touch api.py config.py __init__.py
touch models/__init__.py models/query.py models/health.py
touch routers/__init__.py routers/health.py routers/rag.py
touch services/__init__.py services/rag_service.py
```

### 2. Start the FastAPI server
```bash
uvicorn api:app --reload --host 0.0.0.0 --port 8000
```

The API will be available at `http://localhost:8000`.

## Frontend Integration

### 1. Navigate to the Docusaurus site
```bash
cd my-website
```

### 2. Install frontend dependencies (if not already installed)
```bash
npm install
```

### 3. Create the RAG chatbot component
Create a new React component for the chatbot in `src/components/RagChatbot/index.js`:
```javascript
import React, { useState } from 'react';
import './RagChatbot.css';

const RagChatbot = () => {
  const [query, setQuery] = useState('');
  const [response, setResponse] = useState(null);
  const [loading, setLoading] = useState(false);

  const handleSubmit = async (e) => {
    e.preventDefault();
    setLoading(true);
    setResponse(null);

    try {
      const response = await fetch('http://localhost:8000/rag/query', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ question: query }),
      });

      const data = await response.json();
      setResponse(data);
    } catch (error) {
      console.error('Error:', error);
      setResponse({ error: 'Failed to get response from backend' });
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className="rag-chatbot">
      <h3>Book Question Assistant</h3>
      <form onSubmit={handleSubmit}>
        <textarea
          value={query}
          onChange={(e) => setQuery(e.target.value)}
          placeholder="Ask a question about the book content..."
          rows="3"
          cols="50"
        />
        <button type="submit" disabled={loading}>
          {loading ? 'Asking...' : 'Ask Question'}
        </button>
      </form>

      {response && (
        <div className="response">
          <h4>Answer:</h4>
          <p>{response.answer}</p>

          {response.citations && response.citations.length > 0 && (
            <div className="citations">
              <h5>Sources:</h5>
              <ul>
                {response.citations.map((citation, index) => (
                  <li key={index}>
                    {citation.module} - {citation.chapter}
                    {citation.source_url && ` (${citation.source_url})`}
                  </li>
                ))}
              </ul>
            </div>
          )}

          {response.refusal_reason && (
            <div className="refusal">
              <em>{response.refusal_reason}</em>
            </div>
          )}
        </div>
      )}
    </div>
  );
};

export default RagChatbot;
```

### 4. Add the chatbot to your Docusaurus pages
Import and use the component in your Docusaurus pages as needed.

## API Endpoints

Once the backend is running, you can test the endpoints:

### Health Check
```bash
curl http://localhost:8000/health
```

### General Book Query
```bash
curl -X POST http://localhost:8000/rag/query \
  -H "Content-Type: application/json" \
  -d '{"question": "What are the fundamental principles discussed in the book?"}'
```

### Selected Text Query
```bash
curl -X POST http://localhost:8000/rag/query-selected-text \
  -H "Content-Type: application/json" \
  -d '{
    "question": "What does this text say about robotics?",
    "selected_text_only": true,
    "selected_text_content": "The fundamental principles of robotics include kinematics, dynamics, and control theory..."
  }'
```

## Testing

### Backend Tests
```bash
cd backend
pytest tests/ -v
```

### API Testing
Use the API endpoints directly or use a tool like Postman to test the endpoints with various inputs.

## Troubleshooting

### Common Issues

1. **Environment Variables Not Loaded**
   - Ensure your `.env` file is in the correct location (backend directory)
   - Restart the server after changing environment variables

2. **Qdrant Connection Issues**
   - Verify QDRANT_URL and QDRANT_API_KEY are correct
   - Check that the Qdrant collection exists and has data

3. **API Key Issues**
   - Verify all required API keys are provided in the `.env` file
   - Check that the API keys have the necessary permissions

4. **CORS Issues**
   - If making frontend requests to the backend, ensure CORS is configured in FastAPI

## Next Steps

1. Integrate the RAG chatbot component into your Docusaurus pages
2. Add more sophisticated UI elements for the chatbot
3. Implement session tracking with Neon Postgres if needed
4. Add monitoring and logging to production deployments