import React, { useState, useEffect } from 'react';
import './RagChatbot.css';

/**
 * RAG Chatbot Component
 *
 * A React component that integrates with the RAG backend API to provide
 * book question answering functionality.
 */
const RagChatbot = () => {
  const [query, setQuery] = useState('');
  const [response, setResponse] = useState(null);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState(null);
  const [selectedText, setSelectedText] = useState('');
  const [useSelectedText, setUseSelectedText] = useState(false);

  // Function to get selected text from the page
  useEffect(() => {
    const handleTextSelection = () => {
      const selectedText = window.getSelection().toString().trim();
      if (selectedText) {
        setSelectedText(selectedText);
      }
    };

    document.addEventListener('mouseup', handleTextSelection);
    return () => {
      document.removeEventListener('mouseup', handleTextSelection);
    };
  }, []);

  const handleSubmit = async (e) => {
    e.preventDefault();
    if (!query.trim()) return;

    setLoading(true);
    setError(null);
    setResponse(null);

    try {
      // Prepare the request payload
      let payload;
      if (useSelectedText && selectedText) {
        // Use selected text query endpoint
        payload = {
          question: query,
          selected_text_only: true,
          selected_text_content: selectedText,
        };
      } else {
        // Use general query endpoint
        payload = {
          question: query,
        };
      }

      // Determine the endpoint based on whether we're using selected text
      const endpoint = useSelectedText && selectedText
        ? 'http://localhost:8000/rag/query-selected-text'
        : 'http://localhost:8000/rag/query';

      const response = await fetch(endpoint, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(payload),
      });

      if (!response.ok) {
        throw new Error(`API request failed with status ${response.status}`);
      }

      const data = await response.json();
      setResponse(data);
    } catch (err) {
      console.error('Error:', err);
      setError('Failed to get response from backend. Please make sure the backend server is running on http://localhost:8000');
    } finally {
      setLoading(false);
    }
  };

  const handleUseSelectedTextChange = (e) => {
    setUseSelectedText(e.target.checked);
  };

  return (
    <div className="rag-chatbot">
      <div className="chatbot-header">
        <h3>Book Question Assistant</h3>
        <p>Ask questions about the book content. Select text on the page to ask questions about specific content.</p>
      </div>

      <form onSubmit={handleSubmit} className="chatbot-form">
        {selectedText && (
          <div className="selected-text-preview">
            <label>
              <input
                type="checkbox"
                checked={useSelectedText}
                onChange={handleUseSelectedTextChange}
              />
              Ask about selected text: <em>"{selectedText.substring(0, 100)}{selectedText.length > 100 ? '...' : ''}"</em>
            </label>
          </div>
        )}

        <textarea
          value={query}
          onChange={(e) => setQuery(e.target.value)}
          placeholder="Ask a question about the book content..."
          rows="4"
          cols="50"
          className="query-input"
          disabled={loading}
        />

        <button type="submit" disabled={loading} className="submit-button">
          {loading ? 'Asking...' : 'Ask Question'}
        </button>
      </form>

      {error && (
        <div className="error-message">
          <h4>Error:</h4>
          <p>{error}</p>
          <p className="error-hint">
            Hint: Make sure the backend server is running with <code>uvicorn backend.api:app --reload</code>
          </p>
        </div>
      )}

      {response && (
        <div className="response-container">
          <h4>Answer:</h4>
          <div className="answer">
            {response.answer}
          </div>

          {response.citations && response.citations.length > 0 && (
            <div className="citations">
              <h5>Sources:</h5>
              <ul className="citations-list">
                {response.citations.map((citation, index) => (
                  <li key={index} className="citation-item">
                    <div className="citation-main">
                      <strong>{citation.module}</strong> - {citation.chapter}
                      {citation.section && ` - ${citation.section}`}
                    </div>
                    {citation.source_url && (
                      <div className="citation-url">
                        Source: <a href={citation.source_url} target="_blank" rel="noopener noreferrer">
                          {citation.source_url}
                        </a>
                      </div>
                    )}
                    {citation.page_number !== undefined && (
                      <div className="citation-page">Page: {citation.page_number}</div>
                    )}
                    {citation.paragraph_number !== undefined && (
                      <div className="citation-paragraph">Paragraph: {citation.paragraph_number}</div>
                    )}
                    {citation.citation_type && (
                      <div className="citation-type">Type: {citation.citation_type}</div>
                    )}
                  </li>
                ))}
              </ul>
            </div>
          )}

          {response.retrieved_chunks && response.retrieved_chunks.length > 0 && (
            <div className="retrieved-chunks">
              <h5>Retrieved Content:</h5>
              <ul className="chunks-list">
                {response.retrieved_chunks.map((chunk, index) => (
                  <li key={index} className="chunk-item">
                    <div className="chunk-score">Score: {chunk.score.toFixed(2)}</div>
                    <div className="chunk-text">{chunk.text.substring(0, 200)}{chunk.text.length > 200 ? '...' : ''}</div>
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