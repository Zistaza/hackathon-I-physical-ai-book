import React, { useState, useEffect, useRef } from 'react';
import './RagChatbot.css';

/**
 * RAG Chatbot Component
 *
 * A React component that integrates with the RAG backend API to provide
 * book question answering functionality with modern UI and floating toggle.
 */
const RagChatbot = () => {
  const [query, setQuery] = useState('');
  const [response, setResponse] = useState(null);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState(null);
  const [selectedText, setSelectedText] = useState('');
  const [useSelectedText, setUseSelectedText] = useState(false);
  const [isOpen, setIsOpen] = useState(false);
  const [chatHistory, setChatHistory] = useState([]);

  const messagesEndRef = useRef(null);
  const chatContainerRef = useRef(null);

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

  // We no longer manage dark mode in the chatbot directly
  // The chatbot will inherit the inverted theme from the main website
  // through CSS selectors [data-theme='dark'] vs [data-theme='light']

  // Auto-scroll to bottom when chat history or response changes
  useEffect(() => {
    const scrollToBottom = () => {
      if (messagesEndRef.current) {
        messagesEndRef.current.scrollIntoView({ behavior: 'smooth' });
      }
    };

    // Small delay to ensure DOM has updated
    setTimeout(scrollToBottom, 100);
  }, [chatHistory, response]);

  const handleSubmit = async (e) => {
    e.preventDefault();
    if (!query.trim()) return;

    setLoading(true);
    setError(null);

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
      const newChatEntry = {
        id: Date.now(),
        query: query,
        response: data,
        timestamp: new Date()
      };

      setChatHistory(prev => [...prev, newChatEntry]);
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

  const clearChat = () => {
    setChatHistory([]);
    setResponse(null);
    setQuery('');
    setError(null);
  };

  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  const closeChat = () => {
    setIsOpen(false);
  };

  const nextQuestion = () => {
    // Just clear the current response but keep the chat history visible
    setQuery('');
    setError(null);
  };

  // Render individual chat message
  const renderChatMessage = (chatItem, index) => (
    <div key={chatItem.id} className="chat-message">
      <div className="user-query">
        <div className="query-text">{chatItem.query}</div>
      </div>
      <div className="bot-response">
        <div className="response-content">
          {chatItem.response.answer}
        </div>

        {chatItem.response.citations && chatItem.response.citations.length > 0 && (
          <div className="citations">
            <h5>Sources:</h5>
            <ul className="citations-list">
              {chatItem.response.citations.map((citation, idx) => (
                <li key={idx} className="citation-item">
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

        {chatItem.response.retrieved_chunks && chatItem.response.retrieved_chunks.length > 0 && (
          <div className="retrieved-chunks">
            <h5>Retrieved Content:</h5>
            <ul className="chunks-list">
              {chatItem.response.retrieved_chunks.map((chunk, idx) => (
                <li key={idx} className="chunk-item">
                  <div className="chunk-score">Score: {chunk.score.toFixed(2)}</div>
                  <div className="chunk-text">{chunk.text.substring(0, 200)}{chunk.text.length > 200 ? '...' : ''}</div>
                </li>
              ))}
            </ul>
          </div>
        )}

        {chatItem.response.refusal_reason && (
          <div className="refusal">
            <em>{chatItem.response.refusal_reason}</em>
          </div>
        )}

        <div className="response-actions">
          <button onClick={nextQuestion} className="next-question-btn">Ask Another Question</button>
        </div>
      </div>
    </div>
  );

  return (
    <>
      {/* Floating chat toggle button */}
      {!isOpen && (
        <button
          className="chat-toggle"
          onClick={toggleChat}
          aria-label="Open chatbot"
        >
          <span className="chat-icon">📚</span>
          <span className="chat-text">How can I help you to understand this book?</span>
          <span className="chat-badge">{chatHistory.length > 0 ? chatHistory.length : ''}</span>
        </button>
      )}

      {/* Chat interface */}
      {isOpen && (
        <div className="chat-container">
          <div className="chat-header">
            <div className="chat-title">
              <span className="chat-icon">📚</span>
              Book Assistant
            </div>
            <div className="chat-controls">
              <button
                className="clear-chat-btn"
                onClick={clearChat}
                title="Clear chat"
              >
                🗑️
              </button>
              <button
                className="close-chat-btn"
                onClick={closeChat}
                title="Close chat"
              >
                ×
              </button>
            </div>
          </div>

          <div className="chat-messages" ref={chatContainerRef}>
            {chatHistory.length === 0 ? (
              <div className="welcome-message">
                <h3>Welcome to the Book Assistant! 📚</h3>
                <p>Ask me anything about the Physical AI & Humanoid Robotics content.</p>
                <p>I can help you understand concepts, find specific information, or discuss the material.</p>
              </div>
            ) : (
              chatHistory.map((chatItem, index) => renderChatMessage(chatItem, index))
            )}

            {response && !chatHistory.some(chat => chat.response === response) && (
              <div className="current-response">
                <div className="bot-response">
                  <div className="response-content">
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

                  <div className="response-actions">
                    <button onClick={nextQuestion} className="next-question-btn">Ask Another Question</button>
                  </div>
                </div>
              </div>
            )}

            {error && (
              <div className="error-message">
                <h4>Error:</h4>
                <p>{error}</p>
                <p className="error-hint">
                  Hint: Make sure the backend server is running with <code>uvicorn backend.api:app --reload</code>
                </p>
              </div>
            )}

            {/* Scroll anchor */}
            <div ref={messagesEndRef} />
          </div>

          <form onSubmit={handleSubmit} className="chat-input-form">
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

            <div className="input-area">
              <textarea
                value={query}
                onChange={(e) => setQuery(e.target.value)}
                placeholder="Ask a question about the book content..."
                rows="3"
                className="query-input"
                disabled={loading}
                autoFocus
              />
              <button type="submit" disabled={loading} className="send-button">
                {loading ? '...' : '➤'}
              </button>
            </div>
          </form>
        </div>
      )}
    </>
  );
};

export default RagChatbot;