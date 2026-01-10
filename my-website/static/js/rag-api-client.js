/**
 * RAG API Client
 *
 * A JavaScript client for interacting with the RAG backend API endpoints.
 * Provides methods for health checks and query processing.
 */

class RagApiClient {
  /**
   * Creates a new RAG API client instance
   * @param {string} baseUrl - The base URL of the RAG backend API
   */
  constructor(baseUrl = 'http://localhost:8000') {
    this.baseUrl = baseUrl;
  }

  /**
   * Performs a health check on the RAG backend
   * @returns {Promise<Object>} Health check response
   */
  async healthCheck() {
    try {
      const response = await fetch(`${this.baseUrl}/health`, {
        method: 'GET',
        headers: {
          'Content-Type': 'application/json',
        },
      });

      if (!response.ok) {
        throw new Error(`Health check failed with status: ${response.status}`);
      }

      return await response.json();
    } catch (error) {
      console.error('Health check error:', error);
      throw error;
    }
  }

  /**
   * Queries the RAG system with general book-wide questions
   * @param {string} question - The question to ask
   * @param {string} [sessionId] - Optional session identifier
   * @param {Object} [metadata] - Optional metadata for the query
   * @returns {Promise<Object>} Query response
   */
  async query(question, sessionId = null, metadata = null) {
    const payload = {
      question: question,
    };

    if (sessionId) {
      payload.session_id = sessionId;
    }

    if (metadata) {
      payload.metadata = metadata;
    }

    try {
      const response = await fetch(`${this.baseUrl}/rag/query`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(payload),
      });

      if (!response.ok) {
        const errorData = await response.json();
        throw new Error(`Query failed: ${response.status} - ${errorData.error || 'Unknown error'}`);
      }

      return await response.json();
    } catch (error) {
      console.error('Query error:', error);
      throw error;
    }
  }

  /**
   * Queries the RAG system with selected text constraints
   * @param {string} question - The question to ask
   * @param {string} selectedText - The selected text to constrain the search to
   * @param {string} [sessionId] - Optional session identifier
   * @param {Object} [metadata] - Optional metadata for the query
   * @returns {Promise<Object>} Query response
   */
  async querySelectedText(question, selectedText, sessionId = null, metadata = null) {
    const payload = {
      question: question,
      selected_text_only: true,
      selected_text_content: selectedText,
    };

    if (sessionId) {
      payload.session_id = sessionId;
    }

    if (metadata) {
      payload.metadata = metadata;
    }

    try {
      const response = await fetch(`${this.baseUrl}/rag/query-selected-text`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(payload),
      });

      if (!response.ok) {
        const errorData = await response.json();
        throw new Error(`Selected text query failed: ${response.status} - ${errorData.error || 'Unknown error'}`);
      }

      return await response.json();
    } catch (error) {
      console.error('Selected text query error:', error);
      throw error;
    }
  }

  /**
   * Gets the API version information
   * @returns {Promise<Object>} API info response
   */
  async getApiInfo() {
    try {
      const response = await fetch(`${this.baseUrl}/`, {
        method: 'GET',
        headers: {
          'Content-Type': 'application/json',
        },
      });

      if (!response.ok) {
        throw new Error(`API info request failed with status: ${response.status}`);
      }

      return await response.json();
    } catch (error) {
      console.error('API info error:', error);
      throw error;
    }
  }
}

// Export for use in other modules (if using modules)
if (typeof module !== 'undefined' && module.exports) {
  module.exports = RagApiClient;
}

// Also make it available globally for browser usage
if (typeof window !== 'undefined') {
  window.RagApiClient = RagApiClient;
}

/**
 * Example usage:
 *
 * const client = new RagApiClient('http://localhost:8000');
 *
 * // Health check
 * client.healthCheck()
 *   .then(health => console.log('Health:', health))
 *   .catch(err => console.error('Health check failed:', err));
 *
 * // General query
 * client.query('What are the fundamental principles?')
 *   .then(response => console.log('Response:', response))
 *   .catch(err => console.error('Query failed:', err));
 *
 * // Selected text query
 * client.querySelectedText('What does this mean?', 'The fundamental principle is...')
 *   .then(response => console.log('Response:', response))
 *   .catch(err => console.error('Selected text query failed:', err));
 */