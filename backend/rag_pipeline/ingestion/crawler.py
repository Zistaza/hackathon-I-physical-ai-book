import requests
from bs4 import BeautifulSoup
from typing import List, Optional
from urllib.parse import urljoin, urlparse
import time
from backend.rag_pipeline.utils.helpers import retry_on_failure


class DocusaurusCrawler:
    """
    Crawler for Docusaurus websites that fetches HTML content from URLs
    """
    def __init__(self, session: Optional[requests.Session] = None, timeout: int = 30):
        """
        Initialize the crawler

        Args:
            session: Optional requests session to reuse connections
            timeout: Request timeout in seconds
        """
        self.session = session or requests.Session()
        self.timeout = timeout
        self.headers = {
            'User-Agent': 'Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/91.0.4472.124 Safari/537.36'
        }

    @retry_on_failure(max_retries=3, delay=1.0, backoff=2.0)
    def fetch_page(self, url: str) -> str:
        """
        Fetch HTML content from a single URL

        Args:
            url: URL to fetch

        Returns:
            HTML content of the page

        Raises:
            requests.RequestException: If the request fails after retries
        """
        response = self.session.get(url, headers=self.headers, timeout=self.timeout)
        response.raise_for_status()  # Raise an exception for bad status codes
        response.encoding = response.apparent_encoding  # Handle encoding properly
        return response.text

    def fetch_all_pages(self, urls: List[str]) -> List[tuple[str, str, Optional[Exception]]]:
        """
        Fetch content from multiple URLs

        Args:
            urls: List of URLs to fetch

        Returns:
            List of tuples containing (url, content, error) for each URL
        """
        results = []

        for url in urls:
            try:
                content = self.fetch_page(url)
                results.append((url, content, None))
            except Exception as e:
                # Return the URL, empty content string, and the error
                results.append((url, "", e))
                print(f"Error fetching {url}: {e}")

        return results

    def close(self):
        """
        Close the session to free up resources
        """
        if self.session:
            self.session.close()

    def __enter__(self):
        """
        Context manager entry
        """
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """
        Context manager exit - close session
        """
        self.close()