import hashlib
import time
from typing import Callable, Any
from functools import wraps


def retry_on_failure(max_retries: int = 3, delay: float = 1.0, backoff: float = 2.0):
    """
    Decorator to retry a function on failure

    Args:
        max_retries: Maximum number of retry attempts
        delay: Initial delay between retries in seconds
        backoff: Multiplier for delay after each retry
    """
    def decorator(func: Callable) -> Callable:
        @wraps(func)
        def wrapper(*args, **kwargs) -> Any:
            current_delay = delay
            last_exception = None

            for attempt in range(max_retries + 1):
                try:
                    return func(*args, **kwargs)
                except Exception as e:
                    last_exception = e
                    if attempt == max_retries:
                        # Exhausted all retries, raise the last exception
                        raise last_exception

                    # Wait before retrying
                    time.sleep(current_delay)
                    current_delay *= backoff

            # This should never be reached, but added for type safety
            raise last_exception
        return wrapper
    return decorator


def generate_content_hash(content: str) -> str:
    """
    Generate a hash of the content for idempotency checks

    Args:
        content: The content to hash

    Returns:
        SHA-256 hash of the content as a hex string
    """
    return hashlib.sha256(content.encode()).hexdigest()


def generate_chunk_id(source_url: str, content: str, chunk_index: int) -> str:
    """
    Generate a unique chunk ID based on source URL, content, and chunk index

    Args:
        source_url: The source URL of the content
        content: The content text
        chunk_index: The index of this chunk within the document

    Returns:
        Unique identifier for the chunk
    """
    content_to_hash = f"{source_url}{content}{chunk_index}"
    return hashlib.sha256(content_to_hash.encode()).hexdigest()


def clean_text(text: str) -> str:
    """
    Clean text by removing extra whitespace and normalizing

    Args:
        text: The text to clean

    Returns:
        Cleaned text
    """
    if not text:
        return ""

    # Replace multiple whitespace with single space
    import re
    cleaned = re.sub(r'\s+', ' ', text)

    # Strip leading/trailing whitespace
    cleaned = cleaned.strip()

    return cleaned


def format_elapsed_time(seconds: float) -> str:
    """
    Format elapsed time in a human-readable format

    Args:
        seconds: Time in seconds

    Returns:
        Formatted time string
    """
    if seconds < 1:
        return f"{seconds*1000:.2f}ms"
    elif seconds < 60:
        return f"{seconds:.2f}s"
    elif seconds < 3600:
        minutes = seconds // 60
        remaining_seconds = seconds % 60
        return f"{int(minutes)}m {remaining_seconds:.2f}s"
    else:
        hours = seconds // 3600
        remaining_minutes = (seconds % 3600) // 60
        return f"{int(hours)}h {int(remaining_minutes)}m"


def batch_list(lst: list, batch_size: int) -> list:
    """
    Split a list into batches of specified size

    Args:
        lst: List to batch
        batch_size: Size of each batch

    Returns:
        List of batches
    """
    if not lst:
        return []

    batches = []
    for i in range(0, len(lst), batch_size):
        batches.append(lst[i:i + batch_size])

    return batches


def validate_url(url: str) -> bool:
    """
    Validate if a string is a properly formatted URL

    Args:
        url: URL string to validate

    Returns:
        True if valid, False otherwise
    """
    import re
    url_pattern = re.compile(
        r'^https?://'  # http:// or https://
        r'(?:(?:[A-Z0-9](?:[A-Z0-9-]{0,61}[A-Z0-9])?\.)+[A-Z]{2,6}\.?|'  # domain...
        r'localhost|'  # localhost...
        r'\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3})'  # ...or ip
        r'(?::\d+)?'  # optional port
        r'(?:/?|[/?]\S+)$', re.IGNORECASE)
    return bool(url_pattern.match(url))