#!/usr/bin/env python3
"""
Test script to verify the RAG pipeline implementation
"""
import os
import sys
from backend.rag_pipeline.config.settings import load_config
from backend.rag_pipeline.models.data import ContentChunk, EmbeddingVector
from backend.rag_pipeline.utils.helpers import generate_content_hash
from backend.rag_pipeline.processing.cleaner import TextCleaner
from backend.rag_pipeline.processing.chunker import ContentChunker


def test_configuration():
    """Test that configuration loads correctly"""
    print("Testing configuration loading...")
    try:
        config = load_config()
        print(f"✓ Configuration loaded successfully")
        print(f"  - URLs to process: {len(config.docusaurus_urls)}")
        print(f"  - Chunk size: {config.chunk_size}")
        print(f"  - Chunk overlap: {config.chunk_overlap}")
        return True
    except Exception as e:
        print(f"✗ Configuration loading failed: {e}")
        return False


def test_data_models():
    """Test that data models work correctly"""
    print("\nTesting data models...")
    try:
        # Test ContentChunk
        chunk = ContentChunk(
            content="This is a test content chunk.",
            source_url="https://example.com/test",
            module="test_module",
            chunk_index=0
        )
        print(f"✓ ContentChunk created successfully with ID: {chunk.id[:10]}...")

        # Test EmbeddingVector
        embedding = EmbeddingVector(
            chunk_id=chunk.id,
            vector=[0.1, 0.2, 0.3, 0.4, 0.5],
            model_name="test-model"
        )
        print(f"✓ EmbeddingVector created successfully")

        return True
    except Exception as e:
        print(f"✗ Data model test failed: {e}")
        return False


def test_text_cleaning():
    """Test text cleaning functionality"""
    print("\nTesting text cleaning...")
    try:
        cleaner = TextCleaner()

        # Test with messy text
        messy_text = "  This   text   has   extra   spaces.\n\nAnd\nnewlines.  "
        cleaned = cleaner.clean_and_normalize(messy_text)

        print(f"Original: '{messy_text}'")
        print(f"Cleaned:  '{cleaned}'")

        # Check that it's cleaner
        assert '   ' not in cleaned, "Extra spaces not removed"
        assert '\n\n\n' not in cleaned, "Extra newlines not removed"

        print("✓ Text cleaning works correctly")
        return True
    except Exception as e:
        print(f"✗ Text cleaning test failed: {e}")
        return False


def test_content_chunking():
    """Test content chunking functionality"""
    print("\nTesting content chunking...")
    try:
        chunker = ContentChunker(max_chunk_size=50, overlap=10)

        # Test with longer text
        long_text = "This is a sentence. " * 20  # Creates a longer text
        chunks = chunker.chunk_content(long_text, source_url="https://example.com/test", module="test")

        print(f"Created {len(chunks)} chunks from text")
        for i, chunk in enumerate(chunks[:3]):  # Show first 3 chunks
            print(f"  Chunk {i}: {len(chunk.content)} chars")

        # Verify chunks are within size limits
        for chunk in chunks:
            assert len(chunk.content) <= 50, f"Chunk {chunk.chunk_index} exceeds size limit"

        print("✓ Content chunking works correctly")
        return True
    except Exception as e:
        print(f"✗ Content chunking test failed: {e}")
        return False


def test_content_hashing():
    """Test content hashing for idempotency"""
    print("\nTesting content hashing...")
    try:
        text1 = "This is some content."
        text2 = "This is some content."
        text3 = "This is different content."

        hash1 = generate_content_hash(text1)
        hash2 = generate_content_hash(text2)
        hash3 = generate_content_hash(text3)

        assert hash1 == hash2, "Same content should have same hash"
        assert hash1 != hash3, "Different content should have different hash"

        print("✓ Content hashing works correctly")
        return True
    except Exception as e:
        print(f"✗ Content hashing test failed: {e}")
        return False


def main():
    """Run all tests"""
    print("Running RAG Pipeline Implementation Tests\n")
    print("=" * 50)

    tests = [
        test_configuration,
        test_data_models,
        test_text_cleaning,
        test_content_chunking,
        test_content_hashing
    ]

    passed = 0
    total = len(tests)

    for test_func in tests:
        if test_func():
            passed += 1

    print("\n" + "=" * 50)
    print(f"Tests completed: {passed}/{total} passed")

    if passed == total:
        print("🎉 All tests passed! Implementation is working correctly.")
        return 0
    else:
        print(f"❌ {total - passed} test(s) failed.")
        return 1


if __name__ == "__main__":
    sys.exit(main())