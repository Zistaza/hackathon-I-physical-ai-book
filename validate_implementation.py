#!/usr/bin/env python3
"""
Quick validation script to verify that all required files were created
"""
import os
from pathlib import Path


def validate_file_exists(filepath, description=""):
    """Check if a file exists and print status"""
    path = Path(filepath)
    exists = path.exists()
    status = "✓" if exists else "✗"
    print(f"{status} {description}: {filepath}")
    return exists


def validate_directory_exists(dirpath, description=""):
    """Check if a directory exists and print status"""
    path = Path(dirpath)
    exists = path.is_dir()
    status = "✓" if exists else "✗"
    print(f"{status} {description}: {dirpath}")
    return exists


def main():
    print("Validating RAG Pipeline Implementation\n")
    print("=" * 50)

    # Validate project structure
    directories = [
        ("backend", "Main backend directory"),
        ("backend/rag_pipeline", "Main pipeline package"),
        ("backend/rag_pipeline/config", "Configuration module"),
        ("backend/rag_pipeline/ingestion", "Ingestion module"),
        ("backend/rag_pipeline/processing", "Processing module"),
        ("backend/rag_pipeline/embedding", "Embedding module"),
        ("backend/rag_pipeline/storage", "Storage module"),
        ("backend/rag_pipeline/models", "Data models module"),
        ("backend/rag_pipeline/utils", "Utilities module"),
    ]

    files = [
        ("backend/requirements.txt", "Requirements file"),
        ("backend/pyproject.toml", "Project configuration"),
        ("backend/.env.example", "Environment variables example"),
        ("backend/rag_pipeline/__init__.py", "Package init file"),
        ("backend/rag_pipeline/config/__init__.py", "Config package init"),
        ("backend/rag_pipeline/ingestion/__init__.py", "Ingestion package init"),
        ("backend/rag_pipeline/processing/__init__.py", "Processing package init"),
        ("backend/rag_pipeline/embedding/__init__.py", "Embedding package init"),
        ("backend/rag_pipeline/storage/__init__.py", "Storage package init"),
        ("backend/rag_pipeline/models/__init__.py", "Models package init"),
        ("backend/rag_pipeline/utils/__init__.py", "Utils package init"),
        ("backend/rag_pipeline/config/settings.py", "Settings module"),
        ("backend/rag_pipeline/models/data.py", "Data models"),
        ("backend/rag_pipeline/utils/helpers.py", "Helper functions"),
        ("backend/rag_pipeline/ingestion/crawler.py", "Crawler module"),
        ("backend/rag_pipeline/ingestion/parser.py", "Parser module"),
        ("backend/rag_pipeline/processing/cleaner.py", "Cleaner module"),
        ("backend/rag_pipeline/processing/chunker.py", "Chunker module"),
        ("backend/rag_pipeline/embedding/generator.py", "Embedding generator"),
        ("backend/rag_pipeline/storage/vector_db.py", "Vector storage"),
        ("backend/rag_pipeline/main.py", "Main pipeline orchestrator"),
    ]

    # Validate directories
    print("\nValidating directories:")
    dir_success = 0
    total_dirs = len(directories)
    for dir_path, description in directories:
        if validate_directory_exists(dir_path, description):
            dir_success += 1

    # Validate files
    print("\nValidating files:")
    file_success = 0
    total_files = len(files)
    for file_path, description in files:
        if validate_file_exists(file_path, description):
            file_success += 1

    print(f"\n" + "=" * 50)
    print(f"Directories: {dir_success}/{total_dirs} created")
    print(f"Files: {file_success}/{total_files} created")

    if dir_success == total_dirs and file_success == total_files:
        print("🎉 All required components created successfully!")
        return 0
    else:
        print("❌ Some components are missing.")
        return 1


if __name__ == "__main__":
    exit(main())