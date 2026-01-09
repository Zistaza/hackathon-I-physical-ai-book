def test_adr_validity():
    """Test that the ADRs are valid and complete."""
    import os

    # Check that all required ADR files exist
    adr_files = [
        "/home/emizee/hackathon-I-physical-ai-book/history/adr/0003-rag-agent-technology-stack.md",
        "/home/emizee/hackathon-I-physical-ai-book/history/adr/0004-rag-agent-architecture-pattern.md",
        "/home/emizee/hackathon-I-physical-ai-book/history/adr/0005-rag-agent-determinism-strategy.md",
        "/home/emizee/hackathon-I-physical-ai-book/history/adr/0006-rag-agent-retrieval-integration.md",
        "/home/emizee/hackathon-I-physical-ai-book/history/adr/0007-rag-agent-response-validation.md"
    ]

    for adr_file in adr_files:
        assert os.path.exists(adr_file), f"ADR file does not exist: {adr_file}"

    # Check that each ADR contains required sections
    for adr_file in adr_files:
        with open(adr_file, 'r') as f:
            content = f.read()

        assert "## Decision" in content, f"Decision section missing in {adr_file}"
        assert "## Consequences" in content, f"Consequences section missing in {adr_file}"
        assert "## Alternatives Considered" in content, f"Alternatives section missing in {adr_file}"
        assert "## References" in content, f"References section missing in {adr_file}"

    print("All ADR validity checks passed!")

if __name__ == "__main__":
    test_adr_validity()