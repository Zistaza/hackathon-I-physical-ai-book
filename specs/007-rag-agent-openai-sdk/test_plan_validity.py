def test_plan_validity():
    """Test that the implementation plan is valid and complete."""
    import os

    # Check that all required files exist
    plan_file = "/home/emizee/hackathon-I-physical-ai-book/specs/007-rag-agent-openai-sdk/plan.md"
    research_file = "/home/emizee/hackathon-I-physical-ai-book/specs/007-rag-agent-openai-sdk/research.md"
    data_model_file = "/home/emizee/hackathon-I-physical-ai-book/specs/007-rag-agent-openai-sdk/data-model.md"
    quickstart_file = "/home/emizee/hackathon-I-physical-ai-book/specs/007-rag-agent-openai-sdk/quickstart.md"
    contract_file = "/home/emizee/hackathon-I-physical-ai-book/specs/007-rag-agent-openai-sdk/contracts/rag-agent-contracts.md"

    assert os.path.exists(plan_file), "Plan file does not exist"
    assert os.path.exists(research_file), "Research file does not exist"
    assert os.path.exists(data_model_file), "Data model file does not exist"
    assert os.path.exists(quickstart_file), "Quickstart file does not exist"
    assert os.path.exists(contract_file), "Contract file does not exist"

    # Check that plan file contains required sections
    with open(plan_file, 'r') as f:
        content = f.read()

    assert "Architecture Sketch" in content, "Architecture section missing"
    assert "Phase Structure" in content, "Phase structure section missing"
    assert "Decisions Requiring Documentation" in content, "Decisions section missing"
    assert "Testing & Validation Strategy" in content, "Testing strategy section missing"

    print("All plan validity checks passed!")

if __name__ == "__main__":
    test_plan_validity()