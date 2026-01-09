#!/usr/bin/env python3
"""
Debug the run failure by examining the run object more closely
"""
import os
import sys
import json
import time
from backend.agent import RAGAgent, UserQuery

def debug_run_failure():
    """Debug the run failure"""

    print("Creating agent...")
    agent = RAGAgent()

    print(f"Using model: {agent.assistant.model}")
    print(f"Assistant created with ID: {agent.assistant.id}")

    # Create a simple query
    user_query = UserQuery(
        id="debug_query_1",
        text="What is this book about?",
        selected_text_only=False
    )

    print(f"Processing query: {user_query.text}")

    try:
        # Create a thread for the conversation
        thread = agent.client.beta.threads.create()
        print(f"Thread created: {thread.id}")

        # Add a user message to the thread
        message = agent.client.beta.threads.messages.create(
            thread_id=thread.id,
            role="user",
            content=user_query.text
        )
        print(f"Message added: {message.id}")

        # Run the assistant
        run = agent.client.beta.threads.runs.create(
            thread_id=thread.id,
            assistant_id=agent.assistant.id
        )
        print(f"Run started: {run.id}, status: {run.status}")

        # Poll for completion and check for tool calls
        step_count = 0
        while run.status in ["queued", "in_progress"]:
            time.sleep(1)
            run = agent.client.beta.threads.runs.retrieve(thread_id=thread.id, run_id=run.id)
            print(f"Run status: {run.status}")

            if step_count > 20:  # Prevent infinite loop
                print("Breaking to prevent infinite loop")
                break
            step_count += 1

        print(f"Run completed with status: {run.status}")

        # Check for failure details
        if run.status == "failed":
            print(f"Run failed! Last error: {run.last_error}")
            if hasattr(run, 'incomplete_details'):
                print(f"Incomplete details: {run.incomplete_details}")

        print(f"Run usage: {getattr(run, 'usage', 'Not available')}")

        # Check if the run requires tool calls (retrieval)
        print(f"Required action: {getattr(run, 'required_action', 'None')}")
        if hasattr(run, 'required_action') and run.required_action:
            print(f"Tool calls required: {run.required_action.submit_tool_outputs.tool_calls}")

        # Get the messages
        messages = agent.client.beta.threads.messages.list(thread_id=thread.id)
        print(f"Found {len(messages.data)} messages in thread")

        for msg in messages.data:
            print(f"Message role: {msg.role}, content: {len(msg.content)} parts")
            for content_block in msg.content:
                if hasattr(content_block, 'text') and content_block.text:
                    print(f"  Content: '{content_block.text.value[:200]}...'")

    except Exception as e:
        print(f"Error in debug: {e}")
        import traceback
        traceback.print_exc()

    finally:
        # Clean up
        try:
            agent.client.beta.assistants.delete(agent.assistant.id)
        except:
            pass

if __name__ == "__main__":
    debug_run_failure()