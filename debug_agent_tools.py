#!/usr/bin/env python3
"""
Debug the agent's tool usage by examining the run steps in detail
"""
import os
import sys
import json
import time
from backend.agent import RAGAgent, UserQuery

def debug_tool_usage():
    """Debug the agent's tool usage by examining run steps"""

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

        # Check if the run requires tool calls (retrieval)
        print(f"Required action: {run.required_action}")
        if run.required_action:
            print(f"Tool calls required: {run.required_action.submit_tool_outputs.tool_calls}")

            tool_calls = run.required_action.submit_tool_outputs.tool_calls
            tool_outputs = []

            for tool_call in tool_calls:
                print(f"Processing tool call: {tool_call.id}, function: {tool_call.function.name}")

                if tool_call.function.name == "retrieve_content":
                    # Parse the arguments
                    try:
                        args = json.loads(tool_call.function.arguments)
                        print(f"Parsed arguments: {args}")

                        # Call the retrieval function
                        result = agent.retriever.retrieve_content(
                            query=args.get('query', ''),
                            top_k=args.get('top_k', 5),
                            selected_text_only=args.get('selected_text_only', False),
                            selected_text_content=args.get('selected_text_content')
                        )
                        print(f"Retrieval result: {len(result.get('chunks', []))} chunks found")

                        # Format the result for the assistant
                        output = json.dumps(result)
                        tool_outputs.append({
                            "tool_call_id": tool_call.id,
                            "output": output
                        })
                    except json.JSONDecodeError as e:
                        print(f"Error parsing tool call arguments: {e}")
                        continue
                else:
                    print(f"Unknown tool: {tool_call.function.name}")

            # Submit the tool outputs if any were created
            if tool_outputs:
                print(f"Submitting {len(tool_outputs)} tool outputs")
                run = agent.client.beta.threads.runs.submit_tool_outputs(
                    thread_id=thread.id,
                    run_id=run.id,
                    tool_outputs=tool_outputs
                )

                # Wait for the run to complete after tool submission
                step_count = 0
                while run.status in ["queued", "in_progress"]:
                    time.sleep(1)
                    run = agent.client.beta.threads.runs.retrieve(thread_id=thread.id, run_id=run.id)
                    print(f"After tool submission - Run status: {run.status}")

                    if step_count > 20:  # Prevent infinite loop
                        print("Breaking to prevent infinite loop after tool submission")
                        break
                    step_count += 1

        # Get the messages
        messages = agent.client.beta.threads.messages.list(thread_id=thread.id)

        print(f"Found {len(messages.data)} messages in thread")

        # Extract the assistant's response
        response_content = ""
        for msg in messages.data:
            print(f"Message role: {msg.role}, content: {len(msg.content)} parts")
            if msg.role == "assistant":
                # Handle potential multiple content blocks
                for content_block in msg.content:
                    if hasattr(content_block, 'text') and content_block.text:
                        response_content = content_block.text.value
                        break
                break

        print(f"Assistant response: '{response_content[:200]}...' (truncated)")

        # Look for tool call results in the run steps
        print("\nExamining run steps...")
        run_steps = agent.client.beta.threads.runs.steps.list(
            thread_id=thread.id,
            run_id=run.id
        )

        for step in run_steps.data:
            print(f"Step type: {step.type}")
            if step.type == "tool_calls":
                print(f"Tool calls in step: {len(step.step_details.tool_calls)}")
                for tool_call in step.step_details.tool_calls:
                    print(f"  Tool: {tool_call.type}, {tool_call.function.name if hasattr(tool_call, 'function') else 'N/A'}")
                    if hasattr(tool_call, 'function') and hasattr(tool_call.function, 'output'):
                        print(f"    Output preview: {tool_call.function.output[:200] if tool_call.function.output else 'None'}...")

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
    debug_tool_usage()