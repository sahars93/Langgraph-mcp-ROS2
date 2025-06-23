import asyncio
from mcp import ClientSession, StdioServerParameters
from mcp.client.stdio import stdio_client
from langchain_mcp_adapters.tools import load_mcp_tools
from langgraph.prebuilt import create_react_agent
from langchain_ollama import ChatOllama
from mcp.server.fastmcp import FastMCP
import asyncio
import threading


model = ChatOllama(
    model="llama3.1:70b",
    base_url="http://IP:11434",   
    temperature=0,
    num_ctx=8192,  
    )

async def main():
    server_params = StdioServerParameters(
    command="bash",
    args=[
        "-c",
        "source /opt/ros/humble/setup.bash && python3 new_server.py"
    ],
    )
    async with stdio_client(server_params) as (read, write):
        async with ClientSession(read, write) as session:
            await session.initialize()
            tools = await load_mcp_tools(session)
            agent = create_react_agent(model, tools)

            # Try out the tools via natural language
            user_input = input("Hi! I am the ROS2 agent. How can i help you? :) \n \n Enter:")
            while user_input != "exit":
                msg1 = {"messages": [("user", user_input)]}
                res1 = await agent.ainvoke(msg1)
                for m in res1['messages']:
                    m.pretty_print()
                user_input = input("Enter: ")

if __name__ == "__main__":
    asyncio.run(main())