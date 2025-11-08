---
layout: post
title: "Stock Analyzer AI Agent"
date: 2025-07-14 08:45:28 -0600
categories: AI_Agent
image: /assets/ai_agent/pltr_stock_chart.png
---
I enjoy trading stocks for long-term investing because it requires knowledge, analytical skills, and a long-term perspective. Over time, I have developed technical skills in analyzing stock charts to identify patterns, resistance and support levels, and to gauge whether the stock is becoming bearish or bullish. I also enjoy reading news and listening to tech podcasts to stay up-to-date on upcoming trends.

In addition to personal interests, in the era of AGI, wealth is predominantly generated through equity market investments. 

## A Stock AI Agent
That said, I don't always have time to watch live stock charts. That makes me want to create a stock app where AI agents analyze the charts and relate them to market sentiment to give trading insights.  

Additionally, in many ways, trading stocks is more of a psychological game than a numbers game. Therefore, using AI agents to recommend stock actions can be helpful during major pullbacks when everyone is scared, as well as at market peaks when everyone is greedy. AI has a much calmer mind than we humans. :)

I have a few stocks to start with, but I’d love to add more to the list as the AI agent researches and recommends new stocks in sectors that interest me.

### Architecture
Here’s a brief overview of the architecture. The benefit of agent-based design is that:
1. It is modular, flexible, and with expert stock analyst intelligence. 
2. This design supports the "in the loop" evaluation, enabling the Orchestrator agent to oversee and critique the output of other agents, promoting reflection - a key aspect of the agentic reasoning pattern that helps improve performance. 
3. Additionally, the logging agent records the stock's insights and feedback to facilitate ongoing iterative review and improvement.

<a href="/assets/ai_agent/stock_analyzer_components.drawio.png" target="_blank">
  <img src="/assets/ai_agent/stock_analyzer_components.drawio.png" />
</a>

Here are the responsibilities of each agent: 

* The Orchestrator Agent initiates the flow (e.g., on schedule or event).
* It instructs the Technical Analysis Agent and Sentiment Agent to analyze the stocks.
* The Technical Agent contacts the Market Data Agent for stock data, and the Sentiment Agent uses the News Agent to fetch news data for the stocks.
* The Orchestrator Agent combines all signals and makes a recommendation. 
* All insights are logged by the Logging Agent.

The word “agent” is a general term here. It can refer to a large language model (LLM) or an entity that performs specific tasks. For example:
* The “Technical Analysis Agent” is a large language model. It calculates various technical indicators for stocks and uses reasoning to provide a technical assessment.
* Conversely, the “Market Data Agent” retrieves stock data from the stock exchange.

### Technologies
I use ChatGPT-5 as the large language model (LLM) and LangGraph as the agent framework. The AI agent is built with Python and Angular. The web user interface is developed entirely through Cursor, an AI-assisted coding tool.

The agent functions as a REST service with endpoints for on-demand stock analysis requests. It also runs the "watchlist" job in the background to automatically generate insightful alerts for the user.

<!-- Additionally, it has the "Trade Monitoring" job running in the background by the orchestrator to oversee the stocks. Just like a real Wall Street trader, the job uses a scanner that constantly monitors the list of stocks and only drills down when the technicals indicate a potential move worth trading. -->

<!-- #### LangGraph
Here is the generated LangGraph illustrating the agent workflow where the task is divided into fixed subtasks for greater accuracy and predictability due to the nature of the use case. The workflow includes a human approval step to review and authorize trade execution. 

<a href="/assets/ai_agent/agent_workflow_graph.png" target="_blank">
  <img src="/assets/ai_agent/agent_workflow_graph.png" width="500"/>

The graph describes nodes involved in Prompt Chaining, where each agent node processes the output of the previous one. Here is the code for chaining the nodes. 

```
def build_state_graph(self):
    workflow = StateGraph(State)

    # Add nodes
    workflow.add_node("fetch_market_data", self.fetch_market_data)
    workflow.add_node("check_technical_indicators", self.check_technical_indicators)
    workflow.add_node("technical_analysis", self.technical_analysis)
    ...

    # Add edges to connect nodes
        workflow.add_edge(START, "fetch_market_data")
        workflow.add_edge("fetch_market_data", "check_technical_indicators")
        workflow.add_conditional_edges(
            "check_technical_indicators", self.check_should_analyze, {True: "technical_analysis", False: END}
        )
    ...

    # Compile
    memory = MemorySaver()
    graph = workflow.compile(interrupt_before=["user_trade_approval"], checkpointer=memory)
    return graph
``` -->

### The Stock Analyzer AI Agent
Here is the Stock Analyzer, the AI agent that offers professional stock technical insights and market sentiment analysis.

<!-- The Stock Analyzer app is available at: [https://stock-analyzer.modularmachines.ai](https://stock-analyzer.modularmachines.ai/). It is access-controlled. If you'd like to try it, let me know, and I will create an account for you. You can reset your password once you log in.

Here are the screenshots of the product: -->

<a href="/assets/ai_agent/oklo_screenshot.png" target="_blank">
  <img src="/assets/ai_agent/oklo_screenshot.png" />

In addition to "Quick Insights," the user can also chat with the agent about specific stock topics. Unlike a generic chatbot, the agent has access to the stock's charts and the latest news, so it can provide the user with solid data points for quick decision-making.

<a href="/assets/ai_agent/T_response1.png" target="_blank">
  <img src="/assets/ai_agent/T_response1.png" />

<a href="/assets/ai_agent/T_response2.png" target="_blank">
  <img src="/assets/ai_agent/T_response2.png" />


### Testimony

#### Palantir (PLTR)

Two days before Palantir's earnings report, the Stock Analyzer's insights recommended holding the position and not trimming it. It also noted that news headlines reinforced the technical outlook and supported a bullish run. After the earnings report, PLTR stock surged by 22%, from $154 to $188.

Then, on Friday, August 08, the Analyzer recommended that I trim slightly and take some profits because the stock was strongly overbought. One week later, the stock dropped 6% to $177. Ten days later, on August 19, PLTR sits at $157, 16% down from the price point where the Stock Analyzer recommended trimming.

<a href="/assets/ai_agent/pltr_stock_chart.png" target="_blank">
  <img src="/assets/ai_agent/pltr_stock_chart.png" />

<a href="/assets/ai_agent/pltr_analysis_8-3.png" target="_blank">
  <img src="/assets/ai_agent/pltr_analysis_8-3.png" />

<a href="/assets/ai_agent/pltr_analysis_8-8.png" target="_blank">
  <img src="/assets/ai_agent/pltr_analysis_8-8.png" />

#### Oklo Inc. (OKLO)

On September 25, the Stock Analyzer Agent recommended that I trim my OKLO position modestly at $131, as the OKLO chart indicates a loss of momentum and signals a potential distribution following the big run.

This proved to be a very helpful tip. The OKLO stock fell more than $21, closing at $110 over the next two trading days. Additionally, CNBC later reported that "[Oklo has also seen a cluster of insider selling over the past few days](https://www.cnbc.com/2025/09/25/oklo-nuclear-shares-fall-ai-data-center.html)", which matches my AI agent's speculation based on the chart. 

<a href="/assets/ai_agent/oklo_stock_chart.png" target="_blank">
  <img src="/assets/ai_agent/oklo_stock_chart.png" />

<a href="/assets/ai_agent/oklo_analysis_9-25.png" target="_blank">
  <img src="/assets/ai_agent/oklo_analysis_9-25.png" />


Cheers!