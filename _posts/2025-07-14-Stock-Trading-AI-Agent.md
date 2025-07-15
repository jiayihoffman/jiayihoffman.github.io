---
layout: post
title: "Stock Trading AI Agent"
date: 2025-07-14 08:45:28 -0600
categories: AI_Agent
#image: /assets/media_server/IMG_3341.jpeg
---
I enjoy trading stocks for long-term investing because it requires knowledge, analytical skills, and a long-term perspective. Over time, I have developed technical skills in analyzing stock charts to identify patterns, resistance and support levels, and to gauge whether the stock is turning bearish or bullish. I also like reading news and listening to tech podcasts to better understand the world around me and the upcoming trends. 

## Stock Trading App
That said, I don't always have time to watch stock charts live. That makes me want to build a stock trading app where AI agents analyze the charts and correlate them with different data to make trading decisions.  

Additionally, in many ways, trading stocks is more of a psychological game than a numbers game. Therefore, using AI agents to trade stocks is helpful during major pullbacks when everyone is scared or at market peaks when everyone is greedy. AI has a much calmer mind than we humans. :)

With the app, I have a few stocks to start with, but I’d love to add more to the list as the AI agent recommends new stocks in sectors I’m interested in.

### Architecture
Here’s a brief overview of the architecture. The benefit of agent-based design is that:
1. It is modular, flexible, and with human-like intelligence. 
2. This design supports the "in the loop" evaluation, enabling the Orchestrator agent to critique the output of other agents to promote reflection, which is a key aspect of the agentic reasoning pattern that improves performance. 
3. Additionally, the logging agent records the trading results to facilitate ongoing iterative review and improvement. 

<a href="/assets/ai_agent/flow_chart.png" target="_blank">
  <img src="/assets/ai_agent/flow_chart.png" />
</a>

* The Orchestrator Agent triggers the flow (e.g., on schedule or event).
* It asks the Stock Agent for the current stock list.
* It tells the Market Data Agent and News Agent to fetch data for those stocks.
* The Technical Analysis Agent and Sentiment Agent process their respective data.
* The Trading Rules Agent combines all signals and makes a recommendation.
* The Orchestrator Agent decides whether to act, and if so, instructs the Execution Agent.
* All actions and decisions are logged by the Logging Agent.

### Technologies
I use [Alpaca Brokerage](https://alpaca.markets/), which provides a commission-free, API-first platform for trading. It offers free real-time and historical US stock data via API. Additionally, it supports paper trading for safe testing.  

I use ChatGPT-4o as the LLM, and LangChain and tools as the agent framework. The app is written in Python. 

To be continued... 