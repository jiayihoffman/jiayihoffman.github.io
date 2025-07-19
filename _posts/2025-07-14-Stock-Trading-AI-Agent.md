---
layout: post
title: "Stock Trading AI Agent"
date: 2025-07-14 08:45:28 -0600
categories: AI_Agent
#image: /assets/ai_agent/agent_workflow_graph.png
---
I enjoy trading stocks for long-term investing because it requires knowledge, analytical skills, and a long-term perspective. Over time, I have developed technical skills in analyzing stock charts to identify patterns, resistance and support levels, and to gauge whether the stock is becoming bearish or bullish. I also enjoy reading news and listening to tech podcasts to better understand the world around me and upcoming trends.

## Stock Trading App
That said, I don't always have time to watch live stock charts. That makes me want to build a stock trading app where AI agents analyze the charts and correlate them with different data to make trading decisions.  

Additionally, in many ways, trading stocks is more of a psychological game than a numbers game. Therefore, using AI agents to trade stocks is helpful during major pullbacks when everyone is scared or at market peaks when everyone is greedy. AI has a much calmer mind than we humans. :)

With the app, I have a few stocks to start with, but I’d love to add more to the list as the AI agent recommends new stocks in sectors I’m interested in.

### Architecture
Here’s a brief overview of the architecture. The benefit of agent-based design is that:
1. It is modular, flexible, and with expert stock analyst intelligence. 
2. This design supports the "in the loop" evaluation, enabling the Orchestrator agent to critique the output of other agents to promote reflection, which is a key aspect of the agentic reasoning pattern that help improve performance. 
3. Additionally, the logging agent records the trading results to facilitate ongoing iterative review and improvement. 

<a href="/assets/ai_agent/component_diagram.png" target="_blank">
  <img src="/assets/ai_agent/component_diagram.png" />
</a>

Here is the responsible of each agents. 

* The Orchestrator Agent triggers the flow (e.g., on schedule or event).
* It asks the Stock Picker Agent for the current stock list.
* It tells the Market Data Agent and News Agent to fetch data for those stocks.
* The Technical Analysis Agent and Sentiment Agent process their respective data.
* The Trading Rules Agent combines all signals and makes a recommendation.
* The Orchestrator Agent decides whether to act, and if so, instructs the Execution Agent.
* All actions and decisions are logged by the Logging Agent.

The word “agent” here is a general term. It can be an LLM or an entity that completes specific tasks. For example:
* The “Technical Analysis Agent” is an LLM. It computes the stock’s various technical indicators and uses reasoning to give the stock a technical overview.
* On the other hand, the “Market Data Agent” fetches the stock data from the stock exchange.

### Technologies
I use ChatGPT-4o as the LLM and LangGraph as the agent framework. The app is written in Python.

The app runs as a REST service with endpoints to support on-demand stock analysis requests. 

Additionally, it has the "Trade Monitoring" job running in the background by the orchestrator to oversee the stocks. Just like a real Wall Street trader, the job uses a scanner that constantly monitors the list of stocks and only drills down when the technicals indicate a potential move worth trading.

#### LangGraph
Here is the generated LangGraph representing the agent workflow where the task is broken down into fixed subtasks for higher accuracy and predictability due to the nature of the use case. Within the workflow, there is a human approval task to review and grant permission for the trade execution. 

<a href="/assets/ai_agent/agent_workflow_graph.png" target="_blank">
  <img src="/assets/ai_agent/agent_workflow_graph.png" width="500"/>
</a>

The graph defines nodes involved in Prompt chaining, where each agent node handles the output of the previous one. Here is the code of chaining the nodes. 
```
def build_state_graph(self):
    workflow = StateGraph(State)

    # Add nodes
    workflow.add_node("fetch_market_data", self.fetch_market_data)
    workflow.add_node("check_triggers", self.check_triggers)
    workflow.add_node("technical_analysis", self.technical_analysis)
    ...

    # Add edges to connect nodes
    workflow.add_edge(START, "fetch_market_data")
    workflow.add_edge("fetch_market_data", "check_triggers")
    workflow.add_conditional_edges(
        "check_triggers", self.check_should_analyze, {True: "technical_analysis", False: END}
    )
    ...

    # Compile
    memory = MemorySaver()
    graph = workflow.compile(interrupt_before=["user_trade_approval"], checkpointer=memory)
    return graph
```

### Demonstration
#### NFLX
On July 18, **Netflix** stock dropped 5% after a strong earnings report, and the large price swing triggered sentiment analysis of the stock. Here is the response from my Stock Trading Agent:
```
Sentiment for NFLX News: MIXED (Positive Bias)

Key headlines:
* Earnings beat but weak reaction: Strong Q2 results but “failed to impress” — short-term cautious tone.
* Stock drop: Shares fell 5% despite the beat — short-term negative reaction.
* Analyst action: Many major banks (Wells Fargo, UBS, JP Morgan, TD Cowen, Rosenblatt, Piper Sandler, Morgan Stanley) all maintain Buy/Overweight and raise price targets — strong medium-term positive signal.
* Engagement mixed: “Anemic” engagement but “shockingly strong” retention — mixed narrative, mostly neutralized by higher subscriber stickiness.
* Options activity: Covered call suggests cautious big money hedging.

Implication:
* Short-term sentiment is slightly negative due to the price drop and cautious reaction.
* Medium-term sentiment is positive because multiple analysts raised price targets significantly, reinforcing fundamental confidence.
* This offsets technical weakness (price stalling, slight profit-taking) but does not fully reverse it yet.

Sentiment: MIXED leaning POSITIVE — News flow moderately strengthens the medium-term technical signal but near-term profit-taking risk remains.
```

#### AVGO

The Stock Trading Agent's technical analysis of **Broadcom** stock on July 18:

```
Analysis for AVGO:
* Trend: Strong upward trend — current price (283.27) is well above all key SMAs (20, 50, 150) and EMA 20, showing clear bullish momentum.
* RSI: Elevated at 66.98 — nearing overbought but not extreme yet; mild caution warranted.
* MACD: Positive but slightly below signal line (9.61 vs. 9.77) — suggests momentum is flattening near-term.
* Support/Resistance: Price is just below resistance (288.28) and close to the upper Bollinger Band (290.22) — possible short-term hesitation if volume stays low.
* Bollinger Bands: Wide — signals volatility expansion consistent with the strong uptrend.
* Volume: Recent 3-day average is about 52% of the longer-term average — light volume may limit a strong breakout for now.

Recommendation: HOLD / BUY ON DIP — Strong trend is intact but momentum is flattening and volume is soft near resistance. Hold if in, wait for a pullback toward 270–275 to add. Buy only if it breaks 288.28 with clear volume surge.
```

Cheers!