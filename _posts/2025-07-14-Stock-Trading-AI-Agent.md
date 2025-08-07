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

Here are the responsibilities of each agent: 

* The Orchestrator Agent initiates the flow (e.g., on schedule or event).
* It requests the Stock Picker Agent for the current stock list.
* It instructs the Market Data Agent and News Agent to fetch data for those stocks.
* The Technical Analysis Agent and Sentiment Agent analyze their respective data.
* The Trading Rules Agent combines all signals and makes a recommendation. 
* The Orchestrator Agent decides whether to act and, if so, directs the Execution Agent. 
* All actions and decisions are logged by the Logging Agent.

The word “agent” here is a general term. It can refer to an LLM or an entity that performs specific tasks. For example:
* The “Technical Analysis Agent” is an LLM. It calculates various technical indicators for stocks and uses reasoning to provide a technical assessment.
* Conversely, the “Market Data Agent” retrieves stock data from the stock exchange.

### Technologies
I use ChatGPT-4.1 as the LLM and LangGraph as the agent framework. The app is written in Python.

The app operates as a REST service with endpoints to support on-demand stock analysis requests. 

Additionally, it has the "Trade Monitoring" job running in the background by the orchestrator to oversee the stocks. Just like a real Wall Street trader, the job uses a scanner that constantly monitors the list of stocks and only drills down when the technicals indicate a potential move worth trading.

#### LangGraph
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
```

### Demonstration
Using the above technology, I developed the Stock Analyzer app that offers professional stock insights and analysis upon user request. 

The Stock Analyzer app is available at: [https://stock-analyzer.modularmachines.ai](https://stock-analyzer.modularmachines.ai/). It is access-controlled. If you want to try, please let me know and I will create an account for you, and you can reset your password once you log in. 

#### PLTR
The Stock Analyzer's advice of **PLTR** stock on Aug 02, two days before its earning report:

<a href="/assets/ai_agent/pltr_analysis_screen.png" target="_blank">
  <img src="/assets/ai_agent/pltr_analysis_screen.png" />

#### AVGO

The Stock Analyzer's technical analysis of **Broadcom** stock on July 18:

>**AVGO Technical Analysis and Recommendation**
>
>**Trend:**  
>AVGO is in a clear uptrend. The current price ($283.27) is above the 20/50/150 SMAs and the 20-EMA, all of which are rising.
>
>**RSI (14):**  
>At 66.98, RSI is approaching the overbought zone (70+), but not excessively so. This signals strength but also potential for short-term consolidation or pullback.
>
>**MACD:**  
>MACD (9.61) is slightly below the signal line (9.77), which is a mild bearish crossover; however, the values are close, indicating momentum is slowing but not reversing.
>
>**Support/Resistance:**  
>Price is near the resistance at $288.28. If it breaks above, more upside is likely. Support sits well below at $244.17, protecting against a deep sell-off.
>
>**Bollinger Bands:**  
Bands are moderately wide ($290.22 upper, $253.58 lower), showing some volatility but not an extreme expansion or contraction.
>
>**Volume:**  
>Recent volume is significantly below average (ratio ~0.52), which may indicate either a pause before the next move or waning short-term momentum.
>
---
>
> **Recommendation: _Hold_**  
>The uptrend remains intact, but the proximity to resistance ($288.28), slightly overstretched RSI, slowing momentum (MACD), and lower volumes suggest a short-term pause or consolidation is likely. Wait for a clear breakout above resistance with volume for a fresh **buy** signal, or confirmation of reversal for a **sell**. For now, **hold**.

<!-- ![alt text](/assets/ai_agent/get_tech_analysis_screen.png "Title") -->

<a href="/assets/ai_agent/get_tech_analysis_screen.png" target="_blank">
  <img src="/assets/ai_agent/get_tech_analysis_screen.png" />

#### NFLX
On July 18, **Netflix** stock dropped 5% after a strong earnings report, and the large price swing triggered a sentiment assessment of the stock. Here is the response from my Stock Analyzer Agent:

> **Sentiment for NFLX News: MIXED (Positive Bias)**
>
> **Key headlines:**
> - Earnings beat but weak reaction: Strong Q2 results but “failed to impress” — short-term cautious tone.
> - Stock drop: Shares fell 5% despite the beat — short-term negative reaction.
> - Analyst action: Many major banks (Wells Fargo, UBS, JP Morgan, TD Cowen, Rosenblatt, Piper Sandler, Morgan Stanley) all maintain Buy/Overweight and raise price targets — strong medium-term positive signal.
> - Engagement mixed: “Anemic” engagement but “shockingly strong” retention — mixed narrative, mostly neutralized by higher subscriber stickiness.
> - Options activity: Covered call suggests cautious big money hedging.
>
> **Implication:**
> - Short-term sentiment is slightly negative due to the price drop and cautious reaction.
> - Medium-term sentiment is positive because multiple analysts raised price targets significantly, reinforcing fundamental confidence.
> - This offsets technical weakness (price stalling, slight profit-taking) but does not fully reverse it yet.
>
> **Sentiment**: **MIXED leaning POSITIVE** — News flow moderately strengthens the medium-term technical signal but near-term profit-taking risk remains.

