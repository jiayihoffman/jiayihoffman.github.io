---
layout: post
title: "Stock Analyzer AI Agent"
date: 2025-07-14 08:45:28 -0600
categories: AI_Agent
image: /assets/ai_agent/pltr_stock_chart.png
---
I enjoy trading stocks for long-term investing because it requires knowledge, analytical skills, and a long-term perspective. Over time, I have developed technical skills in analyzing stock charts to identify patterns, resistance and support levels, and to gauge whether the stock is becoming bearish or bullish. I also enjoy reading news and listening to tech podcasts to better understand the world around me and upcoming trends.

## A stock app
That said, I don't always have time to watch live stock charts. That makes me want to create a stock app where AI agents analyze the charts and relate them to market sentiment to give trading suggestions.  

Additionally, in many ways, trading stocks is more of a psychological game than a numbers game. Therefore, using AI agents to recommend stock actions can be helpful during major pullbacks when everyone is scared or at market peaks when everyone is greedy. AI has a much calmer mind than we humans. :)

I have a few stocks to start with, but I’d love to add more to the list as the AI agent researches and recommends new stocks in sectors I’m interested in.

### Architecture
Here’s a brief overview of the architecture. The benefit of agent-based design is that:
1. It is modular, flexible, and with expert stock analyst intelligence. 
2. This design supports the "in the loop" evaluation, enabling the Orchestrator agent to critique the output of other agents to promote reflection, which is a key aspect of the agentic reasoning pattern that help improve performance. 
3. Additionally, the logging agent records the stock's insights and feedback to facilitate ongoing iterative review and improvement.

<a href="/assets/ai_agent/stock_analyzer_components.drawio.png" target="_blank">
  <img src="/assets/ai_agent/stock_analyzer_components.drawio.png" />
</a>

Here are the responsibilities of each agent: 

* The Orchestrator Agent initiates the flow (e.g., on schedule or event).
* It instructs the Technical Analysis Agent and Sentiment Agent to analyze the stocks.
* The Technical Agent contacts the Market Data Agent for stock data, and the Sentiment Agent uses News Agent to fetch news data for the stocks.
* The Orchestrator Agent combines all signals and makes a recommendation. 
* All insights are logged by the Logging Agent.

The word “agent” here is a general term. It can refer to an large language model (LLM) or an entity that performs specific tasks. For example:
* The “Technical Analysis Agent” is a large language model. It calculates various technical indicators for stocks and uses reasoning to provide a technical assessment.
* Conversely, the “Market Data Agent” retrieves stock data from the stock exchange.

### Technologies
I use ChatGPT-4.1 as the large language model (LLM) and LangGraph as the agent framework. The app is built with Python and Angular. The web user interface is developed entirely through AI-assisted coding.

The app operates as a REST service with endpoints to handle on-demand stock analysis requests. It also runs the "watchlist" job in the background to automatically generate insights for stocks configured by the user.

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

### The Stock Analyzer app
Here is the Stock Analyzer, the app that offers professional stock technical insights and market sentiment analysis.

The Stock Analyzer app is available at: [https://stock-analyzer.modularmachines.ai](https://stock-analyzer.modularmachines.ai/). It is access-controlled. If you'd like to try it, let me know, and I will create an account for you. You can reset your password once you log in.

Here are the screenshots of the product:

<a href="/assets/ai_agent/ccj_chart.png" target="_blank">
  <img src="/assets/ai_agent/ccj_chart.png" width="360" />
<a href="/assets/ai_agent/ccj_analysis.png" target="_blank">
  <img src="/assets/ai_agent/ccj_analysis.png" width="370" />

### Testimony

#### Palantir (PLTR)

Two days before Palantir's earnings report, the Stock Analyzer's insights recommended holding the position and not trimming it. It also noted that news headlines reinforced the technical outlook and supported a bullish run. After the earnings report, PLTR stock surged by 22%, from $154 to $188.

Then, on Friday, August 08, the Analyzer recommended that I trim slightly and take some profits because the stock was strongly overbought. One week later, the stock dropped 6% to $177. Ten days later, on August 19, PLTR sits at $157, 16% down from the price point where the Stock Analyzer suggested trimming.

<a href="/assets/ai_agent/pltr_stock_chart.png" target="_blank">
  <img src="/assets/ai_agent/pltr_stock_chart.png" />

<a href="/assets/ai_agent/pltr_analysis_8-3.png" target="_blank">
  <img src="/assets/ai_agent/pltr_analysis_8-3.png" />

<a href="/assets/ai_agent/pltr_analysis_8-8.png" target="_blank">
  <img src="/assets/ai_agent/pltr_analysis_8-8.png" />

#### Tesla (TSLA)

On Aug 27, the Stock Analyzer app recommended that I modestly trim my Tesla stock position at $352 since the TSLA chart indicates uncertainty and lack of conviction in breaking through the resistance.

This turned out to be a very helpful tip. The Tesla stock dropped more than $20 over the next two trading days.

<a href="/assets/ai_agent/tsla_stock_chart.png" target="_blank">
  <img src="/assets/ai_agent/tsla_stock_chart.png" />

<a href="/assets/ai_agent/tsla_analysis_8_27.png" target="_blank">
  <img src="/assets/ai_agent/tsla_analysis_8_27.png" />


#### Broadcom (AVGO)

Here is the Stock Analyzer's technical review of Broadcom stock (symbol AVGO) on July 18. It recommended holding and waiting for a clear breakout since momentum is slowing but not reversing. The uptrend remains intact. The stock price was at $283 on July 18.

On July 25, AVGO broke through resistance and has been soaring since then. Its current price (as of August 15) is $306.34.

<a href="/assets/ai_agent/avgo_stock_chart.png" target="_blank">
  <img src="/assets/ai_agent/avgo_stock_chart.png" />

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

