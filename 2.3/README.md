# Problem 3: Hierarchical Task Network (HTN)

## Overview
This section of the project focuses on transitioning the Interplanetary Museum Vault scenario from a classical planning model into a Hierarchical Task Network (HTN) framework. 

Instead of treating the mission as a flat sequence of instantaneous actions, the HTN approach abstracts the mission's core objective—the `delivery` task (transporting an artifact by a robot between two distinct locations)—into a structured hierarchy of logical subtasks. 

## Why is this section split into two parts?
As HTN domains grow in complexity—especially when introducing multi-agent constraints, specialized item handling, and multi-layered pathfinding—the search space can easily explode, causing solvers to hang or run out of memory. 

To prevent the search from exploding and to maintain a tractable, testable architecture, the HTN implementation has been divided into two distinct directories:

### 1. Core Delivery Decomposition (`task_decomposition`)
This first part focuses purely on the architectural breakdown of the HTN. It models how the abstract `delivery` task is split into three strict, sequential phases: preparation (`prepare_robot`), loading (`load_artifact`), and transportation (`transport_to_target`). This directory is primarily concerned with proving the logic of the decomposition tree and ensuring that subtasks correctly resolve down to primitive actions.

### 2. Delivery Search & Coordination (`delivery_search`)
The second part takes the robust decomposition rules established in the first part and applies them to a broader, multi-artifact search scenario. By leveraging the HTN structure to severely prune invalid branches (e.g., preventing unnecessary teleportation or invalid hand-offs), this phase tests the planner's ability to coordinate multiple robots and execute complex relay strategies efficiently without succumbing to state-space explosion.


**Navigation:**
Please refer to the individual `README.md` files within each sub-directory for specific execution commands, parsing instructions, and detailed explanations of the modeled constraints.