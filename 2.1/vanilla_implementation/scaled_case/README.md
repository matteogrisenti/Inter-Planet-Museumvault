# Scalability Analysis: "Vanilla" PDDL Implementation

## 1. Overview
This document details the scalability testing of a **Single-Robot Domain** modeled using a "Vanilla" PDDL strategy. The goal is to stress-test the implementation by incrementally increasing the number of artifacts and task complexity to identify potential performance bottlenecks in both the **Translation** (memory/grounding) and **Search** (heuristic/state-space) phases.

### Implementation Strategy: "Vanilla" PDDL
* **Core Concept**: Minimal use of advanced requirements (`:adl`, `:conditional-effects`).
* **Logic**: Explicit positive/negative predicates (e.g., `sealing-mode-on` vs `sealing-mode-off`) rather than complex logical implications.
* **Hypothesis**: Simpler logic structures yield faster heuristic evaluation at the cost of verbose plan lengths.

---

## 2. Test Scenarios (The Batch System)

The problem is scaled using an additive "Batch" system, creating four distinct difficulty levels to isolate specific performance variables (Volume vs. Complexity).

| Level | Scenario Name | Items Added | Total Items | Description & Complexity Focus |
| :--- | :--- | :--- | :--- | :--- |
| **0** | **Base Game** | - | ~13 | **Baseline Routing:** Standard distribution of items across safe and pressurized rooms. Establishes the performance baseline. |
| **1** | **+ Batch 1** | +10 | ~23 | **"The Biologist's Cache":** Focuses on simple fetch tasks (Point A $\to$ Point B). Tests the solver's ability to handle increased object volume without complex dependencies. |
| **2** | **+ Batch 2** | +10 | ~33 | **"The Heavy Cores":** Focuses on **Complex Routing**. These items require a 3-step dependency chain involving temperature changes (Pick $\to$ Cryo $\to$ Cool $\to$ Stasis), testing the heuristic's ability to navigate deep search trees. |
| **3** | **+ Batch 3** | +10 | ~43 | **"The Seismic Nightmare":** Focuses on **High Volume & Risk**. Adds items located exclusively in `hall-b` (Seismic Room). This forces the planner to handle a high volume of fetch tasks while repeatedly verifying the non-deterministic seismic safety of the room. |

---

## 3. How to Run the Tests

The scaling is controlled via commenting/uncommenting sections in `problem.pddl`.

1.  Open `problem.pddl`.
2.  Locate the sections marked `;; [BATCH X] ...`.
3.  Uncomment the lines in three specific sections to enable a batch:
    * `(:objects ...)` - To define the new artifacts.
    * `(:init ...)` - To set their initial positions and properties.
    * `(:goal ...)` - To add them to the victory conditions.
4.  Run the planner:
    ```bash
    planutils run prp domain.pddl problem.pddl
    ```

### Notes
We have removed all the output solution becouse uses-less for out goal in order to reduce the weight of the git repository 