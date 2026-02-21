# Problem 3: Hierarchical Task Network (HTN) - Delivery Search
## Overview and Goal
This directory contains the Hierarchical Task Network (HTN) implementation for the Interplanetary Museum Vault scenario. The primary goal of this phase is to transition from a classical planning model to a hierarchical framework, using task decomposition to handle the complex logistics of artifact transport.

Instead of relying solely on a flat sequence of actions, this model abstracts the fundamental delivery task—transporting an artifact between two distinct locations—into structured, sequential subtasks.

### Key Modeling Features
* **Delivery Decomposition**: The high-level delivery task is split into three phases: robot preparation (navigating to the artifact and fetching/dropping Anti-Vibration Pods if necessary), artifact loading, and transportation to the target.
* **Thermal Logistics**: The planner automatically routes temperature-sensitive artifacts through the Cryo-Chamber for chilling before proceeding to their final destination.
* **Multi-Layered Motion**: Navigation is abstracted into two layers: a high-level get_to task for path abstraction and a primitive step task for physical movement between adjacent rooms.
* **Determinized Hazards**: To simplify HTN policy generation, movement into the seismic zone (Hall B) has been determinized. The model assumes the robot successfully waits for a safe window to transition.
* **Relay Strategy Formulation**: The problem.hddl file is structured to facilitate complex multi-agent handoffs, such as a Technician moving an artifact to the Maintenance Tunnel and a Scientist completing the delivery to the Stasis Lab.

## Execution Instructions
1. Run Search (Plan Generation) 
   To generate the plan, we use the PANDA planner inside a Docker container.
   1. Ensure the Docker Desktop application is open and running.
   2. Launch the myplanutils container with elevated privileges, mounting your current directory:

    ```bash
    run -v "/$(pwd):/computer" -it --privileged --rm myplanutils bash
    ``` 
    3. Navigate to the specific project directory inside the container:

    ```bash
    ../computer/2.3/delivery_search/
    ``` 
    
    4. Run the PANDA planner using the A* search algorithm and the tdg-m heuristic:
    
    ```bash
    -jar PANDA.jar -parser hddl -searchAlgorithm astar -heuristic tdg-m domain.hddl problem.hddl raw_plan.dot
    ```
    Note: The tdg-m heuristic is the fastest among the tested options, but the $A^*$ search may still take some time to compute the full hierarchy.

2. Plot Search (Visualization)
    Once the raw_plan.dot file is generated, you can parse and convert it into a readable PDF hierarchy tree using the custom Python parser and Graphviz.
    1. Open a new, local terminal (outside the Docker container) and activate your Python virtual environment:
    ```bash
    myenv/Scripts/activate
    ```
    2. Navigate to the project directory:
    ```bash
    cd 2.3/delivery_search/
    ```
    3. Run the parsing script to clean and format the graph data:
    ```bash
    python parser.py
    ```
    (This will process raw_plan.dot and output a formatted albero_a4.dot file).
    4. Generate the final PDF graph using Graphviz:
    ```bash
    dot -Tpdf albero_a4.dot -o piano_htn.pdf
    ```
