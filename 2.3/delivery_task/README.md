# Problem 3: Hierarchical Task Network (HTN) - Core Delivery Decomposition

## Overview and Goal
This directory contains the initial phase of the Hierarchical Task Network (HTN) implementation for the Interplanetary Museum Vault scenario. The primary focus of this section is the complete decomposition of the central delivery task, which abstracts the complex logistics of transporting an artifact between two distinct locations into a structured hierarchy of subtasks.

Instead of relying on a flat search space, this model forces the planner to logically break down the mission, significantly guiding the heuristic search.

### Key Modeling Features
* **Sequential Delivery Decomposition**: The primary method (m_delivery_direct) breaks the high-level delivery into three strict, sequential phases: prepare_robot, load_artifact, and transport_to_target.
* **Context-Aware Preparation & Loading**: The prepare_robot task dynamically ensures the robot is at the correct location and equipped with an anti-vibration pod if the artifact is fragile. The load_artifact task then manages the specific pickup constraints based on the artifact's type and the robot's available inventory slots.
* **Thermal Routing**: The transport_to_target task manages environmental requirements, including automatically routing temperature-sensitive artifacts through a cryo-chamber to chill them before reaching their final destination.
* **Two-Layered Navigation**: Motion is abstracted into two distinct layers: a high-level get_to task that resolves paths across multiple nodes, and a primitive step task that handles the physical constraints of crossing between adjacent rooms (e.g., pressurization and vacuum seals).
* **Determinized Seismic Hazards**: To streamline the HTN framework and avoid generating full policies, the stochastic risk of entering the seismic zone (Hall B) has been determinized; the model assumes the robot successfully waits for a safe window to transition.
* **Collaborative Relay Strategy**: The problem file is designed to support multi-agent handoffs. For example, a Technician can transport an artifact to the Maintenance Tunnel, acting as a handover point for a Scientist to complete the delivery to the Stasis Lab.

## Execution Instructions
1. Run Search (Plan Generation)
   To generate the hierarchical plan, we use the PANDA planner. Ensure you are in the correct directory containing your HDDL files.Run the PANDA planner using the A* search algorithm and the tdg-m heuristic:
   
   ```bash
   java -jar PANDA.jar -parser hddl -searchAlgorithm astar -heuristic tdg-m domain.hddl problem.hddl raw_plan.dot
   ```

   (Note: tdg-m is the fastest heuristic among the tested options, but $A^$ search may still take some time).
   
2. Plot Search (Visualization)
   Because a Python parser script could not be automatically generated to extract the hierarchical search structure, the graph formatting was done manually. The structured output is saved in structured-result.txt, which was used to generate the .dot files located in the graph directory.To build the PDF visual trees from these manually structured .dot files, run the following Graphviz commands:
   ```bash
   dot -Tpdf delivery_task_1.dot -o delivery_task_1_l.pdf
   dot -Tpdf delivery_task_1.dot -o delivery_task_1_r.pdf
   dot -Tpdf delivery_task_2.dot -o delivery_task_2.pdf
   ```