# 🪐 Interplanetary Museum Vault (IMV) - Planning Project

Welcome to the **Automated Planning Theory and Practice** project repository! This project models a series of planning scenarios based on the "Interplanetary Museum Vault" assignment.

## 📜 Scenario Overview
On Mars’ largest research outpost, a subterranean structure called the **Interplanetary Museum Vault (IMV)** stores fragile artifacts from various space missions. Due to a sudden streak of micro-quakes, pressure fluctuations threaten several high-value items. Autonomous robotic curators are deployed to execute stabilization and relocation tasks while avoiding unstable corridors and using limited cryogenic and anti-vibration resources.

### 🏛️ Zones
*   **Cryo-Chamber**: Maintains sub-zero temperatures (for temperature-sensitive artifacts).
*   **Anti-Vibration Pods**: Dampen vibrations, crucial for transport (only 2 available).
*   **Artifact Halls ($\alpha$ and $\beta$)**: Display areas. Hall $\beta$ is unstable during seismic activity.
*   **Maintenance Tunnel**: A low-pressure area where robots must activate sealing mode.
*   **Stasis-Lab**: Final safe destination for artifacts.

## 📂 Project Structure

Please refer to the detailed `README.md` inside each folder for specific information about the problem variations and execution instructions.

*   [**Folder 2.1**](./2.1/) - Problem 1: Base formulation with a single generic robotic curator.
*   [**Folder 2.2**](./2.2/) - Problem 2: Multi-robot specialized capabilities (Admin, Technician, Scientist) and Drones.
*   [**Folder 2.3**](./2.3/) - Problem 3: Hierarchical Task Networks (HTN) formulation.
*   [**Folder 2.4**](./2.4/) - Problem 4: Durative Actions and Time-based constraints (TILs).
*   [**Folder 2.5**](./2.5/) - Problem 5: ROS2 PlanSys2 Execution in a simulated robotic environment.
*   [**Visualization Tool**](./visualization_tool/) - Tools to visualize the generated plans.

---
*Developed for the Automated Planning Theory and Practice course by Marco Roveri.*
