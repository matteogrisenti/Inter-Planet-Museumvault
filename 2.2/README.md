# Part 2.2: Multi-Robot and Drone Logistics

## Overview
This folder builds upon the base scenario to introduce an architecture with multi-agent capabilities and varying carrying capacities for **Problem 2**.

### Key Additions:
1.  **Specialized Roles**: 
    - **Admin**: Has high-level privileges for critical artifact access.
    - **Technician**: Built for heavy lifting; can carry **two items at once** using a double slot (with the constraint that the second item must be non-fragile).
    - **Scientist**: Authorized to access the Stasis-Lab and handle scientific artifacts.
2.  **Drones**: 
    - Drones are utilized to safely access seismic or hazardous zones without triggering depressurization issues. They are restricted to carrying non-fragile items and have no need for anti-vibration pods.
3.  **Refined Pressurization Logic**: 
    - Entering a pressurized room automatically disables the sealing mode, relying on implicit airlock mechanics.

## 🚀 Execution Commands

To run the standard multi-robot scenario, use `prp`:

```bash
prp domain.pddl problem.pddl
```

For the **drone** specific scenario, use `downward`:

```bash
downward domain.pddl problem.pddl
```
> **Note**: Make sure to navigate to the `drone/` subfolder if the drone files are separated, and execute the command there.