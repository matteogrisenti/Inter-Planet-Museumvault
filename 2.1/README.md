# Part 2.1: Base Problem

## Overview
This folder contains the formulation for **Problem 1** of the Interplanetary Museum Vault scenario. 

In this base case:
- A single generic robotic curator must move artifacts to their safe destinations.
- Artifacts must be protected using anti-vibration pods if they are fragile.
- Temperature-sensitive items must be brought to the Cryo-Chamber to prevent overheating.
- The robot must use its "sealing mode" to safely traverse the unpressurized Maintenance Tunnel.

### ⚙️ Implementation Notes
Several modeling choices were made to optimize the problem. We developed both a vanilla implementation and an optimized "concise" implementation:
- **State Space Reduction**: Removed explicit flag predicates (e.g., `hands-empty`) in favor of logical existence checks.
- **Conditional Effects**: Used `when` clauses to automatically handle robot sealing modes upon entering pressurized rooms, and instantly applying 'cold' conditions when releasing artifacts in the Cryo-Chamber.

## 🚀 Execution Command

To execute the planner and solve the problem in this folder, use the `prp` planner:

```bash
prp domain.pddl problem.pddl
```
