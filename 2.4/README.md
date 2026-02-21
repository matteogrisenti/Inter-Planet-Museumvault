# ⏱️ Part 2.4: Durative Actions and Parallelism

## 📝 Overview
This section tackles **Problem 4**, an evolution of the domain that introduces temporal planning using **Durative Actions**. 

### 🔑 Key Upgrades:
- **Temporal Constraints**: Actions now have specific execution durations (e.g., flying, moving, picking up), allowing robots and drones to perform tasks in parallel when physically possible.
- **Seismic Windows & Timed Initial Literals (TILs)**: Instead of non-deterministic mechanics, the instability of Artifact Hall $\beta$ is explicitly modeled using time. The predicate `(is-safe ?l)` is toggled over time, forcing agents to wait or plan around the dangerous seismic periods.
- **Optimizations**: Predicates that are static (e.g., whether a room is pressurized) are evaluated `at start`. Fragile constraints are explicitly enforced during pickup actions to minimize planner branching.

## 🚀 Execution Commands

To run the base durative problem use `optic`:

```bash
optic -N -E -W5,1 domain.pddl problem.pddl
```

For the **drone** specific durative problem, use `popf`:

```bash
popf domain.pddl problem.pddl
```