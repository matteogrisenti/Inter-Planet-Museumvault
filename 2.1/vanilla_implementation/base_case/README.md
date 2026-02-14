# Single Robot Scenario ( NON DETERMINISTIC )
This project models a robotic curator in a hazardous environment. The domain is designed for non-deterministic planning, specifically requiring a solver capable of handling oneof effects (probabilistic outcomes) and conditional observation, such as PRP (Probabilistic Relevant Planner).

To run the current PDDL model with the PRP solver:
```bash 
planutils run prp domain.pddl problem.pddl -- --dump-policy 2
```

## 🗺️ Environment & Topology
The map is built as a **Star Topology** centered on the **Maintenance Tunnel**, to which all other rooms are linked. The rooms are modeled as instances of the type `location`.

### 1. Atmospheric Pressure
* **(is-pressurized ?l):** Safe for normal operation. The robot must *deactivate* its seal to perform complex manipulation here (though the current planner keeps it simple).
* **(is-unpressurized ?l):** Vacuum areas (e.g., `maintenance-tunnel`).
    * **Constraint:** The robot **must** have `(sealing-mode-on)` to enter.

### 2. Seismic Safety (Non-Deterministic)
Some rooms are prone to earthquakes. The state of these rooms is not fully known until the robot attempts to enter.
* **(is-unseismic ?l):** The room is guaranteed safe.
* **(is-seismic ?l):** The room might be unsafe.
* **Action `try-to-enter-seismic-room`:** This is a non-deterministic action.
    * *Outcome A:* The room is `(is-safe)`, and the robot enters.
    * *Outcome B:* The room is `(is-unsafe)`, and the robot fails to enter (stays at origin).

### 3. Room Specializations
* **(is-standard-room ?l):** Normal drop zones.
* **(is-chill-room ?l):** Cryo-Chamber. Dropping an artifact here applies the `(cold ?a)` effect.



## 🤖 The Robot (Curator)
The robot is a sophisticated agent with internal sealing mechanisms and inventory management.

### States
* **Sealing:** `(sealing-mode-on ?r)` vs `(sealing-mode-off ?r)`.
* **Hands:** `(hands-empty ?r)` or `(carrying ?r ?a)`.
* **Pod Integration:**
    * `(carrying-empty-pod ?r ?p)`: Robot is holding an empty container.
    * `(carrying-full-pod ?r ?p)`: Robot is holding a container with an artifact inside.

### Capabilities (Actions)

| Category | Action | Description |
| :--- | :--- | :--- |
| **Movement** | `move-to-pressurized-room` | Standard move between safe, pressurized rooms. |
| | `move-to-unpressurized-room` | Move into/through vacuum. **Requires:** `sealing-mode-on`. |
| | `try-to-enter-seismic-room` | **Non-deterministic.** Attempts to enter a seismic room. May fail if room is unsafe. |
| **Sealing** | `activate-seal` | Prepares robot for vacuum. |
| | `deactivate-seal` | Opens seals. Only allowed in pressurized rooms. |
| **Pod Handling** | `pick-up-empty-pod` | Picks up a specific pod object `?p`. |
| | `pick-up-full-pod` | Picks up a pod that already contains an item. |
| | `drop-empty-pod` | Drops an empty pod to free hands. |
| | `drop-full-pod` | Drops a full pod (does not unpack it). |
| **Artifacts** | `pick-up-artifact-standard` | Grabs a `no-fragile` item directly. |
| | `put-in-pod` | **Load Logic:** While carrying an empty pod, the robot grabs an artifact and seals it inside. Outcome: `carrying-full-pod`. |
| | `release-artifact` | Drops a `no-fragile` item directly. |
| | `release-artifact-from-pod` | **Unload Logic:** Unpacks item into room. Robot keeps the empty pod. |
| **Cryo** | `release-artifact-in-cryo` | Drops item in Cryo-Chamber -> Item becomes `(cold)`. |
| | `release-artifact-in-cryo-from-pod` | Unpacks item in Cryo-Chamber -> Item becomes `(cold)`. |



## 📦 Objects: Pods & Artifacts

### Pods
Unlike previous iterations where pods were room properties, **Pods are now physical objects** (`pod1`, `pod2`).
* They can be moved between rooms.
* They have states: `(pod-empty ?p)` or `(pod-full ?p)`.
* **Utility:** Required to transport `fragile` items.

### Artifacts
Artifacts have specific handling requirements defined by their type.

* **Fragility:**
    * `(fragile ?a)`: Cannot be picked up directly. Must be handled via `put-in-pod` or pre-loaded pods.
    * `(no-fragile ?a)`: Can be carried in "hand" without a pod.
* **Temperature:**
    * `(warm ?a)`: Initial state.
    * `(cold ?a)`: Achieved by processing the item in a `chill-room`.



## 🏁 Problem Instance (problem.pddl)

### Initial State
* **Robot:** At `entrance`, hands empty, seal off.
* **Pods:** Two empty pods located in `anti-vibration-pods-room`.
* **Seismic Threat:** `hall-b` is marked `(is-seismic)`.
* **Artifacts:**
    * **Hall A:** Contains Core Drills (Warm, Non-Fragile).
    * **Hall B:** Contains Samples & Tools (Fragile, require Pods).

### Goals
1.  **Martian Cores:** Must be at `stasis-lab` AND be `cold`.
    * *Strategy:* Robot must detour to `cryo-chamber` to cool them before final delivery.
2.  **Biological Samples:** Must be at `cryo-chamber` (implicitly becoming cold).
3.  **Rescue Mission:** All artifacts currently in `hall-b` must be moved to `hall-a`.
    * *Constraint:* These items are `fragile`, so the robot must fetch pods from the pod room, bring them to Hall B, load the items, and transport them out.