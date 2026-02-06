# Single Robot Scenario
run script
```bash 
downward --alias lama-first domain.pddl problem.pddl
```

# Introduction
In this specific scenario, a **single robot** must navigate a hazardous environment to preserve artifacts and evacuate threatened zones. The robotic curator has three primary objectives:
1. **Relocate** Martian Core Samples to the **Stasis Lab**.
2. **Move** temperature-sensitive artifacts (Ice/Eggs) to the **Cryo-Chamber**.
3. **Evacuate** all artifacts from **Hall B** (threatened by seismic activity) to **Hall A**.

## 🗺️ Topology
The map is built as a **Star Topology** centered on the **Maintenance Tunnel**, to which all other rooms are linked. The rooms are modeled as instances of the type `location`.

### Location Features
* **🔗 Connection:** Modeled as a tuple feature `(connected ?l1 ?l2 - location)`.
* **💨 Pressurization:** Defines if a room is pressurized. Only the Tunnel is unpressurized `(is-unpressurized ?l - location)`.
* **⚠️ Safety:** Defines if a room is safe to enter `(is-safe ?l -location)`.
* **🌡️ Drop Zones:** Distinguishes room types for drop effects:
    * `(is-standard-room ?l)`: Standard drop behavior.
    * `(is-chill-room ?l)`: Drops here cool the artifact (Cryo-Chamber).
* **📦 Resources:** `(contain-free-pod ?l)` indicates the room stocks empty anti-vibration pods.

> **📝 Note 1:** It would be interesting to test the same problem with a **positive definition** of the last two features (e.g., `is-pressurized`, `is-safe`). In theory, this implies a larger number of atoms which can affect the velocity of the resolver.

> **📝 Note 2:** To increase complexity, we can add a `capacity` constraint for the Cryo-Chamber room.

## 🤖 Robot
Since we have only one robot, we avoid defining it as a `type`, but as a simple object in the problem.

### Robot Features
* **📍 Location:** `(robot-at ?l - location)`
* **✋ Hand Status:** `(hand-empty)` (True if holding nothing)
* **📦 Inventory:**
    * `(carrying ?a - artifact)`: Robot is holding an artifact directly.
    * `(carrying-empty-pods)`: Robot is holding an empty anti-vibration pod.
    * `(carrying-in-pod ?a - artifact)`: Robot is holding an artifact secured inside a pod.
* **⚙️ Mode:** `(sealing-mode)` (Required to enter the unpressurized tunnel).

### Robot Actions
The robot's capabilities are split to handle resource constraints and safety checks:

* **Movement:**
    * `move-empty-*`: Move while holding nothing (Safe Room vs Tunnel).
    * `move-carrying-*`: Move while carrying a **non-fragile** artifact (Safe Room vs Tunnel).
    * `move-fragile-*`: Move while carrying a **fragile** artifact (requires `carrying-in-pod`).
* **Sealing:** `activate-seal` / `deactivate-seal` to traverse the Maintenance Tunnel.
* **Pod Management:**
    * `pick-up-empty-pod`: Retrieve an empty pod from the Pod Room.
    * `drop-empty-pod`: Drop an empty pod to free hands.
* **Manipulation:**
    * `pick-up`: Standard pickup for sturdy items.
    * `secure-pick-up`: Pickup an item and immediately place it into a held empty pod.
    * `drop-standard`: Drop a loose item.
    * `drop-standard-from-pod`: Unload an item from a pod into a standard room (keeping the pod).
    * `drop-in-cryo`: Drop a loose item into the Cryo-Chamber (applies cooling).
    * `drop-in-cryo-from-pod`: Unload an item from a pod into the Cryo-Chamber (applies cooling).

## 💎 Artifacts
Artifacts are modeled with specific **types** and **features** to handle preservation requirements.

### Artifact Types
* **Martian Core Artifact**
* **Martian Generic Artifact**
* **Martian Civilization Artifact**
* **Asteroid Generic Artifact**
* **Venus Generic Artifact**

### Artifact Features
* **Typology:** `(is-type ?a - artifact ?t - artifact-type)`
* **📍 Location:** `(artifact-at ?a - artifact ?l - location)`
* **💔 Fragility:**
    * `(fragile ?a)`: Artifact requires an anti-vibration pod for transport.
    * `(no-fragile ?a)`: Artifact can be carried normally.
* **🌡️ Temperature:**
    * `(warm ?a)`: Default state.
    * `(cold ?a)`: Artifact has been cooled (achieved by dropping in Cryo-Chamber).

> **📝 Note 3:** `need-chill` and `need-vibration-pods` predicates were removed in favor of `cold` state tracking and `fragile` constraints.

### Selected Artifact Set
The problem instance includes specific items initialized with fragility and temperature properties, for example:

| Artifact Name | Type | Fragile? |
| :--- | :--- | :--- |
| `mart-nord-core-drill` | `martian-core` | No |
| `mart-sand-sample` | `martian-generic` | **Yes** |
| `mart-laser-gun` | `martian-civilization` | **Yes** |

## 🏁 Initial State & Goals

### Initial State
The state establishes the star topology centered on the **Maintenance Tunnel**.
* **Robot:** Starts at the `entrance` with `handempty`.
* **Environment:**
    * `maintenance-tunnel` is `is-unpressurized` and `is-safe`.
    * `anti-vibration-pods-room` has `(contain-free-pod)`.
    * All rooms are initially `is-safe` (Hall B safety is static in this problem instance).
* **Artifact Distribution:**
    * **Hall A:** Contains Core Drills and Ice/Organic Samples (Non-Fragile).
    * **Hall B:** Contains Generic/Civilization items (Fragile, require pods).
* **Properties:** Artifacts in Hall B are explicitly marked as `(fragile ?a)`, forcing the robot to use pods to move them.

### Goal State
The mission is complete when:
1. **Martian Core Artifacts** are inside the `stasis-lab` and are `cold`.
2. **Temperature Sensitive Artifacts** (Ice/Eggs) are inside the `cryo-chamber`.
3. **Hall B Artifacts** are evacuated and placed in `hall-a`.

> **📝 Note 4:** We can add flexibility by allowing cold artifacts to be stored in **either** the Cryo-Chamber **or** the Stasis-Lab. This allows us to test if the planner finds the optimal goal versus a suboptimal (but valid) one.