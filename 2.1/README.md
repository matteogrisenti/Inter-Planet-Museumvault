# Single Robot Scenario

## Introduction
In this specific scenario, a **single robot** must navigate a hazardous environment to preserve artifacts and evacuate threatened zones. The robotic curator has three primary objectives:
1.  **Relocate** Martian Core Samples to the **Stasis Lab**.
2.  **Move** temperature-sensitive artifacts (Ice/Eggs) to the **Cryo-Chamber**.
3.  **Evacuate** all artifacts from **Hall B** (threatened by seismic activity) to **Hall A**.


## 🗺️ Topology
The map is built as a **Star Topology** centered on the **Maintenance Tunnel**, to which all other rooms are linked. The rooms are modeled as instances of the type `location`.

### Location Features
* **🔗 Connection:** Modeled as a tuple feature.
    * `(connected ?l1 ?l2 - location)`
* **💨 Pressurization:** Defines if a room is pressurized. Only the Tunnel is unpressurized.
    * `(is-unpressurized ?l - location)`
* **⚠️ Safety:** Defines if a room is safe to enter. Only Hall B can be unsafe during mars-quakes.
    * `(is-unsafe ?l -location)`

> **📝 Note 1:** It would be interesting to test the same problem with a **positive definition** of the last two features:
> * `(is-pressurized ?l - location)`
> * `(is-safe ?l -location)`
>
> In theory, this implies a larger number of atoms (due to the Closed World Assumption usually favoring negative defaults) which can affect the velocity of the resolver.

> **📝 Note 2:** To increase complexity, we can add a `capacity` constraint for the Cryo-Chamber room and potentially for all rooms.



## 🤖 Robot
Since we have only one robot, in this case we can avoid defining it as a `type`, but as a simple object in the problem. This modeling choice is not flexible; indeed, for the next step, we need to change this aspect. But since here we don't need this flexibility, we decided to avoid it and add it in a second moment.

### Robot Features
* **📍 Location:** `(robot-at ?l - location)`
* **✋ Hand Status:** `(hand-empty)` (True if holding nothing)
* **📦 Inventory:** `(carrying ?a - artifact)`
* **🛡️ Equipment:** `(carrying-anti-vibration-pods)` (Robot possesses a pod)
* **⚙️ Mode:** `(sealing-mode)` (Required to enter the unpressurized tunnel)

### Robot Actions
**TODO**



## 💎 Artifacts
They are modeled with specific **types** to handle distinct preservation requirements.

### Artifact Types
* **Martian Core Artifact:** Samples from the Martian underground.
* **Martian Generic Artifact:** Standard Martian surface items.
* **Martian Civilization Artifact:** Proof of alien civilization.
* **Asteroid Generic Artifact:** Samples collected from asteroids.
* **Venus Generic Artifact:** Relics from old Venusian missions.

### Artifact Features
* **Typology:** `(is-type ?a - artifact ?t - artifact-type)`
* **📍Location:** `(artifact-at ?a - artifact ?l - location)`
* **❄️ Needs Cooling:** `(need-chill ?a - artifact)` (Must be kept cold to prevent degradation).
* **📦 Needs Protection:** `(need-anti-vibration-pods ?a - artifact)` (Requires a pod for transport).
* **🌡️ Status:** `(cold ?a - artifact)` (Indicates the artifact is currently cold).

> **📝 Note 3:** `need-chill` and `need-vibration-pods` could technically be removed/inferred because they often coincide with the artifact's initial location. However, we keep them explicit for clarity and validation.

### Selected Artifact Set
The problem instance includes the following specific items:

| Artifact Name | Type |
| :--- | :--- |
| `mart-nord-core-drill` | `martian-core` |
| `mart-sud-core-drill` | `martian-core` |
| `mart-east-core-drill` | `martian-core` |
| `mart-west-core-drill` | `martian-core` |
| `mart-sand-sample` | `martian-generic` |
| `mart-north-pole-ice-sample` | `martian-generic` |
| `mart-mysterious-egg` | `martian-generic` |
| `mart-laser-gun` | `martian-civilization` |
| `mart-pink-hat` | `martian-civilization` |
| `asteroid-MG04TN-ice-sample` | `martian-civilization` |
| `asteroid-AD-----rock-sample` | `martian-civilization` |
| `venus-sand-sample` | `venus-generic` |
| `venus-rock-sample` | `venus-generic` |



## 🏁 Initial State & Goals

### Initial State
The state establishes the star topology centered on the **Maintenance Tunnel**.
* **Robot:** Starts at the `entrance` with `handempty`.
* **Environment:** The `maintenance-tunnel` is set as `is-unpressurized`.
* **Artifact Distribution:**
    * **Hall A:** Contains artifacts that **need to be cold** (Core Drills, Ice & Organic Samples).
    * **Hall B:** Contains all other artifacts.
* **Requirements:** Specific artifacts are initialized with `(need-chill)` and `(need-anti-vibration-pods)` based on their type.

### Goal State
The mission is complete when:
1.  **Martian Core Artifacts** are inside the `stasis-lab`.
2.  **Temperature Sensitive Artifacts** (Ice/Eggs) are inside the `cryo-chamber`.
3.  **Hall B Artifacts** are evacuated and placed in `hall-a`.

> **📝 Note 4:** We can add flexibility by allowing cold artifacts to be stored in **either** the Cryo-Chamber **or** the Stasis-Lab. This allows us to test if the planner finds the optimal goal versus a suboptimal (but valid) one.