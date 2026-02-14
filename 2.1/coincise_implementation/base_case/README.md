# Domain Comparison: Vanilla vs. Coincise

* **Vanilla Implementation:** Relied strictly on :strips and :typing. This forced the use of "Vanilla logic," where every state requires a positive predicate and a counter-predicate (e.g., hands-empty vs carrying).

* **Coincise Implementation:** Adds *:negative-preconditions*, *:conditional-effects*, and *:disjunctive-preconditions*. These allow the model to handle complex effects within a single action.


## Predicate Simplification (State Space Reduction)
In the coincise version, we removed "fleg predicates" to streamline the state space.

* Removed *hands-empty*: Instead of tracking this explicitly, we now use *(not (exists (?a - artifact) (carrying ?r ?a)))*. This ensures the robot only picks up items when its hands are logically empty.

* Removed *sealing-mode-off:* The robot’s status is now a single boolean *(sealed ?r)*.

* Removed *pod-empty / pod-full*: The status of a pod is derived from whether it *(pod-contains ?p ?a)* or not.

* Removed *no-fragile*: Instead of a separate category, the domain now simply checks for *(not (fragile ?a))*.


## Action Simplification 

### Movement and Atmosphere Safety
In the Vanilla version, movement was split into separate actions for pressurized and unpressurized rooms.

* **Optimization:** A single move action now uses an OR condition: *(or (is-pressurized ?to) (sealed ?r))*. This ensures the robot can only enter a vacuum if it is sealed, without needing multiple action definitions.

* **Automation:** We introduced Conditional Effects (when). When a robot enters a pressurized room, the effect *(when (is-pressurized ?to) (not (sealed ?r)))*.

### Unified Artifact Release (Cryo Logic)
The Vanilla domain required separate actions to drop items in standard rooms versus cryo-chambers to handle temperature changes.

* **Optimization**: We merged these into unified "release" actions. Using *(when (is-chill-room ?l) (cold ?a))*, the domain automatically applies the cold status if the destination is a cryo-chamber. This reduces the total action count while keeping the "Instant Temp Logic" intact.