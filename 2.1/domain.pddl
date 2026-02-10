(define (domain single-robot)
    (:requirements :strips :typing :non-deterministic)

    ;; -------------------------------------------------------------------------
    ;; TYPE DEFINITIONS
    ;; -------------------------------------------------------------------------
    (:types
        robot           ; The primary actor
        pod             ; Storage container for fragile artifacts
        location        ; Rooms or Tunnels
        artifact        ; Items to be transported
        artifact-type   ; Categorization for goal requirements
    )

    ;; -------------------------------------------------------------------------
    ;; PREDICATES
    ;; Logic Note: This domain follows "Vanilla PDDL" (No NOT in preconditions).
    ;; Every state has a positive counter-state (e.g., empty vs full).
    ;; -------------------------------------------------------------------------
    (:predicates
        ;; Robot State
        (robot-at ?r - robot ?l - location)
        (hands-empty ?r - robot)          ; Robot can pick something up
        (carrying ?r - robot ?a - artifact)
        (carrying-full-pod ?r - robot ?p - pod)
        (carrying-empty-pod ?r - robot ?p - pod)

        ;; Atmosphere/Sealing Logic
        ;; Constraint: Robots must be 'sealed' to enter unpressurized tunnels.
        (sealing-mode-on ?r - robot)
        (sealing-mode-off ?r - robot)

        ;; Room Safety (Seismic Logic)
        ;; Non-deterministic states: Seismic rooms must be tested before entry.
        (is-seismic ?l - location)           ; Room is prone to earthquakes ( it's safty need to be checked )
        (is-unseismic ?l - location)         ; Room is geologically stable  ( it's safty is sure )
        ; The safe and unsafe are used only to track the state of the seismic rooms 
        ; ( the unseismic room are for deafoult safe )
        (is-safe ?l - location)              ; Room is safe to enter
        (is-unsafe ?l - location)

        ;; Room Spatial Connectivity
        (connected ?l1 ?l2 - location)
        (is-unpressurized ?l - location)  ; Requires Sealing Mode ON
        (is-pressurized ?l - location)    ; Requires Sealing Mode OFF for some tasks

        ;; Room Drop Specializations 
        (is-standard-room ?l - location)    ; Simple droput logic;
        (is-chill-room ?l - location)       ; Cryo-chamber (after the dropout it cool the artifact)

        ;; Artifact Properties
        (is-type ?a - artifact ?t - artifact-type)
        (artifact-at ?a - artifact ?l - location)
        (fragile ?a - artifact)              ; Must be in a pod to move safely
        (no-fragile ?a - artifact)        ; Can be moved by hand
        (cold ?a - artifact)              ; Target state for cryo-items
        (warm ?a - artifact)              ; Initial state for cryo-items

        ;; Pod State
        (pod-at ?p - pod ?l - location)             ; pod position            
        (pod-empty ?p - pod)                        ; pod empty
        (pod-full ?p - pod)                             ; pod contains an artifact
        (pod-contains ?p - pod ?a - artifact)       ; artifact inside the pod 
    )

    ;; =========================================================================
    ;; MOVEMENT ACTIONS
    ;; =========================================================================

    ;; ACTION: move-to-pressurized-room
    ;; PURPOSE: Standard movement between unseismic rooms.
    ;; Note: Only for unseismic rooms. Seismic rooms require the 'try-to-enter' action to check theit
    ;; safty befor enter in.
    (:action move-to-pressurized-room
        :parameters (?r - robot ?from ?to - location)
        :precondition (and 
            (robot-at ?r ?from)
            (connected ?from ?to)
            (is-pressurized ?to)
            (is-unseismic ?to)
        )
        :effect (and 
            (not (robot-at ?r ?from)) 
            (robot-at ?r ?to)
            (not (sealing-mode-on ?r))
            (sealing-mode-off ?r)
        )
    )

    ;; ACTION: move-to-unpressurized-room
    ;; PURPOSE: Transition from a pressurized room to a tunnel/vacuum area.
    ;; SAFETY: Robot MUST have sealing-mode-on to survive the pressure drop.
    (:action move-to-unpressurized-room
        :parameters (?r - robot ?from ?to - location)
        :precondition (and 
            (robot-at ?r ?from)
            (connected ?from ?to)
            (is-unpressurized ?to)
            (is-unseismic ?to)
            (sealing-mode-on ?r) ; Hard physical constraint
        )
        :effect (and 
      (not (robot-at ?r ?from)) 
      (robot-at ?r ?to)
      (not (sealing-mode-on ?r))
      (sealing-mode-off ?r)
  )
    )

    ;; ACTION: try-to-enter-seismic-room
    ;; PURPOSE: Models the risk of entering an earthquake-prone area.
    ;; OUTCOME: 
    ;;   - Success: Room found safe, robot moves in.
    ;;   - Failure: Room found unsafe, robot stays at origin.
    (:action try-to-enter-seismic-room
        :parameters (?r - robot ?to ?from - location)
        :precondition (and 
            (robot-at ?r ?from) 
            (connected ?from ?to) 
            (is-seismic ?to)
        )
        ;; NON DETERMINISTIC EFFECT
        :effect (oneof 
            ;; CASE A: Room is safe
            (and (is-safe ?to) (not (robot-at ?r ?from)) (robot-at ?r ?to))      
            ;; CASE B: Room is unsafe
            (not (is-safe ?to))    
        )
    )

    ;; =========================================================================
    ;; SEALING SYSTEM
    ;; =========================================================================

    ;; ACTION: activate-seal
    ;; PURPOSE: Prepares robot for vacuum/tunnel. Can be done anywhere.
    (:action activate-seal
        :parameters (?r - robot)
        :precondition (sealing-mode-off ?r)
        :effect (and (sealing-mode-on ?r) (not (sealing-mode-off ?r)))
    )

    ;; ACTION: deactivate-seal
    ;; PURPOSE: Opens the robot's external hatches/interfaces.
    ;; CONSTRAINT: Only allowed in pressurized rooms to prevent internal damage.
    (:action deactivate-seal
        :parameters (?r - robot ?l - location)
        :precondition (and 
            (robot-at ?r ?l)
            (is-pressurized ?l) ; Safety check: Cannot vent in a tunnel
            (sealing-mode-on ?r)
        )
        :effect (and (sealing-mode-off ?r) (not (sealing-mode-on ?r)))
    )

    ;; =========================================================================
    ;; LOGISTICS: POD HANDLING
    ;; =========================================================================

    ;; ACTION: pick-up-empty-pod
    ;; PURPOSE: Grabs a free pod from the room.
    ;; TRANSITION: hands empty -> carrying empty pod
    (:action pick-up-empty-pod
        :parameters (?r - robot ?l - location ?p - pod)
        :precondition (and 
            (robot-at ?r ?l)            ;; Robot and Pod should be 
            (pod-at ?p ?l)              ;; in the same location
            (pod-empty ?p)              ;; the pod is empty
            (hands-empty ?r)            ;; the robot have empty hands 
        )
        :effect (and 
            ;; Now the robot carrying an empty pod
            (not (hands-empty ?r))      
            (carrying-empty-pod ?r ?p)  
            ;; The pod is not more in that room but carried by the robot
            (not (pod-at ?p ?l))
        )
    )

    ;; ACTION: pick-up-full-pod
    ;; PURPOSE: Picks up a pod that already contains an artifact.
    ;; TRANSITION: hands empty -> carrying full pod 
    (:action pick-up-full-pod
        :parameters (?r - robot ?l - location ?p - pod ?a - artifact)
        :precondition (and 
            (robot-at ?r ?l)
            (pod-at ?p ?l)
            (pod-contains ?p ?a)
        )
        :effect (and 
            ;; Now the robot carrying a full pod
            (not (hands-empty ?r))
            (carrying-full-pod ?r ?p)
            (carrying ?r ?a)
            ;; The pod is not more in that room but carried by the robot
            (not (pod-at ?p ?l))
        )
    )

    ;; ACTION: drop-empty-pod
    ;; PURPOSE: Frees hands by dropping an empty pod.
    ;; TRANSITION: carrying empty pod -> hands empty
    (:action drop-empty-pod
        :parameters (?r - robot ?p - pod ?l - location)
        :precondition (and 
            (carrying-empty-pod ?r ?p)
            (robot-at ?r ?l)
            (pod-empty ?p)
        )
        :effect (and 
            ;; Now the robot no more is carrying an empty pod, so it has empty hands
            (not (carrying-empty-pod ?r ?p))
            (hands-empty ?r)
            ;; The pod now is not more in the hands of the robot but in the rooms where it has been dropped
            (pod-at ?p ?l)
        )
    )

    ;; ACTION: drop-full-pod
    ;; PURPOSE: Drops a pod containing an artifact (without opening it).
    ;; TRANSITION: carrying full pod -> hands empty 
    (:action drop-full-pod
        :parameters (?r - robot ?p - pod ?l - location ?a - artifact)
        :precondition (and 
            (carrying-full-pod ?r ?p)
            (carrying ?r ?a)
            (robot-at ?r ?l)
            (pod-contains ?p ?a)
        )
        :effect (and 
            (not (carrying-full-pod ?r ?p))
            (not (carrying ?r ?a)) ; Robot no longer holds artifact directly
            (hands-empty ?r)
            (contains-full-pod ?l ?p)
            ;; (artifact-at ?a ?l)    ; Artifact location updates to room
            (pod-at ?p ?l)
        )
    )

    ;; =========================================================================
    ;; LOGISTICS: ARTIFACT HANDLING
    ;; =========================================================================

    ;; ACTION: pick-up-artifact-standard
    ;; PURPOSE: Direct pickup of non-fragile items.
    ;; TRANSITION: 
    (:action pick-up-artifact-standard
        :parameters (?a - artifact ?l - location ?r - robot)
        :precondition (and 
            (robot-at ?r ?l)
            (artifact-at ?a ?l)
            (hands-empty ?r)
            (no-fragile ?a)
        )
        :effect (and 
            (not (hands-empty ?r)) 
            (carrying ?r ?a)            
            (not (artifact-at ?a ?l))
        )
    )

    ;; ACTION: put-in-pod
    ;; PURPOSE: Encapsulates an artifact into a carried empty pod.
    ;; TRANSITION: 
    (:action put-in-pod
        :parameters (?a - artifact ?l - location ?r - robot ?p - pod)
        :precondition (and 
            (robot-at ?r ?l)
            (artifact-at ?a ?l)
            (carrying-empty-pod ?r ?p)
            (pod-empty ?p)
        )
        :effect (and 
            (not (carrying-empty-pod ?r ?p))
            (carrying-full-pod ?r ?p)  ; Transition to 'Full Pod' state
            (carrying ?r ?a)
            (not (artifact-at ?a ?l))
            (not (pod-empty ?p)) (pod-full ?p)
            (pod-contains ?p ?a)
        )
    )

    ;; ACTION: release-artifact (Standard)
    ;; PURPOSE: Drops a non-fragile item directly.
    ;; TRANSITION: 
    (:action release-artifact
        :parameters (?r - robot ?a - artifact ?l - location)
        :precondition (and 
            (robot-at ?r ?l) 
            (carrying ?r ?a)
            (is-standard-room ?l)
            (no-fragile ?a) ; Constraint: Cannot drop fragile items without pod
        )
        :effect (and 
            (not (carrying ?r ?a)) 
            (not (hands-full ?r)) (hands-empty ?r) 
            (artifact-at ?a ?l)
        )
    )

    ;; ACTION: release-artifact-from-pod
    ;; PURPOSE: Unpacks an artifact into a room. Robot KEEPS the empty pod.
    ;; TRANSITION: 
    (:action release-artifact-from-pod
        :parameters (?r - robot ?a - artifact ?l - location ?p - pod)
        :precondition (and
            (robot-at ?r ?l)
            (carrying-full-pod ?r ?p)
            (pod-contains ?p ?a)
            (is-standard-room ?l)
        )
        :effect (and
            (not (carrying-full-pod ?r ?p))
            (carrying-empty-pod ?r ?p) ; Hands remain full, pod empty
            (not (carrying ?r ?a))
            (artifact-at ?a ?l)
            (not (pod-contains ?p ?a))
            (pod-empty ?p) (not (pod-full ?p))
        )
    )

    ;; ACTION: release-artifact-in-cryo
    ;; PURPOSE: Drops non-fragile item in cryo-chamber (Temperature Change).
    (:action release-artifact-in-cryo
        :parameters (?r - robot ?a - artifact ?l - location)
        :precondition (and 
            (robot-at ?r ?l) 
            (carrying ?r ?a)
            (is-chill-room ?l)
            (no-fragile ?a)
        )
        :effect (and 
            (not (carrying ?r ?a)) 
            (not (hands-full ?r)) (hands-empty ?r) 
            (artifact-at ?a ?l)
            (not (warm ?a)) (cold ?a) ; Instant Temp Logic
        )
    )

    ;; ACTION: release-artifact-in-cryo-from-pod
    ;; PURPOSE: Unpacks artifact in cryo-chamber. Robot KEEPS empty pod.
    (:action release-artifact-in-cryo-from-pod
        :parameters (?r - robot ?a - artifact ?l - location ?p - pod)
        :precondition (and 
            (robot-at ?r ?l)
            (carrying-full-pod ?r ?p)
            (pod-contains ?p ?a)
            (is-chill-room ?l)
        )
        :effect (and 
            (not (carrying-full-pod ?r ?p))
            (carrying-empty-pod ?r ?p)
            (artifact-at ?a ?l)
            (not (pod-contains ?p ?a))
            (pod-empty ?p) (not (pod-full ?p))
            (not (warm ?a)) (cold ?a)
        )
    )
)