(define (domain multi-robot)
    (:requirements :strips :typing :non-deterministic)

    ;; Types derived from scenario entities 
    ; The robot is not a type, becouse in this scenario we have only one robot
    ; so it can be modelled as one object in the problem file.
    (:types
        robot             ; Agents (Curator, Technician, Scientist)
        pod               ; Anti-vibration containers
        location          ; Rooms and tunnels
        artifact          ; Items to be retrieved
        artifact-type     ; Categories (scientific, technological, top-secret)
    )

    (:predicates
        ;; --- ROBOT STATE ---
        (robot-at ?r - robot ?l - location)
        (sealing-mode ?r - robot)                       ; Required for unpressurized zones
        
        ;; --- POSSESSION ---
        ; Slot 1: can hold pods or artifacts
        (hands-empty-slot-1 ?r - robot)
        (carrying-slot-1 ?r - robot ?a - artifact)              ; Robot is carrying artifact ?a
        (carrying-pod-slot-1 ?r - robot ?p - pod)               ; Robot is holding a pod
        
        ; Slot 2: can hold only artifacts
        (hands-empty-slot-2 ?r - robot)
        (carrying-slot-2 ?r - robot ?a - artifact)             

        ;; --- CAPABILITIES ---
        (can-access ?r - robot ?l - location)           ; Access permissions per room
        (can-pickup ?r - robot ?at - artifact-type)     ; Permissions per artifact type
        (can-carry-two ?r - robot)                      ; Flag for Technician-type robots

        
        ;; --- ENVIRONMENTAL PROPERTIES ---
        (connected ?l1 ?l2 - location)
        (is-pressurized ?l - location)
        (is-unpressurized ?l - location)
        (is-safe ?l - location)                         ; Room status (safe to enter)
        (is-seismic ?l - location)                      ; Triggers safety checks
        (is-standard-room ?l - location)                ; Normal storage/drop rooms
        (is-chill-room ?l - location)                   ; Rooms that cool artifacts (Cryo-Chamber)

        ;; --- ARTIFACT & POD STATUS ---
        (artifact-at ?a - artifact ?l - location)
        (is-type ?a - artifact ?t - artifact-type)
        (fragile ?a - artifact)
        (no-fragile ?a - artifact)
        (warm ?a - artifact)
        (cold ?a - artifact)
        
        (pod-empty ?p - pod)
        (pod-contains ?p - pod ?a - artifact)           ; Artifact-pod association
        (pod-at ?p - pod ?l - location)
    )

    
    ;; ========================
    ;; ACTIONS 
    ;; ========================

    ;; MOVEMENT
    ;; tries to move to a room without checking its safety status. If the room is safe, the robot can move there, otherwise it can't.
    (:action try-to-enter-seismic-room
        :parameters (?r - robot ?to ?from - location)
        :precondition (and 
            (robot-at ?r ?from) 
            (connected ?from ?to) 
            (is-seismic ?to)
            (can-access ?r ?to)
        )
        ;; NON DETERMINISTIC EFFECT
        :effect (oneof 
            ;; CASE A: Room is safe
            (and 
                (is-safe ?to) 
                (not (robot-at ?r ?from)) (robot-at ?r ?to)
                ; the sealing mode is deactivated automatically
                (not (sealing-mode ?r))    
            )      
            ;; CASE B: Room is unsafe
            (not (is-safe ?to))    
        )
    )

    ;; MOVEMENT

    ;; A: Moving Empty (No artifact constraints) -> No artifact position updade
    ;; 1. Move Empty to a Safe Room (No sealing needed)
    (:action move-to-pressurized-room
        :parameters (?r - robot ?from ?to - location)
        :precondition (and 
            (robot-at ?r ?from)
            (connected ?from ?to)
            (is-pressurized ?to)       ;; Target is pressurized 
            (is-safe ?to)              ;; Target is safe
            (can-access ?r ?to)
        )
        :effect (and 
            (not (robot-at ?r ?from)) 
            (robot-at ?r ?to)
            (not (sealing-mode ?r))
        )
    )

    ;; 2. Move Empty to Tunnel (Requires Sealing)
    (:action move-to-unpressurized-room
        :parameters (?r - robot ?from ?to - location)
        :precondition (and 
            (robot-at ?r ?from)
            (connected ?from ?to)
            (is-unpressurized ?to)     ;; Target is unpressurized
            (sealing-mode ?r)             ;; Constraint: Must be sealed
            ; (is-safe ?to)              ;; Target is safe (tunnel is always safe, so no need to check)
            (can-access ?r ?to)         ; also this could be skipped (since everybody can access the tunnel) but we can keep it for consistency with the other move actions
        )
        :effect (and (not (robot-at ?r ?from)) (robot-at ?r ?to))
    )


    ;; SEALING MECHANISM
    ;; Robot activates sealing mode. Can be done anywhere.
    (:action activate-seal
        :parameters (?r - robot)
        :precondition ()
        :effect (sealing-mode ?r)
    )
    (:action deactivate-seal
        :parameters (?r - robot ?l - location)
        :precondition (and 
            (robot-at ?r ?l)
            (is-pressurized ?l) ; Safety check: Cannot vent in a tunnel
            (sealing-mode ?r)
        )
        :effect (not (sealing-mode ?r))
    )


    ;; POD MANAGEMENT: EQUIPPING & UNEQUIPPING
    ;; 1. Pick Up Pod
    ;; Transition: hands-empty -> carrying-pod
    (:action pick-up-pod-slot-1
        :parameters (?r - robot ?l - location ?p - pod)
        :precondition (and 
            (robot-at ?r ?l)
            (pod-at ?p ?l)
            (hands-empty-slot-1 ?r)                ; Must have free hands
        )
        :effect (and 
            (not (pod-at ?p ?l))
            (not (hands-empty-slot-1 ?r))
            (carrying-pod-slot-1 ?r ?p)
        )
    )
    

    ;; 2. Drop Pod
    ;; Transition: carrying-empty-pod -> hands-empty
    ;; Useful if we need to free hands to move a non-fragile item normally
    (:action drop-pod-slot-1
        :parameters (?r - robot ?p - pod ?l - location)
        :precondition (and 
            (carrying-pod-slot-1 ?r ?p)
            (robot-at ?r ?l)
        )
        :effect (and 
            (not (carrying-pod-slot-1 ?r ?p))
            (hands-empty-slot-1 ?r)
            (pod-at ?p ?l)       ; Pod is now available in the room
        )
    )
    
    
    ;; PICK UP ARTIFACT ACTIONS

    ;; 1. STANDARD PICK UP
    ;; Transition: hands-empty -> carrying ?a
    ;; Used for sturdy items. If you pick up a fragile item this way, 
    ;; you won't be able to move it (due to move constraints).
    (:action pick-up-slot-1
        :parameters (?a - artifact ?at - artifact-type ?l - location ?r - robot)
        :precondition (and 
            (robot-at ?r ?l)                    ; Robot and Artifact in the same 
            (artifact-at ?a ?l)                 ; location
            (hands-empty-slot-1 ?r)             ; Robot free
            (no-fragile  ?a)
            (can-pickup ?r ?at)
            (is-type  ?a ?at)
        )
        :effect (and 
            (not (hands-empty-slot-1 ?r)) 
            (carrying-slot-1 ?r ?a)            
            (not (artifact-at ?a ?l))
        )
    )

    (:action pick-up-slot-2
        :parameters (?a - artifact ?at - artifact-type ?l - location ?r - robot)
        :precondition (and 
            (robot-at ?r ?l)            ; Robot and Artifact in the same 
            (can-carry-two ?r)
            (artifact-at ?a ?l)      ; location
            (hands-empty-slot-2 ?r)             ; Robot free
            (no-fragile  ?a)
            (can-pickup ?r ?at)
            (is-type  ?a ?at)
        )
        :effect (and 
            (not (hands-empty-slot-2 ?r)) 
            (carrying-slot-2 ?r ?a)            
            (not (artifact-at ?a ?l))
        )
    )

    ;; 4. PUT IN POD (Load into Pod)
    ;; Transition: carrying-empty-pod -> carrying-in-pod ?a
    ;; This action "secures" the artifact immediately.
    (:action put-in-pod
        :parameters (?a - artifact ?at - artifact-type ?l - location ?r - robot ?p - pod)
        :precondition (and 
            (robot-at ?r ?l)            ; Robot and Artifact in the same 
            (artifact-at ?a ?l)         ; location
            (carrying-pod-slot-1 ?r ?p)     ; Robot carrying already an empty pod
            (pod-empty ?p)                  ; Pod is empty
            (can-pickup ?r ?at)
            (is-type  ?a ?at)
        )
        :effect (and 
            (not (artifact-at ?a ?l))
            (not (pod-empty ?p))
            (pod-contains ?p ?a)
        )
    )


    ;; DROP DOWN ACTIONS
    ;; A. Dropping to Standard Rooms
    ;; 1. Standard Artifact Drop 
    ;; Transition: carrying ?a -> hands-empty
    (:action release-artifact-slot-1
        :parameters (?r - robot ?a - artifact ?l - location)
        :precondition (and 
            (robot-at ?r ?l) 
            (carrying-slot-1 ?r ?a)
            (is-standard-room ?l)
        )
        :effect (and 
            (not (carrying-slot-1 ?r ?a)) 
            (hands-empty-slot-1 ?r) 
            (artifact-at ?a ?l)
        )
    )

        (:action release-artifact-slot-2
        :parameters (?r - robot ?a - artifact ?l - location)
        :precondition (and 
            (robot-at ?r ?l) 
            (can-carry-two ?r)
            (carrying-slot-2 ?r ?a)
            (is-standard-room ?l)
        )
        :effect (and 
            (not (carrying-slot-2 ?r ?a)) 
            (hands-empty-slot-2 ?r) 
            (artifact-at ?a ?l)
        )
    )

    ;; 2. Unload From Pod (Standard Room)
    ;; Transition: carrying-full-pod -> carrying-empty-pod
    (:action release-artifact-from-pod-slot-1
        :parameters (?r - robot ?a - artifact ?l - location ?p - pod) ;; Aggiunto ?p
        :precondition (and
            (robot-at ?r ?l)
            (carrying-pod-slot-1 ?r ?p)  
            (pod-contains ?p ?a)       
            (is-standard-room ?l)
        )
        :effect (and
            (not (pod-contains ?p ?a))  ; Pod is now empty
            (artifact-at ?a ?l)        ; Artifact is now at the location
            (pod-empty ?p)            ; Pod is marked as empty
        )
    )

    ;; B. Dropping to Cryo-Chamber (Temperature Effect)
    ;; 3. Cryo Drop (From Bare Hands)
    ;; Transition: carrying ?a -> hands-empty
    (:action release-artifact-cryo-slot-1                      ; robot releases a non-fragil artifact in the cryo-chamber and it becomes cold
        :parameters (?r - robot ?a - artifact ?l - location)
        :precondition (and 
            (robot-at ?r ?l) 
            (carrying-slot-1 ?r ?a)
            (is-chill-room ?l)
        )
        :effect (and 
            (not (carrying-slot-1 ?r ?a)) 
            (hands-empty-slot-1 ?r) 
            (artifact-at ?a ?l)
            (not (warm ?a)) (cold ?a) ; Instant Temp Logic
        )
    )

    (:action release-artifact-cryo-slot-2                      ; robot releases a non-fragil artifact in the cryo-chamber and it becomes cold
        :parameters (?r - robot ?a - artifact ?l - location)
        :precondition (and 
            (robot-at ?r ?l) 
            (can-carry-two ?r)
            (carrying-slot-2 ?r ?a)
            (is-chill-room ?l)
        )
        :effect (and 
            (not (carrying-slot-2 ?r ?a)) 
            (hands-empty-slot-2 ?r) 
            (artifact-at ?a ?l)
            (not (warm ?a)) (cold ?a) ; Instant Temp Logic
        )
    )

    ;; 4. Cryo Unload (From Pod)
    (:action release-artifact-cryo-from-pod-slot-1
        :parameters (?r - robot ?a - artifact ?l - location ?p - pod)
        :precondition (and 
            (robot-at ?r ?l)
            (carrying-pod-slot-1 ?r ?p)
            (pod-contains ?p ?a)
            (is-chill-room ?l)
        )
        :effect (and 
            (artifact-at ?a ?l)
            (not (pod-contains ?p ?a))
            (pod-empty ?p)
            (not (warm ?a)) (cold ?a) ; Instant Temp Logic
        )
    )
)