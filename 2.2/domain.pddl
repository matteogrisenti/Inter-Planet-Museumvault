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
    (hands-empty ?r - robot)
    (sealing-mode ?r - robot)                       ; Required for unpressurized zones
    
    ;; --- POSSESSION ---
    (carrying ?r - robot ?a - artifact)             ; Robot is carrying artifact ?a
    (carrying-empty-pod ?r - robot ?p - pod)        ; Robot is holding an empty pod
    (carrying-full-pod ?r - robot ?p - pod)         ; Robot is holding a pod with an artifact inside
    (carrying-second-object ?r - robot ?a - artifact) ; For Technician (second slot)
    (second-slot-empty ?r - robot)                  ; Status of the Technician's second slot

    ;; --- CAPABILITIES ---
    (can-access ?r - robot ?l - location)           ; Access permissions per room
    (can-pickup ?r - robot ?at - artifact-type)  ; Permissions per artifact type
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
    
    (contains-empty-pod ?l - location ?p - pod)
    (contains-full-pod ?l - location ?p - pod)
    (pod-empty ?p - pod)
    (pod-contains ?p - pod ?a - artifact)           ; Artifact-pod association
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
          (can-access ?r ?to)
          (connected ?from ?to) 
          (is-seismic ?to)
          ; (not (sealing-mode ?r)) ;; this is used just for travelling Tunnel to room B
        )
      :effect (and 
          (oneof 
              (and (is-safe ?to) (not (robot-at ?r ?from)) (robot-at ?r ?to) (not (sealing-mode ?r)))          ;; CASE A: Room is safe
              (and (not (is-safe ?to)))    ;; CASE B: Room is unsafe
          )
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
;   (:action deactivate-seal
;     :parameters (?r - robot)
;     :precondition (and (sealing-mode ?r))
;     :effect (not (sealing-mode ?r))
;   )


  ;; POD MANAGEMENT: EQUIPPING & UNEQUIPPING
  ;; 1. Pick Up Pod
  ;; Transition: hands-empty -> carrying-empty-pod
  (:action pick-up-empty-pod
    :parameters (?r - robot ?l - location ?p - pod)
    :precondition (and 
        (robot-at ?r ?l)
        (contains-empty-pod ?l ?p)       ; Must be in room the contain free pod
        (pod-empty ?p)                ; Pod must be empty
        (hands-empty ?r)                ; Must have free hands
    )
    :effect (and 
        (not (contains-empty-pod ?l ?p))
        (not (hands-empty ?r))
        (carrying-empty-pod ?r ?p)
    )
  )

  (:action pick-up-full-pod
      :parameters (?r - robot ?l - location ?p - pod ?a - artifact ?at - artifact-type)
      :precondition (and 
          (robot-at ?r ?l)
          (contains-full-pod ?l ?p)
          (pod-contains ?p ?a)
          (hands-empty ?r)
          (can-pickup ?r ?at)
          (is-type  ?a ?at)
      )
      :effect (and 
          (not (hands-empty ?r))
          (carrying-full-pod ?r ?p)
          (not (contains-full-pod ?l ?p))
      )
  )
  

  ;; 2. Drop Pod
  ;; Transition: carrying-empty-pod -> hands-empty
  ;; Useful if we need to free hands to move a non-fragile item normally
  (:action drop-empty-pod
    :parameters (?r - robot ?p - pod ?l - location)
    :precondition (and 
        (carrying-empty-pod ?r ?p)
        (robot-at ?r ?l)
        (pod-empty ?p)
    )
    :effect (and 
        (not (carrying-empty-pod ?r ?p))
        (hands-empty ?r)
        (contains-empty-pod ?l ?p)       ; Pod is now available in the room
    )
  )

  ;; 3. Drop Full Pod (with artifact inside)
 (:action drop-full-pod
     :parameters (?r - robot ?p - pod ?l - location)
     :precondition (and 
         (carrying-full-pod ?r ?p)
         (robot-at ?r ?l)
     )
     :effect (and 
         (not (carrying-full-pod ?r ?p))
         (hands-empty ?r)
         (contains-full-pod ?l ?p)       ; Pod is now available in the room
     )
 )
 
  
  ;; PICK UP ARTIFACT ACTIONS

  ;; 1. STANDARD PICK UP
  ;; Transition: hands-empty -> carrying ?a
  ;; Used for sturdy items. If you pick up a fragile item this way, 
  ;; you won't be able to move it (due to move constraints).
  (:action pick-up-artifact-standard
    :parameters (?a - artifact ?at - artifact-type ?l - location ?r - robot)
    :precondition (and 
        (robot-at ?r ?l)            ; Robot and Artifact in the same 
        (artifact-at ?a ?l)      ; location
        (hands-empty ?r)             ; Robot free
        (no-fragile  ?a)
        (can-pickup ?r ?at)
        (is-type  ?a ?at)
    )
    :effect (and 
        (not (hands-empty ?r)) 
        (carrying ?r ?a)            
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
        (artifact-at ?a ?l)      ; location
        (carrying-empty-pod ?r ?p)    ; Robot carrying already an empty pod
        (pod-empty ?p)         ; Pod is empty
        (can-pickup ?r ?at)
        (is-type  ?a ?at)
    )
    :effect (and 
        (not (carrying-empty-pod ?r ?p))
        (carrying-full-pod ?r ?p)    
        (not (artifact-at ?a ?l))
        (not (pod-empty ?p))
        (pod-contains ?p ?a)
    )
  )


  ;; DROP DOWN ACTIONS
  ;; A. Dropping to Standard Rooms
  ;; 1. Standard Artifact Drop 
  ;; Transition: carrying ?a -> hands-empty
  (:action release-artifact
    :parameters (?r - robot ?a - artifact ?l - location)
    :precondition (and 
        (robot-at ?r ?l) 
        (carrying ?r ?a)
        (is-standard-room ?l)
        ; (no-fragile ?a)                 ; Fragile items can't be dropped like this, they need to be put in a pod first (handled by move constraints
    )
    :effect (and 
        (not (carrying ?r ?a)) 
        (hands-empty ?r) 
        (artifact-at ?a ?l)
    )
  )

  ;; 2. Unload From Pod (Standard Room)
  ;; Transition: carrying-full-pod -> carrying-empty-pod
  (:action release-artifact-from-pod
    :parameters (?r - robot ?a - artifact ?l - location ?p - pod) ;; Aggiunto ?p
    :precondition (and
        (robot-at ?r ?l)
        (carrying-full-pod ?r ?p)  
        (pod-contains ?p ?a)       
        (is-standard-room ?l)
    )
    :effect (and
        (not (carrying-full-pod ?r ?p))
        (not (pod-contains ?p ?a)) ; Pod is now empty
        (carrying-empty-pod ?r ?p)      ; Robot still holds the empty pod
        (artifact-at ?a ?l)        ; Artifact is now at the location
        (pod-empty ?p)            ; Pod is marked as empty
    )
  )

  ;; B. Dropping to Cryo-Chamber (Temperature Effect)
  ;; 3. Cryo Drop (From Bare Hands)
  ;; Transition: carrying ?a -> hands-empty
  (:action release-artifact-in-cryo                       ; robot releases a non-fragil artifact in the cryo-chamber and it becomes cold
    :parameters (?r - robot ?a - artifact ?l - location)
    :precondition (and 
        (robot-at ?r ?l) 
        (carrying ?r ?a)
        (is-chill-room ?l)
        (warm  ?a )
    )
    :effect (and 
        (not (carrying ?r ?a)) 
        (hands-empty ?r) 
        (artifact-at ?a ?l)
        (not (warm ?a)) (cold ?a) ; Instant Temp Logic
    )
  )

  ;; 4. Cryo Unload (From Pod)
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
        (carrying-empty-pod ?r ?p)     ; Pod remains with robot
        (artifact-at ?a ?l)
        (not (pod-contains ?p ?a))
        (pod-empty ?p)
        (not (warm ?a)) (cold ?a) ; Instant Temp Logic
    )
  )

  ;; * Second object slot actions (Technician Robot Only) - Pickup and Release
  
  ; Pick-up action
  (:action pick-up-second-object
      :parameters (
            ?r - robot ?a - artifact ?at - artifact-type ?l - location
      )
      :precondition (and 
            (robot-at ?r ?l)
            (can-carry-two ?r)
            (artifact-at ?a ?l)
            (second-slot-empty ?r)           ; Second slot must be empty
            (can-pickup ?r ?at)
            (is-type  ?a ?at)
            (no-fragile  ?a)                 ; The second slot can contain non-fragile objects only (so it cannot take 2 pods simultaneously)
        )
      :effect (and 
          (not (second-slot-empty ?r))
          (carrying-second-object ?r ?a)
          (not (artifact-at ?a ?l))
      )
  )

  (:action release-second-object
      :parameters (?r - robot ?a - artifact ?l - location)
      :precondition (and 
            (robot-at ?r ?l)
            (carrying-second-object ?r ?a)
        )
      :effect (and 
          (second-slot-empty ?r)
          (not (carrying-second-object ?r ?a))
          (artifact-at ?a ?l)
      )
  )
  
)