(define (domain temporal-multi-robot)
  (:requirements :strips :typing :non-deterministic :durative-actions)

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
    (can-pickup ?r - robot ?at - artifact-type)     ; Permissions per artifact type
    (can-carry-two ?r - robot)                      ; Flag for Technician-type robots

    ;; --- ENVIRONMENTAL PROPERTIES ---
    (connected ?l1 ?l2 - location)
    (is-pressurized ?l - location)
    (is-unpressurized ?l - location)
    (is-safe ?l - location)                      ; ! Room safety status (this is used to determine if the robot can enter or not, 
                                                        ; !but it is not deterministic, so we have to check it every time we try to enter a room)
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

  ;; MOVEMENT ACTIONS

  ;; A: Moving Empty (No artifact constraints) -> No artifact position updade
  ;; 1. Move Empty to a Safe Room (No sealing needed)
  (:durative-action move-to-pressurized-room
    :parameters (?r - robot ?from ?to - location)
    :duration (= ?duration 5)
    :condition (and 
        (at start (robot-at ?r ?from))   
        (over all (connected ?from ?to)) 
        (over all (is-pressurized ?to))     ;; Target is pressurized at all times during the move
        (over all (is-safe ?to))              ;; Target must be safe at all times during the move (this is what allows us to avoid the try-to-enter-seismic-room action)
        (over all (can-access ?r ?to))
    )
    :effect (and 
        (at start (not (robot-at ?r ?from))) ; first remove robot from original location
        (at end (robot-at ?r ?to))          ; then place it in the new location when reached
        (at end (not (sealing-mode ?r)))   ; when enters room the sealing mode automatically turns off
    )
  )

  ;; 2. Move Empty to Tunnel (Requires Sealing)
  (:durative-action move-to-unpressurized-room
    :parameters (?r - robot ?from ?to - location)
    :duration (= ?duration 5)
    :condition (and 
        (at start (robot-at ?r ?from))   
        (at start (sealing-mode ?r))             ;; Constraint: Must be sealed
        (over all (connected ?from ?to)) 
        (over all (is-unpressurized ?to))     ;; Target is unpressurized
        (over all (is-safe ?to))              ;; Target is safe
        (over all (can-access ?r ?to))
    )
    :effect (and 
        (at start (not (robot-at ?r ?from))) 
        (at end (robot-at ?r ?to))
  )


  ;; SEALING MECHANISM
  ;; Robot activates sealing mode. Can be done anywhere.
  (:durative-action activate-seal
    :parameters (?r - robot)
    :duration (= ?duration 2)
    :condition (at start (not (sealing-mode ?r)))
    :effect (at end (sealing-mode ?r))
  )
  (:durative-action deactivate-seal ;; probably unused due to the automatic deactivation when entering pressurized rooms, but added for completeness
    :parameters (?r - robot)
    :duration (= ?duration 2)
    :condition (at start (sealing-mode ?r))
    :effect (at end (not (sealing-mode ?r)))
  )


  ;; POD MANAGEMENT: EQUIPPING & UNEQUIPPING
  ;; 1. Pick Up Pod
  ;; Transition: hands-empty -> carrying-empty-pod
  (:durative-action pick-up-empty-pod
    :parameters (?r - robot ?l - location ?p - pod)
    :duration (= ?duration 3)
    :condition (and 
        (at start (pod-empty ?p))                  ; Pod must be empty
        (at start (hands-empty ?r))                ; Must have free hands
        (over all (robot-at ?r ?l))                ; Robot must be at the location for the entire duration
        (over all (contains-empty-pod ?l ?p))      ; Must be in room the contain free pod
    )
    :effect (and 
        (at start (not (contains-empty-pod ?l ?p))) ; Pod is no longer available in the room (no other robot can take it while it is picked up)
        (at start (not (hands-empty ?r)))           ; hands are not empty anomer (it can't take another objects meanwhile)
        (at end (carrying-empty-pod ?r ?p))         ; Robot is now carrying the empty pod
    )
  )

  (:durative-action pick-up-full-pod
      :parameters (?r - robot ?l - location ?p - pod ?a - artifact ?at - artifact-type)
      :duration (= ?duration 3)
      :condition (and 
          (at start (contains-full-pod ?l ?p))      ; Pod must be in the room at the start
          (at start (hands-empty ?r))               ; Must have free hands at the start
          (over all (pod-contains ?p ?a))           ; Pod must contain the artifact for the entire duration
          (over all (robot-at ?r ?l))               ; Robot must be at the location for the entire duration;
          (over all (can-pickup ?r ?at))            ; Robot must be able to pick up the artifact type for the entire duration
          (over all (is-type  ?a ?at))              ; Artifact must be of the correct type for the entire duration
      )
      :effect (and 
          (at start (not (hands-empty ?r)))              ; Robot's hands are no longer empty at the start (cannot take other objects while it is picking up something)
          (at start (not (contains-full-pod ?l ?p)))     ; Pod is no longer available in the room (no other robot can take it while it is picked up)
          (at end (carrying-full-pod ?r ?p))             ; Robot is now carrying the full pod at the end
          (at end (carrying ?r ?a))                      ; Robot is now also carrying the artifact at the end
      )
  )
  

  ;; 2. Drop Pod
  (:durative-action drop-empty-pod
    :parameters (?r - robot ?p - pod ?l - location)
    :duration (= ?duration 3)
    :condition (and 
        (at start (carrying-empty-pod ?r ?p))      ; Robot must be carrying the empty pod at the start
        (over all (pod-empty ?p))            ; Pod must not be empty at the start (it is being dropped, so it will become available in the room)
        (over all (robot-at ?r ?l))                ; Robot must be at the location for the entire duration
    )
    :effect (and 
        (at start (not (carrying-empty-pod ?r ?p)))
        (at end (hands-empty ?r))
        (at end (contains-empty-pod ?l ?p))       ; Pod is now available in the room
    )
  )

  ;; 3. Drop Full Pod (with artifact inside)
 (:durative-action drop-full-pod
     :parameters (?r - robot ?p - pod ?l - location ?a - artifact)
     :duration (= ?duration 3)
     :condition (and 
         (at start (carrying-full-pod ?r ?p))           ; Robot must be carrying the full pod at the start
         (at start (carrying ?r ?a))                    ; Robot must be carrying the artifact for the entire duration (it is being dropped, so it will become available in the room)
         (over all (robot-at ?r ?l))                    ; Robot must be at the location for the entire duration
         (over all (not (pod-empty ?p)))                ; Pod must not be empty along the entire duration (otherwise one could take the object inside)
         (over all (pod-contains ?p ?a))                ; The contained object must be kept inside the pod
     )
     :effect (and 
         (at start (not (carrying-full-pod ?r ?p)))     ; Robot is no longer carrying the full pod at the start
         (at end (hands-empty ?r))                      ; Robot's hands are empty at the end
         (at end (contains-full-pod ?l ?p))             ; Full pod is now available in the room
         (at end (artifact-at ?a ?l))                   ; Artifact is now at the location
     )
 )
 
  
  ;; PICK UP ARTIFACT ACTIONS

  ;; 1. STANDARD PICK UP
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
        (carrying ?r ?a)     
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
        (carrying-empty-pod ?r ?p)      ; Robot still holds the empty pod
         (not (carrying ?r ?a))     ; Robot no longer carries the artifact
         (artifact-at ?a ?l)        ; Artifact is now at the location
         (not (pod-contains ?p ?a)) ; Pod is now empty
         (pod-empty ?p)            ; Pod is marked as empty
        (not (carrying ?r ?a))
        (artifact-at ?a ?l)
        (not (pod-contains ?p ?a))
        (pod-empty ?p)
    )
  ).

  ;; B. Dropping to Cryo-Chamber (Temperature Effect)
  ;; 3. Cryo Drop (From Bare Hands)
  ;; Transition: carrying ?a -> hands-empty
  (:action release-artifact-in-cryo
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
            (no-fragile  ?a)
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