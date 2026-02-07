(define (domain single-robot)
  (:requirements :strips :typing :non-deterministic) ;; ! ADDED NON-DETERMINISTIC REQUIREMENT FOR THE DROP ACTIONS

  ;; Types derived from scenario entities 
  ; The robot is not a type, becouse in this scenario we have only one robot
  ; so it can be modelled as one object in the problem file.
  (:types
    robot             ; The unique robot in the environment (but can be extended to multiple robots in the future)
    pod               ; Anti-vibration pod: used to secure fragile artifacts during transportation
    location          ; Locations: the different rooms in the enviroment
    artifact          ; Artifacts: the different artifact in the enviroment
    artifact-type     ; Artifacts Tipologies: the different tipologies of artifacts in the enviroment
  )

  (:predicates
    ;; Robot Features - they all refears to the unique robot in the enviroment
    (robot-at ?r - robot ?l - location)                    ; Robot's current location
    (hands-empty ?r - robot)                                ; Robot's hand status ( true if empty)  
    (carrying ?r - robot ?a - artifact)                    ; Robot is carrying the artifact ?a
    (carrying-full-pod ?r - robot ?p - pod)               ; *Robot has a secure fragile artifact inside an anti vibration pod (this assumes that if has a pod and an artifact, this latter is inside the pod)
    (carrying-empty-pod ?r - robot ?p - pod)                 ; Robot is carrying empty pods (ready to secure fragile artifacts)
    (sealing-mode ?r - robot)                              ; Robot mode (sealing or normal)

    ;; Locations Features
    (checked ?l - location)                     ; ! True if the location has been checked by the robot
    (unchecked ?l - location)                   ; ! True if the location has not been checked by the robot
    (safety-unknown ?l - location)              ; ! True if the safety status is unknown (allows re-checking)
    (connected ?l1 ?l2 - location)            ; Location are linked or not 
    (is-unpressurized ?l - location)          ; True for tuneel
    (is-pressurized   ?l - location)
    (is-unsafe ?l - location)                     ; True if not safe to enter 
    (is-safe ?l - location)                    ; ( Model the mars_quake const for Hall B)
    ; The next flag are used to shape the drop action; which effect change based on the 
    ; nature of the location where the artifact is dropped down.
    (is-standard-room ?l - location)           ; change the location of the dropped artifact
    (is-chill-room ?l - location)              ; change both the locationand the temperature state of the artifact
    ; This feature is required to model the fact that in the pod storage roome there are free
    ; pod to collect
    (contains-empty-pod ?l - location ?p - pod)
    (contains-full-pod ?l - location ?p - pod) ; This feature is required to model the fact that in the pod storage roome there are full
    ; Note 1 : can be a nice think to test the same problem but with only positive 
    ; or negative definition of the last two is- feature:
    ; - (is-pressurized ?l - location)      or      - (is-unpressurized ?l - location)
    ; - (is-safe ?l -location)                      - (is-unsafe ?l -location)
    ; In theory this will require the 'not' operator in the definition of the action to 
    ; the safe or the pressure feature. The 'not' operator need requirement ':negative-preconditions'
    ; which managment in the solver can slow down the resolution. 

    ; Note 2: To increase complexity we can add a capacity for the cryo-chamber room

    ;; Artifacts Features
    (is-type  ?a - artifact ?t - artifact-type)     ; Artifact a belong to the Artifact Tipology t 
    (artifact-at  ?a - artifact ?l - location)      ; Artifact a at location l 
    ; (artifact-carried-by ?a - artifact ?r - robot)  ; Artifact a is carried by the robot (used to model the fact that the artifact is in the hand of the robot)
    ; (carrying-in-pod ?a - artifact)                 ; Artifact a is carried inside
    (fragile  ?a - artifact)                     ; Artifact need to be traveled inside an anti-vibration-pod
    (no-fragile  ?a - artifact )                   
    (cold  ?a - artifact )                       ; Artifact are already cold
    (warm  ?a - artifact )

    ;; Pod Features
    (pod-contains ?p - pod ?a - artifact)           ; Pod p contains artifact a (used to model the fact that the artifact is inside the pod)
    (pod-empty ?p - pod)                            ; Pod is empty and can be used to secure a fragile artifact
    

    ; Note 3: need-chill and need-vibration-pods can be removed becouse coincid with the location feature of the 
    ; artifact becouse both feature depend on where the artifact is situated

  )

  
  ;; ========================
  ;; ACTIONS 
  ;; ========================

  ;; MOVEMENT

  ;; ! Check sismic status of a room: this action is used to check if a room is safe or not.
  (:action check-seismic-status
    :parameters (?r - location ?current - location)
    :precondition (and (robot-at ?current) (connected ?current ?r) (safety-unknown ?r))
    :effect (and 
        (checked ?r)
        (not (safety-unknown ?r))
        (oneof  ;; NON-DETERMINISTIC EFFECT: the result of the check can be either safe or unsafe
            (is-safe ?r)          ;; CASE A: Room is safe
            (not (is-safe ?r))    ;; CASE B: Room is unsafe
        )
    )
  )

  ;; ! Action to wait for seismic window (resetting knowledge to allow retry)
  (:action wait-for-seismic-window
    :parameters (?r - location)
    :precondition (and 
        (checked ?r)       ;; We have checked
        (not (is-safe ?r)) ;; And it is unsafe (earthquake)
    )
    :effect (and 
        (not (checked ?r))      ;; Forget the result
        (safety-unknown ?r)     ;; Reset to unknown
    )
  )

  ;; MOVEMENT





  ;; A: Moving Empty (No artifact constraints) -> No artifact position updade
  ;; 1. Move Empty to a Safe Room (No sealing needed)
  (:action move-empty-safe
    :parameters (?r - robot ?from ?to - location)
    :precondition (and 
        (robot-at ?r ?from)
        (connected ?from ?to)
        (hands-empty ?r)
        (is-pressurized ?to)       ;; Target is pressurized 
        (is-safe ?to)              ;; Target is safe
    )
    :effect (and (not (robot-at ?r ?from)) (robot-at ?r ?to))
  )
  ;; 2. Move Empty to Tunnel (Requires Sealing)
  (:action move-empty-tunnel
    :parameters (?r - robot ?from ?to - location)
    :precondition (and 
        (robot-at ?r ?from)
        (connected ?from ?to)
        (hands-empty ?r)
        (is-unpressurized ?to)     ;; Target is unpressurized
        (sealing-mode)             ;; Constraint: Must be sealed
        (is-safe ?to)              ;; Target is safe
    )
    :effect (and (not (robot-at ?r ?from)) (robot-at ?r ?to))
  )

  ;; B. Moving Standard Artifacts (No pod needed)
  ;; 3. Move Standard Item to Safe Room
  (:action move-carrying-safe
    :parameters (?r - robot ?from ?to - location ?a - artifact)
    :precondition (and 
        (robot-at ?r ?from)
        (connected ?from ?to)
        (carrying ?r ?a)
        (no-fragile ?a)            ;; Ensure it's NOT fragile
        (is-pressurized ?to)       ;; Target is pressurized 
        (is-safe ?to)              ;; Target is safe  
    )
    :effect (and 
        (not (robot-at ?r ?from)) (robot-at ?r ?to) (artifact-at ?a ?to) (not (artifact-at ?a ?from)) ;; ! Move robot and artifact together
    )
  )
  ;; 4. Move Standard Item to Tunnel (Requires Sealing)
  (:action move-carrying-tunnel
    :parameters (?r - robot ?from ?to - location ?a - artifact)
    :precondition (and 
        (robot-at ?r ?from)
        (connected ?from ?to)
        (carrying ?r ?a)
        (no-fragile ?a)            ;; Ensure it's NOT fragile
        (is-unpressurized ?to)     ;; Target is unpressurized
        (sealing-mode)             ;; Constraint: Must be sealed
        (is-safe ?to)              ;; Target is safe
    )
    :effect (and 
        (not (robot-at ?r ?from)) (robot-at ?r ?to) (artifact-at ?a ?to) (not (artifact-at ?a ?from)) ;; ! Move robot and artifact together
    )
  )

  ;; C. Moving Fragile Artifacts (Pod required)
  ;; 5. Move Fragile Item to Safe Room (Requires Pod)
  (:action move-fragile-safe
    :parameters (?r - robot ?p - pod ?from ?to - location ?a - artifact)
    :precondition (and 
        (robot-at ?r ?from)
        (connected ?from ?to)
        ;; Constraint: The fragile asrtifact must put inside an 
        ;; anti-vibration-pod to be secured during trasporation
        (carrying-full-pod ?r ?p)              ;; <=
        (pod-contains ?p ?a)
        (is-pressurized ?to)              ;; Target is pressurized 
        (is-safe ?to)                     ;; Target is safe 
    )
    :effect (and 
        (not (robot-at ?r ?from)) (robot-at ?r ?to) (artifact-at ?a ?to) (not (artifact-at ?a ?from)) ;; ! Move robot and artifact together
    )
  )
  ;; 6. Move Fragile Item to Tunnel (Requires Pod AND Sealing)
  (:action move-fragile-tunnel
    :parameters (?r - robot ?p - pod ?from ?to - location ?a - artifact)
    :precondition (and 
        (robot-at ?from)
        (connected ?from ?to)
        (carrying-full-pod ?r ?p)              ;; <=
        (pod-contains ?p ?a)
        ;; Constraint: The fragile asrtifact must put inside an 
        ;; anti-vibration-pod to be secured during trasporation
        (is-unpressurized ?to)            ;; Target is unpressurized
        (sealing-mode)                    ;; Constraint: Must be sealed
        (is-safe ?to)                     ;; Target is safe
    )
    :effect (and 
        (not (robot-at ?from)) (robot-at ?to) (artifact-at ?a ?to) (not (artifact-at ?a ?from)) ;; ! Move robot and artifact together
    )
  )


  ;; SEALING MECHANISM
  ;; Robot activates sealing mode. Can be done anywhere.
  (:action activate-seal
    :parameters ()
    :precondition ()
    :effect (sealing-mode)
  )
  (:action deactivate-seal
    :parameters ()
    :precondition ()
    :effect (not (sealing-mode))
  )


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
        (not (hands-empty ?r))
        (carrying-empty-pod ?r ?p)
    )
  )

  (:action pick-up-full-pod
      :parameters (?r - robot ?l - location ?p - pod ?a - artifact)
      :precondition (and 
          (robot-at ?r ?l)
          (contains-full-pod ?l ?p)
          (pod-contains ?p ?a)
          (hands-empty ?r)
      )
      :effect (and 
          (not (hands-empty ?r))
          (carrying-full-pod ?r ?p)
          (carrying ?r ?a)
          (not(contains-full-pod ?l ?p))
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

 (:action drop-full-pod
     :parameters (?r - robot ?p - pod ?l - location ?a - artifact)
     :precondition (and 
         (carrying-full-pod ?r ?p)
         (carrying ?r ?a)
         (robot-at ?r ?l)
         (not (pod-empty ?p))
         (pod-contains ?p ?a)
     )
     :effect (and 
         (not (carrying-full-pod ?r ?p))
         (hands-empty ?r)
         (contains-full-pod ?l ?p)       ; Pod is now available in the room
         (artifact-at ?a ?l)             ; Artifact is now at the location
     )
 )
 
  

  ;; PICK UP ACTIONS

  ;; 1. STANDARD PICK UP
  ;; Transition: hands-empty -> carrying ?a
  ;; Used for sturdy items. If you pick up a fragile item this way, 
  ;; you won't be able to move it (due to move constraints).
  (:action pick-up-artifact-standard
    :parameters (?a - artifact ?l - location ?r - robot)
    :precondition (and 
        (robot-at ?r ?l)            ; Robot and Artifact in the same 
        (artifact-at ?a ?l)      ; location
        (hands-empty ?r)             ; Robot free
    )
    :effect (and 
        (not (hands-empty)) 
        (carrying ?a)            
        (not (artifact-at ?a ?l))
    )
  )

  ;; 4. PUT IN POD (Load into Pod)
  ;; Transition: carrying-empty-pod -> carrying-in-pod ?a
  ;; This action "secures" the artifact immediately.
  (:action put-in-pod
    :parameters (?a - artifact ?l - location ?r - robot ?p - pod)
    :precondition (and 
        (robot-at ?l)            ; Robot and Artifact in the same 
        (artifact-at ?a ?l)      ; location
        (carrying-empty-pod ?r ?p)    ; Robot carrying already an empty pod
        (pod-empty ?p)         ; Pod is empty
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
  ;; Transition: carrying-in-pod ?a -> carrying-empty-pod
  ;; Logic: We place the artifact down, but keep the empty pod in hand.
  (:action release-artifact-from-pod
    :parameters (?r - robot ?a - artifact ?l - location)
    :precondition (and 
        (robot-at ?r ?l)
        (carrying-in-pod ?r ?a)
        (is-standard-room ?l)
    )
    :effect (and 
        (not (carrying-in-pod ?r ?a))
        (carrying-empty-pod ?r)      ; Pod remains with robot
        (artifact-at ?a ?l)
    )
  )

  ;; Note: we have not modelled yet the action to drop the pod with inside an artifact;
  ;; Transition carrying-in-pod ?a -> empty hand
  ;; To inglude this logi we should also consider that the artifact could stay inside 
  ;; or outside a pod and also more ...

  ;; B. Dropping to Cryo-Chamber (Temperature Effect)
  ;; 3. Cryo Drop (From Bare Hands)
  ;; Transition: carrying ?a -> hands-empty
  (:action drop-in-cryo
    :parameters (?a - artifact ?l - location)
    :precondition (and 
        (robot-at ?l) 
        (carrying ?a)
        (is-chill-room ?l)
    )
    :effect (and 
        (not (carrying ?a)) 
        (hands-empty) 
        (artifact-at ?a ?l)
        (not (warm ?a)) (cold ?a) ; Instant Temp Logic
    )
  )

  ;; 4. Cryo Unload (From Pod)
  ;; Transition: carrying-in-pod ?a -> carrying-empty-pod
  (:action drop-in-cryo-from-pod
    :parameters (?a - artifact ?l - location)
    :precondition (and 
        (robot-at ?l)
        (carrying-in-pod ?a)
        (is-chill-room ?l)
    )
    :effect (and 
        (not (carrying-in-pod ?a))
        (carrying-empty-pod)     ; Pod remains with robot
        (artifact-at ?a ?l)
        (not (warm ?a)) (cold ?a) ; Instant Temp Logic
    )
  )
)