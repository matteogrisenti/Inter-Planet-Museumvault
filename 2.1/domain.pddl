(define (domain single-robot)
  (:requirements :strips :typing)

  ;; Types derived from scenario entities 
  (:types  
    robot
    location          ; Locations: the different rooms in the enviroment
    artifact          ; Artifacts: the different artifact in the enviroment
    artifact-type     ; Artifacts Tipologies: the different tipologies of artifacts in the enviroment
  )

  (:predicates
    ;; Robot Features - they all refears to the unique robot in the enviroment
    (robot-at ?l - location)                    ; Robot's current location
    (hand-empty)                                ; Robot's hand status ( true if empty)  
    (carrying ?a - artifact)                    ; Robot is carrying the artifact ?a
    (carrying-empty-pods)                                 ; Robot is carrying an empty anti vibration pod
    (carrying-in-pod ?a - artifact)                                 ; Robot has a secure fragile artifact inside an anti vibration pod
    (sealing-mode)                              ; Robot mode (sealing or normal)

    ;; Locations Features
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
    (contain-free-pod ?l - location)

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
    (fragile  ?a - artifact)                     ; Artifact need to be traveled inside an anti-vibration-pod
    (no-fragile  ?a - artifact )                   
    (cold  ?a - artifact )                       ; Artifact are already cold
    (warm  ?a - artifact )

    ; Note 3: need-chill and need-vibration-pods can be removed becouse coincid with the location feature of the 
    ; artifact becouse both feature depend on where the artifact is situated

  )

  
  ;; ========================
  ;; ACTIONS 
  ;; ========================

  ;; MOVEMENT
  ;; A: Moving Empty (No artifact constraints)
  ;; 1. Move Empty to a Safe Room (No sealing needed)
  (:action move-empty-safe
    :parameters (?from ?to - location)
    :precondition (and 
        (robot-at ?from)
        (connected ?from ?to)
        (hand-empty)
        (is-pressurized ?to)       ;; Target is pressurized 
        (is-safe ?to)              ;; Target is safe
    )
    :effect (and (not (robot-at ?from)) (robot-at ?to))
  )
  ;; 2. Move Empty to Tunnel (Requires Sealing)
  (:action move-empty-tunnel
    :parameters (?from ?to - location)
    :precondition (and 
        (robot-at ?from)
        (connected ?from ?to)
        (hand-empty)
        (is-unpressurized ?to)     ;; Target is unpressurized
        (sealing-mode)             ;; Constraint: Must be sealed
        (is-safe ?to)              ;; Target is safe
    )
    :effect (and (not (robot-at ?from)) (robot-at ?to))
  )

  ;; B. Moving Standard Artifacts (No pod needed)
  ;; 3. Move Standard Item to Safe Room
  (:action move-carrying-safe
    :parameters (?from ?to - location ?a - artifact)
    :precondition (and 
        (robot-at ?from)
        (connected ?from ?to)
        (carrying ?a)
        (no-fragile ?a)            ;; Ensure it's NOT fragile
        (is-pressurized ?to)       ;; Target is pressurized 
        (is-safe ?to)              ;; Target is safe  
    )
    :effect (and 
        (not (robot-at ?from)) (robot-at ?to)
    )
  )
  ;; 4. Move Standard Item to Tunnel (Requires Sealing)
  (:action move-carrying-tunnel
    :parameters (?from ?to - location ?a - artifact)
    :precondition (and 
        (robot-at ?from)
        (connected ?from ?to)
        (carrying ?a)
        (no-fragile ?a)            ;; Ensure it's NOT fragile
        (is-unpressurized ?to)     ;; Target is unpressurized
        (sealing-mode)             ;; Constraint: Must be sealed
        (is-safe ?to)              ;; Target is safe
    )
    :effect (and 
        (not (robot-at ?from)) (robot-at ?to)
    )
  )

  ;; C. Moving Fragile Artifacts (Pod required)
  ;; 5. Move Fragile Item to Safe Room (Requires Pod)
  (:action move-fragile-safe
    :parameters (?from ?to - location ?a - artifact)
    :precondition (and 
        (robot-at ?from)
        (connected ?from ?to)
        ;; Constraint: The fragile asrtifact must put inside an 
        ;; anti-vibration-pod to be secured during trasporation
        (carrying-in-pod ?a)              ;; <=
        (is-pressurized ?to)              ;; Target is pressurized 
        (is-safe ?to)                     ;; Target is safe 
    )
    :effect (and 
        (not (robot-at ?from)) (robot-at ?to)
    )
  )
  ;; 6. Move Fragile Item to Tunnel (Requires Pod AND Sealing)
  (:action move-fragile-tunnel
    :parameters (?from ?to - location ?a - artifact)
    :precondition (and 
        (robot-at ?from)
        (connected ?from ?to)
        ;; Constraint: The fragile asrtifact must put inside an 
        ;; anti-vibration-pod to be secured during trasporation
        (carrying-in-pod ?a)              ;; <=
        (is-unpressurized ?to)            ;; Target is unpressurized
        (sealing-mode)                    ;; Constraint: Must be sealed
        (is-safe ?to)                     ;; Target is safe
    )
    :effect (and 
        (not (robot-at ?from)) (robot-at ?to)
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
  ;; Transition: hand-empty -> carrying-empty-pods
  (:action pick-up-empty-pod
    :parameters (?l - location)
    :precondition (and 
        (robot-at ?l)
        (contain-free-pod ?l)       ; Must be in room the contain free pod
        (hand-empty)                ; Must have free hands
    )
    :effect (and 
        (not (hand-empty))
        (carrying-empty-pods)
    )
  )

  ;; 2. Drop Pod
  ;; Transition: carrying-empty-pods -> hand-empty
  ;; Useful if we need to free hands to move a non-fragile item normally
  (:action drop-empty-pod
    :parameters ()
    :precondition (and 
        (carrying-empty-pods)
    )
    :effect (and 
        (not (carrying-empty-pods))
        (hand-empty)
    )
  )

 
  ;; PICK UP ACTIONS
  ;; 1. STANDARD PICK UP
  ;; Transition: hand-empty -> carrying ?a
  ;; Used for sturdy items. If you pick up a fragile item this way, 
  ;; you won't be able to move it (due to move constraints).
  (:action pick-up
    :parameters (?a - artifact ?l - location)
    :precondition (and 
        (robot-at ?l)            ; Robot and Artifact in the same 
        (artifact-at ?a ?l)      ; location
        (hand-empty)             ; Robot free
    )
    :effect (and 
        (not (hand-empty)) 
        (carrying ?a)            
        (not (artifact-at ?a ?l))
    )
  )

  ;; 4. SECURE PICK UP (Load into Pod)
  ;; Transition: carrying-empty-pods -> carrying-in-pod ?a
  ;; This action "secures" the artifact immediately.
  (:action secure-pick-up
    :parameters (?a - artifact ?l - location)
    :precondition (and 
        (robot-at ?l)            ; Robot and Artifact in the same 
        (artifact-at ?a ?l)      ; location
        (carrying-empty-pods)    ; Robot carrying already an empty pod
    )
    :effect (and 
        (not (carrying-empty-pods))
        (carrying-in-pod ?a)     
        (not (artifact-at ?a ?l))
    )
  )


  ;; DROP DOWN ACTIONS
  ;; A. Dropping to Standard Rooms
  ;; 1. Standard Artifact Drop 
  ;; Transition: carrying ?a -> hand-empty
  (:action drop-standard
    :parameters (?a - artifact ?l - location)
    :precondition (and 
        (robot-at ?l) 
        (carrying ?a)
        (is-standard-room ?l)
    )
    :effect (and 
        (not (carrying ?a)) 
        (hand-empty) 
        (artifact-at ?a ?l)
    )
  )

  ;; 2. Unload From Pod (Standard Room)
  ;; Transition: carrying-in-pod ?a -> carrying-empty-pods
  ;; Logic: We place the artifact down, but keep the empty pod in hand.
  (:action drop-standard-from-pod
    :parameters (?a - artifact ?l - location)
    :precondition (and 
        (robot-at ?l)
        (carrying-in-pod ?a)
        (is-standard-room ?l)
    )
    :effect (and 
        (not (carrying-in-pod ?a))
        (carrying-empty-pods)      ; Pod remains with robot
        (artifact-at ?a ?l)
    )
  )

  ;; Note: we have not modelled yet the action to drop the pod with inside an artifact;
  ;; Transition carrying-in-pod ?a -> empty hand
  ;; To inglude this logi we should also consider that the artifact could stay inside 
  ;; or outside a pod and also more ...

  ;; B. Dropping to Cryo-Chamber (Temperature Effect)
  ;; 3. Cryo Drop (From Bare Hands)
  ;; Transition: carrying ?a -> hand-empty
  (:action drop-in-cryo
    :parameters (?a - artifact ?l - location)
    :precondition (and 
        (robot-at ?l) 
        (carrying ?a)
        (is-chill-room ?l)
    )
    :effect (and 
        (not (carrying ?a)) 
        (hand-empty) 
        (artifact-at ?a ?l)
        (not (warm ?a)) (cold ?a) ; Instant Temp Logic
    )
  )

  ;; 4. Cryo Unload (From Pod)
  ;; Transition: carrying-in-pod ?a -> carrying-empty-pods
  (:action drop-in-cryo-from-pod
    :parameters (?a - artifact ?l - location)
    :precondition (and 
        (robot-at ?l)
        (carrying-in-pod ?a)
        (is-chill-room ?l)
    )
    :effect (and 
        (not (carrying-in-pod ?a))
        (carrying-empty-pods)     ; Pod remains with robot
        (artifact-at ?a ?l)
        (not (warm ?a)) (cold ?a) ; Instant Temp Logic
    )
  )
)