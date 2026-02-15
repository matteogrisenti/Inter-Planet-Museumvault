(define (domain temporal-multi-robot)
  (:requirements :strips :typing :timed-initial-literals :durative-actions)

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
    (sealing-mode-on ?r - robot) 
    (sealing-mode-off ?r - robot)                      ; Required for unpressurized zones
    
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
    (is-safe ?l - location)                         ; !Room safety status (this is used to determine if the robot can enter or not, 
                                                        ; !but it is not deterministic, so we have to check it every time we try to enter a room)
    (is-chill-room ?l - location)                   ; Rooms that cool artifacts (Cryo-Chamber)

    ;; --- ARTIFACT & POD STATUS ---
    (artifact-at ?a - artifact ?l - location)
    (is-type ?a - artifact ?t - artifact-type)
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
    :duration (= ?duration 10)
    :condition (and 
        (at start (robot-at ?r ?from))   
        (at start (connected ?from ?to))        ; use at start (state is fixed)
        (at start (is-pressurized ?to))         ; use at start (state is fixed)
        (over all (is-safe ?to))                ; must be safe for the entire action (no quake)
        (at start (can-access ?r ?to))
    )
    :effect (and 
        (at start (not (robot-at ?r ?from))) ; first remove robot from original location
        (at end (robot-at ?r ?to))          ; then place it in the new location when reached
        (at end (not (sealing-mode-on ?r)))   ; when enters room the sealing mode automatically turns off
        (at end (sealing-mode-off ?r))      ; automatically deactivate sealing mode (it can only come from unpressurized tunnel)
        )
  )

  ;; 2. Move Empty to Tunnel (Requires Sealing)
  (:durative-action move-to-unpressurized-room
    :parameters (?r - robot ?from ?to - location)
    :duration (= ?duration 10)
    :condition (and 
        (at start (robot-at ?r ?from))   
        (at start (sealing-mode-on ?r))           ; Sealing mode must be on at the start to enter unpressurized areas
        (at start (connected ?from ?to))          ; use at start (state is fixed)
        (at start (is-unpressurized ?to))         ; use at start (state is fixed)
        (at start (can-access ?r ?to))            ; use at start (state is fixed)
        ; (over all (is-safe ?to))                ; No need since the tunnel is always safe 
    )
    :effect (and 
        (at start (not (robot-at ?r ?from))) 
        (at end (robot-at ?r ?to))
    )
  )

  ;; SEALING MECHANISM
  ;; Robot activates sealing mode. Can be done anywhere.
  (:durative-action activate-seal
    :parameters (?r - robot)
    :duration (= ?duration 2)
    :condition (at start (sealing-mode-off ?r))
    :effect (and 
        (at start (not (sealing-mode-off ?r)))
        (at end (sealing-mode-on ?r))
        )
  )

  ;; REMOVED DEACRTIVATE-SEAL SINCE IT IS AUTOMATICALLY DEACTIVATED WHEN ENTERING A PRESSURIZED ROOM (THIS IS HANDLED IN THE EFFECT OF THE MOVEMENT ACTIONS, NO NEED FOR A SEPARATE ACTION)
  ;; due to the topology of the environment, we can afford this.

  ;; POD MANAGEMENT: EQUIPPING & UNEQUIPPING
  ;; 1. Pick Up Pod
  ;; Transition: hands-empty -> carrying-empty-pod
  (:durative-action pick-up-empty-pod
    :parameters (?r - robot ?l - location ?p - pod)
    :duration (= ?duration 2)
    :condition (and 
        (at start (hands-empty ?r))                ; Must have free hands
        (at start (contains-empty-pod ?l ?p))      ; Must be in room the contain free pod
        (over all (pod-empty ?p))                  ; Pod must be empty
        (over all (robot-at ?r ?l))                ; Robot must be at the location for the entire duration
    )
    :effect (and 
        (at start (not (contains-empty-pod ?l ?p))) ; Pod is no longer available in the room (no other robot can take it while it is picked up)
        (at start (not (hands-empty ?r)))           ; hands are not empty anomer (it can't take another objects meanwhile)
        (at end (carrying-empty-pod ?r ?p))         ; Robot is now carrying the empty pod
    )
  )

  (:durative-action pick-up-full-pod
      :parameters (?r - robot ?l - location ?p - pod ?a - artifact ?at - artifact-type)
      :duration (= ?duration 4)
      :condition (and 
          (at start (contains-full-pod ?l ?p))      ; Pod must be in the room at the start
          (at start (hands-empty ?r))               ; Must have free hands at the start
          (at start (can-pickup ?r ?at))            ; use at start (state is fixed)
          (at start (is-type  ?a ?at))              ; use at start (state is fixed)
          (over all (robot-at ?r ?l))               ; Robot must be at the location for the entire duration;
          (over all (pod-contains ?p ?a))           ; Pod must contain the artifact for the entire duration
      )
      :effect (and 
          (at start (not (hands-empty ?r)))              ; Robot's hands are no longer empty at the start (cannot take other objects while it is picking up something)
          (at start (not (contains-full-pod ?l ?p)))     ; Pod is no longer available in the room (no other robot can take it while it is picked up)
          (at end (carrying-full-pod ?r ?p))             ; Robot is now carrying the full pod at the end
      )
  )
  

  ;; 2. Drop Pod
  (:durative-action drop-empty-pod
    :parameters (?r - robot ?p - pod ?l - location)
    :duration (= ?duration 2)
    :condition (and 
        (at start (carrying-empty-pod ?r ?p))      ; Robot must be carrying the empty pod at the start
        (over all (robot-at ?r ?l))                ; Robot must be at the location for the entire duration
        (over all (pod-empty ?p))                  ; Pod must not be empty at the start (it is being dropped, so it will become available in the room)
    )
    :effect (and 
        (at start (not (carrying-empty-pod ?r ?p)))
        (at end (hands-empty ?r))
        (at end (contains-empty-pod ?l ?p))       ; Pod is now available in the room
    )
  )

  ;; 3. Drop Full Pod (with artifact inside)
 (:durative-action drop-full-pod
     :parameters (?r - robot ?p - pod ?l - location)
     :duration (= ?duration 4)
     :condition (and 
         (at start (carrying-full-pod ?r ?p))           ; Robot must be carrying the full pod at the start
         (over all (robot-at ?r ?l))                    ; Robot must be at the location for the entire duration
     )
     :effect (and 
         (at start (not (carrying-full-pod ?r ?p)))     ; Robot is no longer carrying the full pod at the start
         (at end (hands-empty ?r))                      ; Robot's hands are empty at the end
         (at end (contains-full-pod ?l ?p))             ; Full pod is now available in the room
     )
 )
 
  
  ;; PICK UP ARTIFACT ACTIONS

  ;; 1. STANDARD PICK UP
  (:durative-action pick-up-artifact
    :parameters (?a - artifact ?at - artifact-type ?l - location ?r - robot)
    :duration (= ?duration 1)
    :condition (and 
        (at start (artifact-at ?a ?l))         ; location
        (at start (hands-empty ?r))             ; Robot free
        (at start (no-fragile  ?a))
        (at start (can-pickup ?r ?at))
        (at start (is-type  ?a ?at))
        (over all (robot-at ?r ?l))            ; Robot and Artifact in the same 
    )
    :effect (and 
        (at start (not (artifact-at ?a ?l)))
        (at start (not (hands-empty ?r))) 
        (at end (carrying ?r ?a))            
    )
  )

  ;; 4. PUT IN POD (Load into Pod)
  ;; Transition: carrying-empty-pod -> carrying-in-pod ?a
  ;; This action "secures" the artifact immediately.
  (:durative-action put-in-pod
    :parameters (?a - artifact ?at - artifact-type ?l - location ?r - robot ?p - pod)
    :duration (= ?duration 4)
    :condition (and 
        (at start (artifact-at ?a ?l))         
        (at start (carrying-empty-pod ?r ?p))  ; Robot carrying already an empty pod
        (at start (pod-empty ?p))              ; Pod is empty
        (at start (can-pickup ?r ?at))         ; use at start since these are fixed properties
        (at start (is-type  ?a ?at))           ; use at start since these are fixed properties
        (over all (robot-at ?r ?l))            ; Robot and Artifact in the same location for the entire duration
    )
    :effect (and 
        (at start (not (artifact-at ?a ?l)))
        (at start (not (carrying-empty-pod ?r ?p)))
        (at start (not (pod-empty ?p)))
        (at end (carrying-full-pod ?r ?p))  
        (at end (pod-contains ?p ?a))
    )
  )


  ;; DROP DOWN ACTIONS
  ;; A. Dropping to Standard Rooms
  ;; 1. Standard Artifact Drop 
  (:durative-action release-artifact
    :parameters (?r - robot ?a - artifact ?l - location)
    :duration (= ?duration 1)
    :condition (and 
        (at start (carrying ?r ?a))
        (over all (robot-at ?r ?l)) 
    )
    :effect (and 
        (at start (not (carrying ?r ?a))) 
        (at end (hands-empty ?r)) 
        (at end (artifact-at ?a ?l))
    )
  )

  ;; 2. Unload From Pod (Standard Room)
  ;; Transition: carrying-full-pod -> carrying-empty-pod
  (:durative-action release-artifact-from-pod
    :parameters (?r - robot ?a - artifact ?l - location ?p - pod)
    :duration (= ?duration 4)
    :condition (and
        (at start (carrying-full-pod ?r ?p))  
        (at start (pod-contains ?p ?a))       
        (over all (robot-at ?r ?l))
    )
    :effect (and
        (at start (not (carrying-full-pod ?r ?p)))
        (at start (not (pod-contains ?p ?a))) ; Pod is now empty
        (at end (artifact-at ?a ?l))
        (at end (pod-empty ?p))
        (at end (carrying-empty-pod ?r ?p))      ; Robot still holds the empty pod
    )
  )

  ;; ! B. Cooling artifact in the Cryo-Chamber (Temperature Effect)
  ; This works both if robots carries just and objects but also if the object is in the pod (instead of realeasing it and then cooling it -> in the end it should have the same effect/time cost)
;   (:durative-action cool-artifact-in-cryo
;     :parameters (?r - robot ?a - artifact ?l - location)
;     :duration (= ?duration 2)
;     :condition (and 
;         (at start (warm ?a))                      ; Artifact must be warm at the start
;         (over all (artifact-at ?a ?l))              ; Artifact must be inside the Cryo-Chamber for the whole time of the action
;         (at start (robot-at ?r ?l))                 ; Robot must be at the location for the entire duration
;         (over all (is-chill-room ?l))               ; Location must be a chill room for the entire duration
;     )
;     :effect (and 
;         (at start (not (warm ?a)))                  ; Artifact is no longer warm at the start
;         (at end (cold ?a))                          ; Artifact is cold at the end
;     )
;   )

  (:durative-action cool-artifact-while-carrying
    :parameters (?r - robot ?a - artifact ?l - location)
    :duration (= ?duration 12)
    :condition (and 
        (at start (warm ?a))                       ; Artifact must be warm at the start
        (at start  (is-chill-room ?l))             ; use at start since these is a fixed properties
        (over all (robot-at ?r ?l))                ; Robot must be at the location for the entire duration
        (over all (carrying ?r ?a))              
    )
    :effect (and 
        (at start (not (warm ?a)))                  ; Artifact is no longer warm at the start
        (at end (cold ?a))                          ; Artifact is cold at the end
    )
  )

  (:durative-action cool-artifact-while-carrying-in-pod
     :parameters (?r - robot ?a - artifact ?l - location ?p - pod)
     :duration (= ?duration 14)
     :condition (and 
         (at start (warm ?a))                       ; Artifact must be warm at the start
         (at start (is-chill-room ?l))              ; use at start since these is fixed properties
         (over all (carrying-full-pod ?r ?p))       ; Robot must be carrying the full pod (this implies pod-contains ?p ?a) for the entire duration
         (over all (robot-at ?r ?l))                 ; Robot must be at the location for the entire duration
     )
     :effect (and 
         (at start (not (warm ?a)))                  ; Artifact is no longer warm at the start
         (at end (cold ?a))                          ; Artifact is cold at the end
     )
  )


  ;; * Second object slot actions (Technician Robot Only) - Pickup and Release
  
  ; Pick-up action
  (:durative-action pick-up-second-object
      :parameters (
            ?r - robot ?a - artifact ?at - artifact-type ?l - location
      )
      :duration (= ?duration 2)
      :condition (and 
            (at start (artifact-at ?a ?l))
            (at start (second-slot-empty ?r))           ; Second slot must be empty
            (at start (can-carry-two ?r))               ; use at start since these is fixed properties
            (at start (can-pickup ?r ?at))
            (at start (is-type  ?a ?at))
            (at start (no-fragile  ?a))
            (over all (robot-at ?r ?l))
        )
      :effect (and 
          (at start (not (second-slot-empty ?r)))
          (at start (not (artifact-at ?a ?l)))
          (at end (carrying-second-object ?r ?a))
      )
  )

  (:durative-action release-second-object
      :parameters (?r - robot ?a - artifact ?l - location)
      :duration (= ?duration 2)
      :condition (and 
            (at start (carrying-second-object ?r ?a))
            (over all (robot-at ?r ?l))
        )
      :effect (and 
          (at start (not (carrying-second-object ?r ?a)))
          (at end (second-slot-empty ?r))
          (at end (artifact-at ?a ?l))
      )
  )
)