(define (domain single-robot)
  (:requirements :strips :typing)

  ;; Types derived from scenario entities 
  ; The robot is not a type, becouse in this scenario we have only one robot
  ; so it can be modelled as one object in the problem file.
  (:types  
    location          ; Locations: the different rooms in the enviroment
    artifact          ; Artifacts: the different artifact in the enviroment
    artifact-type     ; Artifacts Tipologies: the different tipologies of artifacts in the enviroment
  )

  (:predicates
    ;; Robot Features - they all refears to the unique robot in the enviroment
    (robot-at ?l - location)                    ; Robot's current location
    (hand-empty)                                ; Robot's hand status ( true if empty)  
    (carrying ?a - artifact)                    ; Robot is carrying the artifact ?a
    (carrying-anti-vibration-pods)              ; Robot is carrying an empty anti vibration pod
    (sealing-mode)                              ; Robot mode (sealing or normal)

    ;; Locations Features
    (connected ?l1 ?l2 - location)            ; Location are linked or not 
    (is-unpressurized ?l - location)          ; True for tuneel
    (is-unsafe ?l -location)                  ; True if not safe to enter ( Model the mars_quake const for Hall B)

    ; Note 1 : can be a nice think to test the same problem but with a positive definition of the last two is- feature:
    ; - (is-pressurized ?l - location)
    ; - (is-safe ?l -location)
    ; In theory this will lead to a bigger number of atoms that can affect the velocity of the resolver

    ; Note 2: To increase complexity we can add a capacity for the cryo-chamber room

    ;; Artifacts Features
    (is-type  ?a - artifact ?t - artifact-type)     ; Artifact a belong to the Artifact Tipology t 
    (artifact-at  ?a - artifact ?l - location)      ; Artifact a at location l  
    (need-chill  ?a - artifact)                     ; Artifact need to be cool down
    (need-anti-vibration-pods  ?a - artifact)       ; Artifact need to be traveled inside an anti-vibration-pod
    (cold  ?a - artifact)                           ; Artifact are already cold

    ; Note 3: need-chill and need-vibration-pods can be removed becouse coincid with the location feature of the 
    ; artifact becouse both feature depend on where the artifact is situated

  )

  ; TODO (:action )


)