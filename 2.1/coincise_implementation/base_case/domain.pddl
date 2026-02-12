(define (domain single-robot-optimized)
    (:requirements :strips :typing :non-deterministic :negative-preconditions :conditional-effects :disjunctive-preconditions)

    (:types
        robot pod location artifact artifact-type - object
    )

    (:predicates
        (robot-at ?r - robot ?l - location)
        (carrying ?r - robot ?a - artifact)      
        (carrying-pod ?r - robot ?p - pod)       
        (sealed ?r - robot)                      
        
        ;; Status predicates to replace existential quantifiers
        (hands-full ?r - robot)                  
        (pod-full ?p - pod)                      

        ;; Room States
        (is-seismic ?l - location)
        (is-safe ?l - location)
        (is-pressurized ?l - location)
        (is-standard-room ?l - location)
        (is-chill-room ?l - location)
        (connected ?l1 ?l2 - location)

        ;; Artifact/Pod Properties
        (artifact-at ?a - artifact ?l - location)
        (pod-at ?p - pod ?l - location)
        (pod-contains ?p - pod ?a - artifact)
        (fragile ?a - artifact)
        (cold ?a - artifact)
    )

    ;; =========================================================================
    ;; MOVEMENT LOGIC
    ;; =========================================================================

    (:action activate-seal
        :parameters (?r - robot)
        :precondition (not (sealed ?r))
        :effect (sealed ?r)
    )

    (:action move
        :parameters (?r - robot ?from ?to - location)
        :precondition (and 
            (robot-at ?r ?from)
            (connected ?from ?to)
            (not (is-seismic ?to))
            (or (is-pressurized ?to) (sealed ?r))
        )
        :effect (and 
            (not (robot-at ?r ?from))
            (robot-at ?r ?to)
            (when (is-pressurized ?to) (not (sealed ?r)))
        )
    )

    (:action try-to-enter-seismic-room
        :parameters (?r - robot ?from ?to - location)
        :precondition (and (robot-at ?r ?from) (connected ?from ?to) (is-seismic ?to))
        :effect (oneof 
            (and (is-safe ?to) (not (robot-at ?r ?from)) (robot-at ?r ?to) (when (is-pressurized ?to) (not (sealed ?r))))
            (not (is-safe ?to))
        )
    )

    ;; =========================================================================
    ;; LOGISTICS: PODS & ARTIFACTS
    ;; =========================================================================

    (:action pick-up-pod
        :parameters (?r - robot ?p - pod ?l - location)
        :precondition (and 
            (robot-at ?r ?l) 
            (pod-at ?p ?l) 
            (not (hands-full ?r))
        )
        :effect (and (not (pod-at ?p ?l)) (carrying-pod ?r ?p) (hands-full ?r))
    )

    (:action drop-pod
        :parameters (?r - robot ?p - pod ?l - location)
        :precondition (and (robot-at ?r ?l) (carrying-pod ?r ?p))
        :effect (and (not (carrying-pod ?r ?p)) (pod-at ?p ?l) (not (hands-full ?r)))
    )

    (:action put-in-pod
        :parameters (?r - robot ?a - artifact ?p - pod ?l - location)
        :precondition (and 
            (robot-at ?r ?l) 
            (artifact-at ?a ?l) 
            (carrying-pod ?r ?p) 
            (not (pod-full ?p))
        )
        :effect (and (not (artifact-at ?a ?l)) (pod-contains ?p ?a) (pod-full ?p))
    )

    (:action release-artifact-from-pod
        :parameters (?r - robot ?a - artifact ?p - pod ?l - location)
        :precondition (and (robot-at ?r ?l) (carrying-pod ?r ?p) (pod-contains ?p ?a))
        :effect (and 
            (not (pod-contains ?p ?a))
            (not (pod-full ?p))
            (artifact-at ?a ?l)
            (when (is-chill-room ?l) (cold ?a))
        )
    )

    (:action pick-up-artifact-hand
        :parameters (?r - robot ?a - artifact ?l - location)
        :precondition (and 
            (robot-at ?r ?l) 
            (artifact-at ?a ?l) 
            (not (fragile ?a))
            (not (hands-full ?r))
        )
        :effect (and (not (artifact-at ?a ?l)) (carrying ?r ?a) (hands-full ?r))
    )

    (:action release-artifact-hand
        :parameters (?r - robot ?a - artifact ?l - location)
        :precondition (and (robot-at ?r ?l) (carrying ?r ?a))
        :effect (and 
            (not (carrying ?r ?a)) 
            (not (hands-full ?r))
            (artifact-at ?a ?l)
            (when (is-chill-room ?l) (cold ?a))
        )
    )
)