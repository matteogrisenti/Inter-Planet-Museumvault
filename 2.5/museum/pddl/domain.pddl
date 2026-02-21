(define (domain museum)
  (:requirements :strips :typing :durative-actions)

  (:types
    robot
    pod
    location
    artifact
    artifact_type
  )

  (:predicates
    (robot_at ?r - robot ?l - location)
    (hands_empty_slot_1 ?r - robot)
    (carrying_slot_1 ?r - robot ?a - artifact)
    (carrying_pod_slot_1 ?r - robot ?p - pod)
    (hands_empty_slot_2 ?r - robot)
    (carrying_slot_2 ?r - robot ?a - artifact)
    (sealing_mode_on ?r - robot)
    (sealing_mode_off ?r - robot)
    (can_access ?r - robot ?l - location)
    (can_pickup ?r - robot ?at - artifact_type)
    (can_carry_two ?r - robot)
    (connected ?l1 ?l2 - location)
    (is_pressurized ?l - location)
    (is_unpressurized ?l - location)
    (is_safe ?l - location)
    (is_chill_room ?l - location)
    (artifact_at ?a - artifact ?l - location)
    (is_type ?a - artifact ?t - artifact_type)
    (no_fragile ?a - artifact)
    (warm ?a - artifact)
    (cold ?a - artifact)
    (pod_at ?p - pod ?l - location)
    (pod_empty ?p - pod)
    (pod_contains ?p - pod ?a - artifact)
    (is_seismic ?l - location)
    (can_fly ?r - robot)
  )

  (:durative-action fly_into_seismic_room   ; for drone case
        :parameters (?r - robot ?to ?from - location)
        :duration (= ?duration 10)
        :condition (and 
            (at start (sealing_mode_on ?r))
            (at start (robot_at ?r ?from))
            (at start (connected ?from ?to))
            (at start (is_seismic ?to))
            (at start (can_access ?r ?to))
            (at start (can_fly ?r))
        )
        :effect (and 
            (at start (not (robot_at ?r ?from)))
            (at end (not (sealing_mode_on ?r)))
            (at end (robot_at ?r ?to))
            (at end (sealing_mode_off ?r))
        )
    )
  

  (:durative-action move_to_pressurized_room
    :parameters (?r - robot ?from ?to - location)
    :duration (= ?duration 10)
    :condition (and 
        (at start (robot_at ?r ?from))
        (at start (connected ?from ?to))
        (at start (is_pressurized ?to))
        (over all (is_safe ?to))
        (at start (can_access ?r ?to))
    )
    :effect (and 
        (at start (not (robot_at ?r ?from)))
        (at end (robot_at ?r ?to))
        (at end (not (sealing_mode_on ?r)))
        (at end (sealing_mode_off ?r))
    )
  )

  (:durative-action move_to_unpressurized_room
    :parameters (?r - robot ?from ?to - location)
    :duration (= ?duration 10)
    :condition (and 
        (at start (robot_at ?r ?from))
        (at start (sealing_mode_on ?r))
        (at start (connected ?from ?to))
        (at start (is_unpressurized ?to))
        (at start (can_access ?r ?to))
    )
    :effect (and 
        (at start (not (robot_at ?r ?from)))
        (at end (robot_at ?r ?to))
    )
  )

  (:durative-action activate_seal
    :parameters (?r - robot)
    :duration (= ?duration 2)
    :condition (at start (sealing_mode_off ?r))
    :effect (and 
        (at start (not (sealing_mode_off ?r)))
        (at end (sealing_mode_on ?r))
    )
  )

  (:durative-action pick_up_empty_pod_slot_1
    :parameters (?r - robot ?l - location ?p - pod)
    :duration (= ?duration 2)
    :condition (and 
        (at start (hands_empty_slot_1 ?r))
        (at start (pod_at ?p ?l))
        (at start (pod_empty ?p))           ; <--- it was over all
        (at start (robot_at ?r ?l))         ; <--- it was over all
    )
    :effect (and 
        (at start (not (pod_at ?p ?l)))
        (at start (not (hands_empty_slot_1 ?r)))
        (at end (carrying_pod_slot_1 ?r ?p))
    )
  )

  (:durative-action pick_up_full_pod_slot_1
      :parameters (?r - robot ?l - location ?p - pod ?a - artifact ?at - artifact_type)
      :duration (= ?duration 4)
      :condition (and 
          (at start (pod_at ?p ?l))
          (at start (hands_empty_slot_1 ?r))
          (at start (can_pickup ?r ?at))
          (at start (is_type  ?a ?at))
          (over all (robot_at ?r ?l))
          (over all (pod_contains ?p ?a))
      )
      :effect (and 
          (at start (not (hands_empty_slot_1 ?r)))
          (at start (not (pod_at ?p ?l)))
          (at end (carrying_pod_slot_1 ?r ?p))
      )
  )

 (:durative-action drop_pod_slot_1
     :parameters (?r - robot ?p - pod ?l - location)
     :duration (= ?duration 4)
     :condition (and 
         (at start (carrying_pod_slot_1 ?r ?p))
         (over all (robot_at ?r ?l))
     )
     :effect (and 
         (at start (not (carrying_pod_slot_1 ?r ?p)))
         (at end (hands_empty_slot_1 ?r))
         (at end (pod_at ?p ?l))
     )
 )

  (:durative-action pick_up_slot_1
    :parameters (?a - artifact ?at - artifact_type ?l - location ?r - robot)
    :duration (= ?duration 1)
    :condition (and 
        (at start (artifact_at ?a ?l))
        (at start (hands_empty_slot_1 ?r))
        (at start (no_fragile  ?a))
        (at start (can_pickup ?r ?at))
        (at start (is_type  ?a ?at))
        (over all (robot_at ?r ?l))
    )
    :effect (and 
        (at start (not (artifact_at ?a ?l)))
        (at start (not (hands_empty_slot_1 ?r)))
        (at end (carrying_slot_1 ?r ?a))
    )
  )

  (:durative-action put_in_pod_slot_1
    :parameters (?a - artifact ?at - artifact_type ?l - location ?r - robot ?p - pod)
    :duration (= ?duration 4)
    :condition (and 
        (at start (artifact_at ?a ?l))
        (at start (carrying_pod_slot_1 ?r ?p))
        (at start (pod_empty ?p))
        (at start (can_pickup ?r ?at))
        (at start (is_type  ?a ?at))
        (over all (robot_at ?r ?l))
    )
    :effect (and 
        (at start (not (artifact_at ?a ?l)))
        (at start (not (pod_empty ?p)))
        (at end (carrying_pod_slot_1 ?r ?p))
        (at end (pod_contains ?p ?a))
    )
  )

  (:durative-action release_artifact_slot_1
    :parameters (?r - robot ?a - artifact ?l - location)
    :duration (= ?duration 1)
    :condition (and 
        (at start (carrying_slot_1 ?r ?a))
        (over all (robot_at ?r ?l))
    )
    :effect (and 
        (at start (not (carrying_slot_1 ?r ?a)))
        (at end (hands_empty_slot_1 ?r))
        (at end (artifact_at ?a ?l))
    )
  )

  (:durative-action release_artifact_from_pod_slot_1
    :parameters (?r - robot ?a - artifact ?l - location ?p - pod)
    :duration (= ?duration 4)
    :condition (and
        (at start (carrying_pod_slot_1 ?r ?p))
        (at start (pod_contains ?p ?a))
        (over all (robot_at ?r ?l))
    )
    :effect (and
        (at start (not (pod_contains ?p ?a)))
        (at end (artifact_at ?a ?l))
        (at end (pod_empty ?p))
    )
  )

  (:durative-action cool_artifact_while_carrying_slot_1
    :parameters (?r - robot ?a - artifact ?l - location)
    :duration (= ?duration 12)
    :condition (and 
        (at start (warm ?a))
        (at start (is_chill_room ?l))
        (over all (robot_at ?r ?l))
        (over all (carrying_slot_1 ?r ?a))
    )
    :effect (and 
        (at start (not (warm ?a)))
        (at end (cold ?a))
    )
  )

  (:durative-action cool_artifact_while_carrying_in_pod_slot_1
     :parameters (?r - robot ?a - artifact ?l - location ?p - pod)
     :duration (= ?duration 14)
     :condition (and 
         (at start (warm ?a))
         (at start (is_chill_room ?l))                  
         (over all (carrying_pod_slot_1 ?r ?p))         ; <---
         (over all (robot_at ?r ?l))                    ; <---
     )
     :effect (and 
         (at start (not (warm ?a)))
         (at end (cold ?a))
     )
  )

  (:durative-action pick_up_slot_2
      :parameters (?r - robot ?a - artifact ?at - artifact_type ?l - location)
      :duration (= ?duration 2)
      :condition (and 
            (at start (artifact_at ?a ?l))
            (at start (hands_empty_slot_2 ?r))
            (at start (can_carry_two ?r))
            (at start (can_pickup ?r ?at))
            (at start (is_type  ?a ?at))
            (at start (no_fragile  ?a))
            (over all (robot_at ?r ?l))
        )
      :effect (and 
          (at start (not (hands_empty_slot_2 ?r)))
          (at start (not (artifact_at ?a ?l)))
          (at end (carrying_slot_2 ?r ?a))
      )
  )

  (:durative-action release_artifact_slot_2
      :parameters (?r - robot ?a - artifact ?l - location)
      :duration (= ?duration 2)
      :condition (and 
            (at start (can_carry_two ?r))
            (at start (carrying_slot_2 ?r ?a))
            (over all (robot_at ?r ?l))
        )
      :effect (and 
          (at start (not (carrying_slot_2 ?r ?a)))
          (at end (hands_empty_slot_2 ?r))
          (at end (artifact_at ?a ?l))
      )
  )
)
