(define (problem temporal-multi-robot-problem)
  (:domain temporal-multi-robot)

  (:objects
    ;; --- Robots ---
    curator technician scientist - robot

    ;; --- Pods ---
    pod1 pod2 - pod
    
    ;; --- Locations ---
    entrance maintenance-tunnel - location
    hall-a hall-b cryo-chamber - location
    anti-vibration-pods-room stasis-lab - location

    ;; --- Typologies ---
    technological scientific top-secret - artifact-type
      
    ;; --- Artifacts ---
    mart-nord-core-drill mart-sud-core-drill - artifact
    mart-east-core-drill mart-west-core-drill - artifact
    mart-sand-sample mart-north-pole-ice-sample - artifact
    mart-mysterious-egg mart-laser-gun mart-pink-hat - artifact
    rover-wheel space-suit quantum-chip - artifact
    asteroid-MG04TN-ice-sample asteroid-AD29TV-rock-sample - artifact
    venus-sand-sample venus-rock-sample - artifact
  )

  (:init
    ;; ============================================================
    ;; ROBOT CAPABILITIES & INITIAL STATE
    ;; ============================================================
    
    ;; Curator
    (sealing-mode-off curator)
    (robot-at curator entrance) (hands-empty curator)
    (can-access curator entrance) (can-access curator maintenance-tunnel) 
    (can-access curator hall-a) (can-access curator hall-b) 
    (can-access curator cryo-chamber) (can-access curator anti-vibration-pods-room)
    (can-pickup curator scientific) (can-pickup curator top-secret)

    ;; Technician
    (sealing-mode-off technician)
    (robot-at technician entrance) (hands-empty technician) (can-carry-two technician) (second-slot-empty technician)
    
    (can-access technician entrance) (can-access technician maintenance-tunnel) 
    (can-access technician hall-a) (can-access technician hall-b) 
    (can-access technician cryo-chamber) (can-access technician anti-vibration-pods-room)
    (can-pickup technician technological)

    ;; Scientist
    (sealing-mode-off scientist)
    (robot-at scientist stasis-lab) (hands-empty scientist)
    (can-access scientist stasis-lab) (can-access scientist maintenance-tunnel)
    (can-pickup scientist scientific) (can-pickup scientist top-secret) (can-pickup scientist technological)
    
    ;; ============================================================
    ;; WORLD TOPOLOGY & PROPERTIES
    ;; ============================================================
    
    ;; Connections
    (connected entrance maintenance-tunnel) (connected maintenance-tunnel entrance)
    (connected maintenance-tunnel hall-a) (connected hall-a maintenance-tunnel)
    (connected maintenance-tunnel hall-b) (connected hall-b maintenance-tunnel)
    (connected maintenance-tunnel cryo-chamber) (connected cryo-chamber maintenance-tunnel)
    (connected maintenance-tunnel anti-vibration-pods-room) (connected anti-vibration-pods-room maintenance-tunnel)
    (connected maintenance-tunnel stasis-lab) (connected stasis-lab maintenance-tunnel)

    ;; Pressure & Safety
    (is-unpressurized maintenance-tunnel)
    (is-pressurized entrance) (is-pressurized hall-a) (is-pressurized hall-b) 
    (is-pressurized cryo-chamber) (is-pressurized anti-vibration-pods-room) (is-pressurized stasis-lab)

    ;; Static safe rooms (Hall B is handled via TILs above)
    (is-safe entrance) (is-safe hall-a) (is-safe cryo-chamber)
    (is-safe anti-vibration-pods-room) (is-safe maintenance-tunnel) (is-safe stasis-lab)
    
    ;; ============================================================
    ;; SEISMIC ACTIVITY (TIMED INITIAL LITERALS)
    ;; ============================================================
    ;; Hall B starts as safe.
    (not (is-safe hall-b))

    (at 10 (is-safe hall-b))
    
    ;; First Earthquake window: Unsafe between time 20 and 40
    ; (at 20 (not (is-safe hall-b)))
    ; (at 40 (is-safe hall-b))

    ;; Second Earthquake window: Unsafe between time 70 and 90
    ; (at 70 (not (is-safe hall-b)))
    ; (at 90 (is-safe hall-b))

    ;; Special Room Properties
    (is-chill-room cryo-chamber)

    ;; Pods
    (pod-empty pod1) (pod-empty pod2)
    (contains-empty-pod anti-vibration-pods-room pod1) (contains-empty-pod anti-vibration-pods-room pod2)
  )

  ;; ============================================================
  ;; GOAL STATE
  ;; ============================================================
  (:goal (and
    
    (robot-at curator hall-a)
    (carrying-empty-pod curator pod1)

    ;; Final Locations
    ; (artifact-at mart-nord-core-drill stasis-lab) (cold mart-nord-core-drill)
    ; (artifact-at mart-sud-core-drill stasis-lab) (cold mart-sud-core-drill)
    ; (artifact-at mart-east-core-drill stasis-lab) (cold mart-east-core-drill)
    ; (artifact-at mart-west-core-drill stasis-lab) (cold mart-west-core-drill)
    ; (artifact-at rover-wheel stasis-lab)
    ; (artifact-at space-suit stasis-lab)
    ; (artifact-at quantum-chip stasis-lab) (cold quantum-chip)

    ; (artifact-at mart-north-pole-ice-sample cryo-chamber)
    ; (artifact-at mart-mysterious-egg cryo-chamber)    
    ; (artifact-at asteroid-MG04TN-ice-sample cryo-chamber)

    ; (artifact-at mart-sand-sample hall-a)
    ; (artifact-at mart-laser-gun hall-a)
    ; (artifact-at mart-pink-hat hall-a)  
    ; (artifact-at asteroid-AD29TV-rock-sample hall-a)  
    ; (artifact-at venus-sand-sample hall-a)
    ; (artifact-at venus-rock-sample hall-a)
    )   
  )
)