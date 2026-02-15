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
    (is-safe hall-b)
    
    ;; First Earthquake window
    (at 10 (not (is-safe hall-b)))
    (at 31 (is-safe hall-b))

    ;; Second Earthquake window
    (at 52 (not (is-safe hall-b)))
    (at 67 (is-safe hall-b))

    ;; Third Earthquake window
    (at 81 (not (is-safe hall-b)))
    (at 94 (is-safe hall-b))

    ; Fourth Earthquake window
    (at 109 (not (is-safe hall-b)))
    (at 120 (is-safe hall-b))

    ; Fifth Earthquake window
    (at 158 (not (is-safe hall-b)))
    (at 169 (is-safe hall-b))

    ;; Special Room Properties
    (is-chill-room cryo-chamber)

    ;; Pods
    (pod-empty pod1) (pod-empty pod2)
    (contains-empty-pod anti-vibration-pods-room pod1) (contains-empty-pod anti-vibration-pods-room pod2)

    ;; ============================================================
    ;; ARTIFACTS BY ROOM (Position, Type, Features)
    ;; ============================================================

    ;; --- HALL A ---
    ;; Martian Core Drills
    (artifact-at mart-nord-core-drill hall-a) (is-type mart-nord-core-drill scientific) (warm mart-nord-core-drill) (no-fragile mart-nord-core-drill)
    ; (artifact-at mart-sud-core-drill hall-a) (is-type mart-sud-core-drill scientific) (warm mart-sud-core-drill) (no-fragile mart-sud-core-drill)
    ; (artifact-at mart-east-core-drill hall-a) (is-type mart-east-core-drill scientific) (warm mart-east-core-drill) (no-fragile mart-east-core-drill)
    ; (artifact-at mart-west-core-drill hall-a) (is-type mart-west-core-drill scientific) (warm mart-west-core-drill) (no-fragile mart-west-core-drill)
    
    ; ;; Hall A: Samples & Mysterious Egg
    (artifact-at mart-north-pole-ice-sample hall-a) (is-type mart-north-pole-ice-sample scientific) (warm mart-north-pole-ice-sample) (no-fragile mart-north-pole-ice-sample)
    (artifact-at mart-mysterious-egg hall-a) (is-type mart-mysterious-egg top-secret) (warm mart-mysterious-egg) (no-fragile mart-mysterious-egg)
    ; (artifact-at asteroid-MG04TN-ice-sample hall-a) (is-type asteroid-MG04TN-ice-sample scientific) (warm asteroid-MG04TN-ice-sample) (no-fragile asteroid-MG04TN-ice-sample)

    ;; --- HALL B ---
    ;; Mission Gear
    (artifact-at rover-wheel hall-b) (is-type rover-wheel technological) (warm rover-wheel) (no-fragile rover-wheel)
    ; (artifact-at space-suit hall-b) (is-type space-suit technological) (warm space-suit) (no-fragile space-suit)
    (artifact-at quantum-chip hall-b) (is-type quantum-chip technological) (warm quantum-chip)

    ;; Hall B: Samples & Civilization Artifacts
    (artifact-at mart-sand-sample hall-b) (is-type mart-sand-sample scientific) (warm mart-sand-sample)
    (artifact-at mart-laser-gun hall-b) (is-type mart-laser-gun top-secret) (warm mart-laser-gun)
    ; (artifact-at mart-pink-hat hall-b) (is-type mart-pink-hat top-secret) (warm mart-pink-hat)
    ; (artifact-at asteroid-AD29TV-rock-sample hall-b) (is-type asteroid-AD29TV-rock-sample scientific) (warm asteroid-AD29TV-rock-sample)
    ; (artifact-at venus-sand-sample hall-b) (is-type venus-sand-sample scientific) (warm venus-sand-sample) (no-fragile venus-sand-sample)
    ; (artifact-at venus-rock-sample hall-b) (is-type venus-rock-sample scientific) (warm venus-rock-sample) (no-fragile venus-rock-sample)
  )

  ;; ============================================================
  ;; GOAL STATE
  ;; ============================================================
  (:goal (and
    ;; Final Locations
    (artifact-at mart-nord-core-drill stasis-lab) (cold mart-nord-core-drill)
    ; (artifact-at mart-sud-core-drill stasis-lab) (cold mart-sud-core-drill)
    ; (artifact-at mart-east-core-drill stasis-lab) (cold mart-east-core-drill)
    ; (artifact-at mart-west-core-drill stasis-lab) (cold mart-west-core-drill)
    ; (artifact-at rover-wheel stasis-lab)
    ; (artifact-at space-suit stasis-lab)
    (artifact-at quantum-chip stasis-lab) (cold quantum-chip)

    (artifact-at mart-north-pole-ice-sample cryo-chamber)
    (artifact-at mart-mysterious-egg cryo-chamber)    
    ; (artifact-at asteroid-MG04TN-ice-sample cryo-chamber)

    (artifact-at mart-sand-sample hall-a)
    (artifact-at mart-laser-gun hall-a)
    ; (artifact-at mart-pink-hat hall-a)  
    ; (artifact-at asteroid-AD29TV-rock-sample hall-a)  
    ; (artifact-at venus-sand-sample hall-a)
    ; (artifact-at venus-rock-sample hall-a)
    )   
  )
)