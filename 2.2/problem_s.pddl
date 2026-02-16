(define (problem multi-robot-micro-problem)
  (:domain multi-robot)

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
      
    ;; --- Artifacts (REDUCED FOR TESTING) ---
    mart-nord-core-drill - artifact
    quantum-chip - artifact
  )

  (:init
    ;; ============================================================
    ;; ROBOT CAPABILITIES & INITIAL STATE
    ;; ============================================================
    
    ;; Curator
    (robot-at curator entrance) (hands-empty-slot-1 curator)
    (can-access curator entrance) (can-access curator maintenance-tunnel) 
    (can-access curator hall-a) (can-access curator hall-b) 
    (can-access curator cryo-chamber) (can-access curator anti-vibration-pods-room)

    (can-pickup curator scientific) (can-pickup curator top-secret)

    ;; Technician
    (robot-at technician entrance)      (can-carry-two technician)
    (hands-empty-slot-1 technician)     (hands-empty-slot-2 technician)

    (can-access technician entrance) (can-access technician maintenance-tunnel) 
    (can-access technician hall-a) (can-access technician hall-b) 
    (can-access technician cryo-chamber) (can-access technician anti-vibration-pods-room)

    (can-pickup technician technological)

    ;; Scientist
    (robot-at scientist stasis-lab)   (hands-empty-slot-1 scientist)

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

    (is-safe entrance) (is-safe hall-a) (is-safe cryo-chamber)
    (is-safe anti-vibration-pods-room) (is-safe maintenance-tunnel) (is-safe stasis-lab)
    
    ;; Special Room Properties
    (is-seismic hall-b)
    (is-chill-room cryo-chamber)
    (is-standard-room hall-a) (is-standard-room hall-b) (is-standard-room entrance) 
    (is-standard-room anti-vibration-pods-room) (is-standard-room maintenance-tunnel) (is-standard-room stasis-lab)

    ;; Pods
    (pod-empty pod1) (pod-empty pod2)
    (pod-at pod1 anti-vibration-pods-room ) (pod-at pod2 anti-vibration-pods-room)

    ;; ============================================================
    ;; ARTIFACTS BY ROOM (Position, Type, Features)
    ;; ============================================================

    ;; Artifact 1: Standard transport, requires cooling, 'Scientific' capability
    (artifact-at mart-nord-core-drill hall-a) 
    (is-type mart-nord-core-drill scientific) 
    (warm mart-nord-core-drill) 
    (no-fragile mart-nord-core-drill)
    
    ;; Artifact 2: Seismic room, fragile (needs pod), requires cooling, 'Technological' capability
    (artifact-at quantum-chip hall-b) 
    (is-type quantum-chip technological) 
    (warm quantum-chip) 
    (fragile quantum-chip)
  )

  ;; ============================================================
  ;; GOAL STATE
  ;; ============================================================
  (:goal (and
    (artifact-at mart-nord-core-drill stasis-lab) (cold mart-nord-core-drill)
    (artifact-at quantum-chip stasis-lab) (cold quantum-chip)
  ))
)