(define (problem single-robot-problem)
  (:domain single-robot)

  ;; -------------------------------------------------------------------------
  ;; OBJECT DEFINITIONS
  ;; -------------------------------------------------------------------------
  (:objects
    ;; Robot
    curator                     - robot

    ;; Pods
    pod1 pod2                   - pod

    ;; Locations
    entrance                    - location
    maintenance-tunnel          - location
    hall-a hall-b               - location
    cryo-chamber                - location
    anti-vibration-pods-room    - location
    stasis-lab                  - location

    ;; Artifact Types
    martian-core                - artifact-type
    martian-generic             - artifact-type
    martian-civilization        - artifact-type                                 
    asteroid-generic            - artifact-type           
    venus-generic               - artifact-type    
      
    ;; Artifacts 
    ; Martian Core Artifacts
    mart-nord-core-drill         - artifact
    mart-sud-core-drill          - artifact
    mart-east-core-drill         - artifact
    mart-west-core-drill         - artifact
    
    ; Martian Generic Artifacts
    mart-sand-sample            - artifact
    mart-north-pole-ice-sample  - artifact
    mart-mysterious-egg         - artifact
      
    ; Martian Civilization Artifacts
    mart-laser-gun              - artifact
    mart-pink-hat               - artifact
      
    ; Asteroid Generic Artifacts
    asteroid-MG04TN-ice-sample  - artifact
    asteroid-AD29TV-rock-sample - artifact
    
    ; Venus Generic Artifacts
    venus-sand-sample           - artifact
    venus-rock-sample           - artifact
  )

  ;; -------------------------------------------------------------------------
  ;; INITIAL STATE
  ;; -------------------------------------------------------------------------
  (:init
    
    ;; --- POD STATE ---
    (pod-empty pod1) 
    (pod-empty pod2)
    ;; FIX: 'contains-empty-pod' does not exist in domain. Used 'pod-at'.
    (pod-at pod1 anti-vibration-pods-room)
    (pod-at pod2 anti-vibration-pods-room)

    ;; --- TOPOLOGY ---
    (connected entrance maintenance-tunnel)          (connected maintenance-tunnel entrance)
    (connected maintenance-tunnel hall-a)            (connected hall-a maintenance-tunnel)
    (connected maintenance-tunnel hall-b)            (connected hall-b maintenance-tunnel)
    (connected maintenance-tunnel cryo-chamber)      (connected cryo-chamber maintenance-tunnel)
    (connected maintenance-tunnel anti-vibration-pods-room) (connected anti-vibration-pods-room maintenance-tunnel)
    (connected maintenance-tunnel stasis-lab)        (connected stasis-lab maintenance-tunnel)

    ;; --- ATMOSPHERE ---
    (is-pressurized entrance)                   (is-pressurized hall-a)
    (is-pressurized hall-b)                     (is-pressurized cryo-chamber)
    (is-pressurized anti-vibration-pods-room)   (is-pressurized stasis-lab)
    
    (is-unpressurized maintenance-tunnel)

    ;; --- SEISMIC / SAFETY STATUS ---
    ;; Safe rooms
    (is-unseismic entrance)
    (is-unseismic hall-a)
    (is-unseismic cryo-chamber)
    (is-unseismic anti-vibration-pods-room)
    (is-unseismic maintenance-tunnel)
    (is-unseismic stasis-lab)

    ;; Seismic rooms (Hall B is dangerous)
    ;; We do NOT declare (is-safe hall-b) here. The robot must verify it.
    (is-seismic hall-b) 

    ;; --- ROOM SPECIALIZATIONS ---
    (is-standard-room hall-a)             (is-standard-room hall-b)
    (is-standard-room entrance)           (is-standard-room anti-vibration-pods-room) 
    (is-standard-room maintenance-tunnel) (is-standard-room stasis-lab)
    
    (is-chill-room cryo-chamber)

    ;; --- ROBOT INITIAL STATE ---
    (robot-at curator entrance)
    (hands-empty curator)
    (sealing-mode-off curator) ;; Initialize seal state
    
    ;; --- ARTIFACT TAXONOMY ---
    (is-type  mart-nord-core-drill         martian-core) 
    (is-type  mart-sud-core-drill          martian-core)
    (is-type  mart-east-core-drill         martian-core)
    (is-type  mart-west-core-drill         martian-core)

    (is-type  mart-sand-sample             martian-generic) 
    (is-type  mart-north-pole-ice-sample   martian-generic) 
    (is-type  mart-mysterious-egg          martian-generic) 

    (is-type  mart-laser-gun               martian-civilization)
    (is-type  mart-pink-hat                martian-civilization)

    (is-type  asteroid-MG04TN-ice-sample   asteroid-generic)
    (is-type  asteroid-AD29TV-rock-sample  asteroid-generic)

    (is-type  venus-sand-sample            venus-generic)
    (is-type  venus-rock-sample            venus-generic)
                        
    ;; --- ARTIFACT POSITIONS ---
    ;; Hall A
    (artifact-at  mart-nord-core-drill         hall-a)
    (artifact-at  mart-sud-core-drill          hall-a)
    (artifact-at  mart-east-core-drill         hall-a)
    (artifact-at  mart-west-core-drill         hall-a)
    (artifact-at  mart-north-pole-ice-sample   hall-a)
    (artifact-at  mart-mysterious-egg          hall-a)
    (artifact-at  asteroid-MG04TN-ice-sample   hall-a)

    ;; Hall B
    (artifact-at  mart-sand-sample             hall-b)
    (artifact-at  mart-laser-gun               hall-b)
    (artifact-at  mart-pink-hat                hall-b)  
    (artifact-at  asteroid-AD29TV-rock-sample  hall-b)  
    (artifact-at  venus-sand-sample            hall-b)
    (artifact-at  venus-rock-sample            hall-b)

    ;; --- ARTIFACT PROPERTIES (Temp) ---
    (warm  mart-nord-core-drill)          (warm  mart-sud-core-drill)
    (warm  mart-east-core-drill)          (warm  mart-west-core-drill)
    (warm  mart-north-pole-ice-sample)    (warm  mart-mysterious-egg)
    (warm  asteroid-MG04TN-ice-sample)    (warm  mart-sand-sample)
    (warm  mart-laser-gun)                (warm  mart-pink-hat)
    (warm  asteroid-AD29TV-rock-sample)   (warm  venus-sand-sample)
    (warm  venus-rock-sample)

    ;; --- ARTIFACT PROPERTIES (Fragility) ---
    ;; Items that require a POD
    (fragile  mart-sand-sample)
    (fragile  mart-laser-gun)
    (fragile  mart-pink-hat)  
    (fragile  asteroid-AD29TV-rock-sample)  
    (fragile  venus-sand-sample)
    (fragile  venus-rock-sample)

    ;; Items that can be moved by HAND
    (no-fragile mart-nord-core-drill)      (no-fragile mart-sud-core-drill)
    (no-fragile mart-east-core-drill)      (no-fragile mart-west-core-drill)
    (no-fragile mart-mysterious-egg)       (no-fragile asteroid-MG04TN-ice-sample)
    (no-fragile mart-north-pole-ice-sample)
  )

  ;; -------------------------------------------------------------------------
  ;; GOAL STATE
  ;; -------------------------------------------------------------------------
  (:goal (and
    ;; 1. Martian Cores: Must be COLD and in STASIS-LAB
    ;; Note: Robot must detour to Cryo-Chamber to cool them first!
    (artifact-at  mart-nord-core-drill         stasis-lab)  
    (artifact-at  mart-sud-core-drill          stasis-lab)   
    (artifact-at  mart-east-core-drill         stasis-lab) 
    (artifact-at  mart-west-core-drill         stasis-lab)
    
    (cold  mart-nord-core-drill)  
    (cold  mart-sud-core-drill)   
    (cold  mart-east-core-drill) 
    (cold  mart-west-core-drill)

    ;; 2. Biological/Ice Samples: Must be in CRYO-CHAMBER
    ;; Since they need to be in the cryo-chamber they are also cold
    (artifact-at  mart-north-pole-ice-sample   cryo-chamber)
    (artifact-at  mart-mysterious-egg          cryo-chamber)    
    (artifact-at  asteroid-MG04TN-ice-sample   cryo-chamber)
    
    ;; 3. Rescue Mission: Retrieve fragile items from Hall B (Seismic) to Hall A
    (artifact-at  mart-sand-sample             hall-a)
    (artifact-at  mart-laser-gun               hall-a)
    (artifact-at  mart-pink-hat                hall-a)  
    (artifact-at  asteroid-AD29TV-rock-sample  hall-a)  
    (artifact-at  venus-sand-sample            hall-a)
    (artifact-at  venus-rock-sample            hall-a) 
    )   
  )
)