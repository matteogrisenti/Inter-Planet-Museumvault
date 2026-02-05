(define (problem single-robot-problem)
  (:domain single-robot)

  (:objects
    ;; Robot
    curator

    ;; Locations (from map) 
    entrance                    - location
    maintenance-tunnel          - location
    hall-a hall-b               - location
    cryo-chamber                - location
    anti-vibration-pods-room    - location
    stasis-lab                  - location

    ;; Artifacts Tipology
    martian-core            - artifact-type
    martian-generic         - artifact-type
    martian-civilization    - artifact-type                                 
    asteroid-generic        - artifact-type           
    venus-generic           - artifact-type    
      
    ;; Articats 
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
    asteroid-AD-----rock-sample - artifact
    
    ; Venus Generic Artifacts
    venus-sand-sample           - artifact
    venus-rock-sample           - artifact
  )

  (:init
    ;; TOPOLOGY 
    ; Entrance leads to Tunnel.
    (connected entrance maintenance-tunnel) (connected maintenance-tunnel entrance)
    ; Assuming Tunnel connects to everything.
    (connected maintenance-tunnel hall-a)               (connected hall-a maintenance-tunnel)
    (connected maintenance-tunnel hall-b)               (connected hall-b maintenance-tunnel)
    (connected maintenance-tunnel cryo-chamber)         (connected cryo-chamber maintenance-tunnel)
    (connected maintenance-tunnel anti-vibration-pods)  (connected anti-vibration-pods-room maintenance-tunnel)
    (connected maintenance-tunnel stasis-lab)           (connected stasis-lab maintenance-tunnel)

    ;; ROOM PROPERTIES
    ;; Rooms are pressurized/safe. Tunnel is not. 
    (is-unpressurized maintenance-tunnel)
    ; (is-unsafe hall-b) we consider that at the beginning there is not a mart-quake, so the hall B is safe


    ;; ROBOT
    (at entrance)   ; Robot inital position is in the entrance
    (handempty)     ; Robot start without any object 

    
    ;; ARTIFACTS TYPOLOGY
    ; Martian Core Artifacts - martian-core
    (is-type  mart-nord-core-drill martian-core ) 
    (is-type  mart-sud-core-drill  martian-core )
    (is-type  mart-east-core-drill martian-core )
    (is-type  mart-west-core-drill martian-core )
    ; Martian Generic Artifacts - martian-generic 
    (is-type  mart-sand-sample           martian-generic ) 
    (is-type  mart-north-pole-ice-sample martian-generic ) 
    (is-type  mart-mysterious-egg        martian-generic ) 
    ; Martian Civilization Artifacts - martian-civilization
    (is-type  mart-laser-gun martian-civilization )
    (is-type  mart-pink-hat  martian-civilization )
    ; Asteroid Generic Artifacts - asteroid-generic
    (is-type  asteroid-MG04TN-ice-sample  martian-civilization )
    (is-type  asteroid-AD-----rock-sample martian-civilization )
    ; Venus Generic Artifacts - venus-generic
    (is-type  venus-sand-sample venus-generic )
    (is-type  venus-rock-sample venus-generic )
                        
    
    ;; ARTIFACTS INITIAL POSITION
    ; Hall A: The artifact that need to be cooled down in the Cryo Chamber 
    ;         The artifact that need to be cooled down are the Core Drill, Ice & Organic Sample
    ; Core Drill Artifact
    (at-artifact  mart-nord-core-drill        hall-a )
    (at-artifact  mart-sud-core-drill         hall-a )
    (at-artifact  mart-east-core-drill        hall-a )
    (at-artifact  mart-west-core-drill        hall-a )
    ; Ice & Organic Sample
    (at-artifact  mart-north-pole-ice-sample  hall-a )
    (at-artifact  mart-mysterious-egg         hall-a )
    (at-artifact  asteroid-MG04TN-ice-sample  hall-a )

    ; Hall B: All the other stuffs
    (at-artifact  mart-sand-sample            hall-b )
    (at-artifact  mart-laser-gun              hall-b )
    (at-artifact  mart-pink-hat               hall-b )  
    (at-artifact  asteroid-AD-----rock-sample hall-b )  
    (at-artifact  venus-sand-sample           hall-b )
    (at-artifact  venus-rock-sample           hall-b )


    ;; ARTIFACTS INITIAL FEATURES
    ; Need Chill  -  need-chill
    (need-chill  mart-nord-core-drill )
    (need-chill  mart-sud-core-drill  )
    (need-chill  mart-east-core-drill )
    (need-chill  mart-west-core-drill )
    (need-chill  mart-north-pole-ice-sample )
    (need-chill  mart-mysterious-egg  )
    (need-chill  asteroid-MG04TN-ice-sample )
    ; Need Anti Vibration Pods  -  need-anti-vibration-pods
    (need-anti-vibration-pods  mart-sand-sample  )
    (need-anti-vibration-pods  mart-laser-gun    )
    (need-anti-vibration-pods  mart-pink-hat     )  
    (need-anti-vibration-pods  asteroid-AD-----rock-sample )  
    (need-anti-vibration-pods  venus-sand-sample )
    (need-anti-vibration-pods  venus-rock-sample )
  )

  ;; The goal is reached when all the artifact where bringed in their final destitation:
  ;     - Mart Core Artifacts need to be inside the Stasis-Lab
  ;     - The other artifact that need to be cold are in the Cry Chamber 
  ;     - The artifact in Hall B must be retrive and replaced in the Hall A ( evacuated from the danger room )
  
  ; Nota 4: We can add the possibility to the other artifact that need to be cold to be both in the Cry Chamber or 
  ; in the Stasis-Lab. In this way we test if the planner is able to reach the optimal goal or a suboptimal case

  :goal( and (
      
    ; Mart Core Artifacts need to be inside the Stasis-Lab
    (at-artifact  mart-nord-core-drill        stasis-lab )
    (at-artifact  mart-sud-core-drill         stasis-lab )
    (at-artifact  mart-east-core-drill        stasis-lab )
    (at-artifact  mart-west-core-drill        stasis-lab )
    ; The other artifact that need to be cold are in the Cry Chamber 
    (at-artifact  mart-north-pole-ice-sample  cryo-chamber )
    (at-artifact  mart-mysterious-egg         cryo-chamber )    
    (at-artifact  asteroid-MG04TN-ice-sample  cryo-chamber )
    ; The artifact in Hall B must be retrive and replaced in the Hall A 
    (at-artifact  mart-sand-sample            hall-a )
    (at-artifact  mart-laser-gun              hall-a )
    (at-artifact  mart-pink-hat               hall-a )  
    (at-artifact  asteroid-AD-----rock-sample hall-v )  
    (at-artifact  venus-sand-sample           hall-a )
    (at-artifact  venus-rock-sample           hall-a )
    
  ))