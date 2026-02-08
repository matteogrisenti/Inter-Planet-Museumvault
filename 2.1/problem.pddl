(define (problem single-robot-problem)
  (:domain single-robot)

  (:objects
    ;; Robot
    curator                     - robot
    curator2                    - robot

    ;; Pods
    pod1                        - pod
    pod2                        - pod

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

  (:init
    
    ;; PODS
    (pod-empty pod1) (pod-empty pod2) ; Both pods are empty at the beginning

    ;; TOPOLOGY 
    ; Entrance leads to Tunnel.
    (connected entrance maintenance-tunnel) (connected maintenance-tunnel entrance)
    ; Assuming Tunnel connects to everything.
    (connected maintenance-tunnel hall-a)                    (connected hall-a maintenance-tunnel)
    (connected maintenance-tunnel hall-b)                    (connected hall-b maintenance-tunnel)
    (connected maintenance-tunnel cryo-chamber)              (connected cryo-chamber maintenance-tunnel)
    (connected maintenance-tunnel anti-vibration-pods-room)  (connected anti-vibration-pods-room maintenance-tunnel)
    (connected maintenance-tunnel stasis-lab)                (connected stasis-lab maintenance-tunnel)

    ;; ROOM PRESSUR PROPERTIES 
    ;; Rooms are pressurized/safe. 
    (is-pressurized entrance )                    (is-pressurized hall-a )
    (is-pressurized hall-b   )                    (is-pressurized cryo-chamber )
    (is-pressurized anti-vibration-pods-room )    (is-pressurized stasis-lab )
    ;; Tunnel is not. 
    (is-unpressurized maintenance-tunnel)

    ;; ROOM SAFTY PROPERTIES 
    ;; All the roms are safe at the beginning. 
    (is-safe entrance )                   (is-safe hall-a   )
    ;; (is-safe hall-b   )                   
    (is-safe cryo-chamber )
    (is-safe anti-vibration-pods-room )   (is-safe maintenance-tunnel )
    (is-safe stasis-lab )

    ;; CHECKED AND UNCHECKED PROPERTIES
    (checked entrance )                   (checked hall-a)
    (checked cryo-chamber)      (checked anti-vibration-pods-room ) (checked maintenance-tunnel ) (checked stasis-lab )
    (safety-unknown hall-b) ; !Hall B is the only room that need to be checked by the robot, becouse it is the only one that is not safe at the beginning.
    
    ;; ROOM DROPOUT TYPE PROPERTIES
    (is-standard-room hall-a )             (is-standard-room hall-b )
    (is-standard-room entrance )           (is-standard-room anti-vibration-pods-room ) 
    (is-standard-room maintenance-tunnel ) (is-standard-room stasis-lab)
    ;; chiller rooms
    (is-chill-room cryo-chamber )

    ;; ROOM PICKUP TYPE PROPERTY
    (contains-empty-pod anti-vibration-pods-room pod1)  ; Pod1 is in the anti-vibration-pods-room at the beginning
    (contains-empty-pod anti-vibration-pods-room pod2)  ; Pod2 is in the anti-vibration-pods-room at the beginning
    
    
    ;; ROBOT
    (robot-at curator entrance)   ; Robot inital position is in the entrance
    (hands-empty curator)          ; Robot start without any object 
    
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
    (is-type  asteroid-MG04TN-ice-sample  asteroid-generic )
    (is-type  asteroid-AD29TV-rock-sample asteroid-generic )
    ; Venus Generic Artifacts - venus-generic
    (is-type  venus-sand-sample venus-generic )
    (is-type  venus-rock-sample venus-generic )
                        
    
    ;; ARTIFACTS INITIAL POSITION
    ; Hall A: The artifact that need to be cooled down in the Cryo Chamber 
    ;         The artifact that need to be cooled down are the Core Drill, Ice & Organic Sample
    ; Core Drill Artifact
    (artifact-at  mart-nord-core-drill        hall-a )
    (artifact-at  mart-sud-core-drill         hall-a )
    (artifact-at  mart-east-core-drill        hall-a )
    (artifact-at  mart-west-core-drill        hall-a )
    ; Ice & Organic Sample
    (artifact-at  mart-north-pole-ice-sample  hall-a )
    (artifact-at  mart-mysterious-egg         hall-a )
    (artifact-at  asteroid-MG04TN-ice-sample  hall-a )

    ; Hall B: All the other stuffs
    (artifact-at  mart-sand-sample            hall-b )
    (artifact-at  mart-laser-gun              hall-b )
    (artifact-at  mart-pink-hat               hall-b )  
    (artifact-at  asteroid-AD29TV-rock-sample hall-b )  
    (artifact-at  venus-sand-sample           hall-b )
    (artifact-at  venus-rock-sample           hall-b )

 
    ;; ARTIFACTS INITIAL FEATURES
    ; Temperature Features
    (warm  mart-nord-core-drill )          (warm  mart-sud-core-drill  )
    (warm  mart-east-core-drill )          (warm  mart-west-core-drill )
    (warm  mart-north-pole-ice-sample )    (warm  mart-mysterious-egg  )
    (warm  asteroid-MG04TN-ice-sample )    (warm  mart-sand-sample )
    (warm  mart-laser-gun )                (warm  mart-pink-hat )
    (warm  asteroid-AD29TV-rock-sample )   (warm  venus-sand-sample )
    (warm  venus-rock-sample )

    ; Need Anti Vibration Pods  -  fragile
    (fragile  mart-sand-sample  )
    (fragile  mart-laser-gun    )
    (fragile  mart-pink-hat     )  
    (fragile  asteroid-AD29TV-rock-sample )  
    (fragile  venus-sand-sample )
    (fragile  venus-rock-sample )

    ; Artifacl in Hall A - no-fragile
    (no-fragile mart-nord-core-drill )      (no-fragile mart-sud-core-drill )
    (no-fragile mart-east-core-drill )      (no-fragile mart-west-core-drill )
    (no-fragile mart-mysterious-egg  )      (no-fragile asteroid-MG04TN-ice-sample )
    (no-fragile mart-north-pole-ice-sample )
  )

  ;; The goal is reached when all the artifact where bringed in their final destitation:
  ;     - Mart Core Artifacts need to be cold and inside the Stasis-Lab
  ;     - The other artifact that need to be cold are in the Cry Chamber 
  ;     - The artifact in Hall B must be retrive and replaced in the Hall A ( evacuated from the danger room )
  
  ; Nota 4: We can add the possibility to the other artifact that need to be cold to be both in the Cry Chamber or 
  ; in the Stasis-Lab. In this way we test if the planner is able to reach the optimal goal or a suboptimal case

  (:goal (and
    ; LOCATION CONSTRAINTS 
    ; Mart Core Artifacts need to be inside the Stasis-Lab
    (artifact-at  mart-nord-core-drill        stasis-lab )  
    (artifact-at  mart-sud-core-drill         stasis-lab )   
    (artifact-at  mart-east-core-drill        stasis-lab ) 
    (artifact-at  mart-west-core-drill        stasis-lab )
    ; The other artifact that need to be cold are in the Cry Chamber 
    (artifact-at  mart-north-pole-ice-sample  cryo-chamber )
    (artifact-at  mart-mysterious-egg         cryo-chamber )    
    (artifact-at  asteroid-MG04TN-ice-sample  cryo-chamber )
    ; The artifact in Hall B must be retrive and replaced in the Hall A 
    (artifact-at  mart-sand-sample            hall-a )
    (artifact-at  mart-laser-gun              hall-a )
    (artifact-at  mart-pink-hat               hall-a )  
    (artifact-at  asteroid-AD29TV-rock-sample hall-a )  
    (artifact-at  venus-sand-sample           hall-a )
    (artifact-at  venus-rock-sample           hall-a ) 

    ; TEMPERATURE CONSTRAINT
    (cold  mart-nord-core-drill )  
    (cold  mart-sud-core-drill  )   
    (cold  mart-east-core-drill ) 
    (cold  mart-west-core-drill )
    ;; the artifact that need to be in the cry chamber are for default cold; becouse the domain
    ;; is modelled as instant cool down process, so they can be homitted
    ; (cold  mart-north-pole-ice-sample )
    ; (cold  mart-mysterious-egg        )    
    ; (cold  asteroid-MG04TN-ice-sample )
    )   
  )
)