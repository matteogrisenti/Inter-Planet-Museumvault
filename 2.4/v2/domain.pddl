(define (domain temporal-multi-robot)
  (:requirements :strips :typing :timed-initial-literals :durative-actions)

  (:types
    robot             
    pod               
    location          
    artifact          
    artifact-type     
  )

  (:predicates
    (robot-at ?r - robot ?l - location)
    (hands-empty ?r - robot)
    (sealing-mode-on ?r - robot) 
    (sealing-mode-off ?r - robot)                      
    
    (carrying ?r - robot ?a - artifact)             
    (carrying-empty-pod ?r - robot ?p - pod)        
    (carrying-full-pod ?r - robot ?p - pod)         
    (carrying-second-object ?r - robot ?a - artifact) 
    (second-slot-empty ?r - robot)                  

    (can-access ?r - robot ?l - location)           
    (can-pickup ?r - robot ?at - artifact-type)     
    (can-carry-two ?r - robot)                      

    (connected ?l1 ?l2 - location)
    (is-pressurized ?l - location)
    (is-unpressurized ?l - location)
    (is-safe ?l - location)                         
    (is-chill-room ?l - location)                   

    (artifact-at ?a - artifact ?l - location)
    (is-type ?a - artifact ?t - artifact-type)
    (fragile ?a - artifact)         
    (no-fragile ?a - artifact)
    (warm ?a - artifact)
    (cold ?a - artifact)
    
    (contains-empty-pod ?l - location ?p - pod)
    (contains-full-pod ?l - location ?p - pod)
    (pod-empty ?p - pod)
    (pod-contains ?p - pod ?a - artifact)           
  )

  ;; ========================
  ;; MOVEMENT & SEALING (Invariati)
  ;; ========================

  (:durative-action move-to-pressurized-room
    :parameters (?r - robot ?from ?to - location)
    :duration (= ?duration 10)
    :condition (and 
        (at start (robot-at ?r ?from))   
        (at start (connected ?from ?to))        
        (at start (is-pressurized ?to))         
        (over all (is-safe ?to))                
        (at start (can-access ?r ?to))
    )
    :effect (and 
        (at start (not (robot-at ?r ?from))) 
        (at end (robot-at ?r ?to))          
        (at end (not (sealing-mode-on ?r)))   
        (at end (sealing-mode-off ?r))      
        )
  )

  (:durative-action move-to-unpressurized-room
    :parameters (?r - robot ?from ?to - location)
    :duration (= ?duration 10)
    :condition (and 
        (at start (robot-at ?r ?from))   
        (at start (sealing-mode-on ?r))           
        (at start (connected ?from ?to))          
        (at start (is-unpressurized ?to))         
        (at start (can-access ?r ?to))            
    )
    :effect (and 
        (at start (not (robot-at ?r ?from))) 
        (at end (robot-at ?r ?to))
    )
  )

  (:durative-action activate-seal
    :parameters (?r - robot)
    :duration (= ?duration 2)
    :condition (at start (sealing-mode-off ?r))
    :effect (and 
        (at start (not (sealing-mode-off ?r)))
        (at end (sealing-mode-on ?r))
        )
  )

  ;; ========================
  ;; POD MANAGEMENT (Invariati)
  ;; ========================

  (:durative-action pick-up-empty-pod
    :parameters (?r - robot ?l - location ?p - pod)
    :duration (= ?duration 2)
    :condition (and 
        (at start (hands-empty ?r))                
        (at start (contains-empty-pod ?l ?p))      
        (over all (pod-empty ?p))                  
        (over all (robot-at ?r ?l))                
    )
    :effect (and 
        (at start (not (contains-empty-pod ?l ?p))) 
        (at start (not (hands-empty ?r)))           
        (at end (carrying-empty-pod ?r ?p))         
    )
  )
  
  (:durative-action drop-empty-pod
    :parameters (?r - robot ?p - pod ?l - location)
    :duration (= ?duration 2)
    :condition (and 
        (at start (carrying-empty-pod ?r ?p))      
        (over all (robot-at ?r ?l))                
        (over all (pod-empty ?p))                  
    )
    :effect (and 
        (at start (not (carrying-empty-pod ?r ?p)))
        (at end (hands-empty ?r))
        (at end (contains-empty-pod ?l ?p))       
    )
  )
  
  ;; GENERIC DROP ACTIONS (Can remain generic)
  
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

  (:durative-action drop-full-pod
     :parameters (?r - robot ?p - pod ?l - location)
     :duration (= ?duration 4)
     :condition (and 
         (at start (carrying-full-pod ?r ?p))           
         (over all (robot-at ?r ?l))                    
     )
     :effect (and 
         (at start (not (carrying-full-pod ?r ?p)))     
         (at end (hands-empty ?r))                      
         (at end (contains-full-pod ?l ?p))             
     )
 )

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
        (at start (not (pod-contains ?p ?a))) 
        (at end (artifact-at ?a ?l))
        (at end (pod-empty ?p))
        (at end (carrying-empty-pod ?r ?p))      
    )
  )

  ;; ==========================================================
  ;; SPECIALIZED PICK UP ACTIONS (NO FRAGILE)
  ;; ==========================================================
  
  ;; 1. SCIENTIFIC -> CURATOR & SCIENTIST
  ;; Richiede solo che il robot sia abilitato al tipo scientifico.
  (:durative-action pick-up-SCIENTIFIC-artifact
    :parameters (?a - artifact ?l - location ?r - robot)
    :duration (= ?duration 1)
    :condition (and 
        (at start (artifact-at ?a ?l))
        (at start (hands-empty ?r))
        (over all (robot-at ?r ?l))
        (at start (no-fragile ?a))
        
        ;; TYPE CHECKS (Positive Only)
        (at start (is-type ?a scientific))
        (at start (can-pickup ?r scientific))
        ;; Abbiamo rimosso i vincoli negativi sugli altri tipi per evitare errori
    )
    :effect (and 
        (at start (not (artifact-at ?a ?l)))
        (at start (not (hands-empty ?r))) 
        (at end (carrying ?r ?a))            
    )
  )

  ;; 2. TECHNOLOGICAL -> TECHNICIAN Only
  ;; Qui possiamo usare "can-carry-two" come filtro positivo perché solo il Tecnico ce l'ha!
  (:durative-action pick-up-TECHNOLOGICAL-artifact
    :parameters (?a - artifact ?l - location ?r - robot)
    :duration (= ?duration 1)
    :condition (and 
        (at start (artifact-at ?a ?l))
        (at start (hands-empty ?r))
        (over all (robot-at ?r ?l))
        (at start (no-fragile ?a))
        
        ;; TYPE CHECKS
        (at start (is-type ?a technological))
        (at start (can-pickup ?r technological))

        ;; SYMMETRY BREAKING (Positive Only)
        ;; Solo il tecnico ha questa flag, quindi forza l'uso del tecnico senza usare "not"
        (at start (can-carry-two ?r))
    )
    :effect (and 
        (at start (not (artifact-at ?a ?l)))
        (at start (not (hands-empty ?r))) 
        (at end (carrying ?r ?a))            
    )
  )

  ;; 3. TOP-SECRET -> SCIENTIST & CURATOR
  ;; Rimosso il vincolo "not carry two". Ora sia Scienziato che Curatore possono farlo.
  ;; Questo risolve il bug dell'uovo misterioso.
  (:durative-action pick-up-TOP-SECRET-artifact
    :parameters (?a - artifact ?l - location ?r - robot)
    :duration (= ?duration 1)
    :condition (and 
        (at start (artifact-at ?a ?l))
        (at start (hands-empty ?r))
        (over all (robot-at ?r ?l))
        (at start (no-fragile ?a))
        
        ;; TYPE CHECKS
        (at start (is-type ?a top-secret))
        (at start (can-pickup ?r top-secret))
    )
    :effect (and 
        (at start (not (artifact-at ?a ?l)))
        (at start (not (hands-empty ?r))) 
        (at end (carrying ?r ?a))            
    )
  )

  ;; ==========================================================
  ;; SPECIALIZED PUT-IN-POD ACTIONS (FRAGILE)
  ;; ==========================================================

  ;; 1. PUT IN POD - SCIENTIFIC
  (:durative-action put-in-pod-SCIENTIFIC
    :parameters (?a - artifact ?l - location ?r - robot ?p - pod)
    :duration (= ?duration 4)
    :condition (and 
        (at start (artifact-at ?a ?l))         
        (at start (carrying-empty-pod ?r ?p))
        (at start (pod-empty ?p))
        (over all (robot-at ?r ?l))
        (at start (fragile ?a)) ;; Assumiamo che tu abbia aggiunto "fragile" nel problem file
        
        (at start (is-type ?a scientific))
        (at start (can-pickup ?r scientific))
    )
    :effect (and 
        (at start (not (artifact-at ?a ?l)))
        (at start (not (carrying-empty-pod ?r ?p)))
        (at start (not (pod-empty ?p)))
        (at end (carrying-full-pod ?r ?p))  
        (at end (pod-contains ?p ?a))
    )
  )

  ;; 2. PUT IN POD - TECHNOLOGICAL (Technician)
  (:durative-action put-in-pod-TECHNOLOGICAL
    :parameters (?a - artifact ?l - location ?r - robot ?p - pod)
    :duration (= ?duration 4)
    :condition (and 
        (at start (artifact-at ?a ?l))         
        (at start (carrying-empty-pod ?r ?p))
        (at start (pod-empty ?p))
        (over all (robot-at ?r ?l))
        (at start (fragile ?a))
        
        (at start (is-type ?a technological))
        (at start (can-pickup ?r technological))
        (at start (can-carry-two ?r)) ;; Forza il Tecnico positivamente
    )
    :effect (and 
        (at start (not (artifact-at ?a ?l)))
        (at start (not (carrying-empty-pod ?r ?p)))
        (at start (not (pod-empty ?p)))
        (at end (carrying-full-pod ?r ?p))  
        (at end (pod-contains ?p ?a))
    )
  )

  ;; 3. PUT IN POD - TOP-SECRET
  (:durative-action put-in-pod-TOP-SECRET
    :parameters (?a - artifact ?l - location ?r - robot ?p - pod)
    :duration (= ?duration 4)
    :condition (and 
        (at start (artifact-at ?a ?l))         
        (at start (carrying-empty-pod ?r ?p))
        (at start (pod-empty ?p))
        (over all (robot-at ?r ?l))
        (at start (fragile ?a))
        
        (at start (is-type ?a top-secret))
        (at start (can-pickup ?r top-secret))
    )
    :effect (and 
        (at start (not (artifact-at ?a ?l)))
        (at start (not (carrying-empty-pod ?r ?p)))
        (at start (not (pod-empty ?p)))
        (at end (carrying-full-pod ?r ?p))  
        (at end (pod-contains ?p ?a))
    )
  )
  
  ;; GENERIC PICK UP FULL POD
  ;; We can leave this generic or split it. 
  ;; Since the artifact is already in the pod, we rely on the fact that 
  ;; usually the same robot who put it there will carry it, or any capable robot.
  ;; Leaving it generic for flexibility in transport.
  (:durative-action pick-up-full-pod
      :parameters (?r - robot ?l - location ?p - pod ?a - artifact ?at - artifact-type)
      :duration (= ?duration 4)
      :condition (and 
          (at start (contains-full-pod ?l ?p))
          (at start (hands-empty ?r))
          (over all (robot-at ?r ?l))
          (over all (pod-contains ?p ?a))
          
          ;; Just check capability, no need to force strict role here 
          ;; (allows handovers if necessary, though unlikely)
          (at start (can-pickup ?r ?at))
          (at start (is-type ?a ?at))
      )
      :effect (and 
          (at start (not (hands-empty ?r)))
          (at start (not (contains-full-pod ?l ?p)))
          (at end (carrying-full-pod ?r ?p))
      )
  )

  ;; ========================
  ;; COOLING ACTIONS
  ;; ========================

  (:durative-action cool-artifact-while-carrying
    :parameters (?r - robot ?a - artifact ?l - location)
    :duration (= ?duration 12)
    :condition (and 
        (at start (warm ?a))                       
        (at start  (is-chill-room ?l))             
        (over all (robot-at ?r ?l))                
        (over all (carrying ?r ?a))              
    )
    :effect (and 
        (at start (not (warm ?a)))                 
        (at end (cold ?a))                          
    )
  )

  (:durative-action cool-artifact-while-carrying-in-pod
     :parameters (?r - robot ?a - artifact ?l - location ?p - pod)
     :duration (= ?duration 14)
     :condition (and 
         (at start (warm ?a))                       
         (at start (is-chill-room ?l))              
         (over all (carrying-full-pod ?r ?p))       
         (over all (robot-at ?r ?l))                 
     )
     :effect (and 
         (at start (not (warm ?a)))                  
         (at end (cold ?a))                          
     )
  )


  ;; ========================
  ;; SECOND SLOT (TECHNICIAN)
  ;; ========================
  
  (:durative-action pick-up-second-object
      :parameters (
            ?r - robot ?a - artifact ?at - artifact-type ?l - location
      )
      :duration (= ?duration 2)
      :condition (and 
            (at start (artifact-at ?a ?l))
            (at start (second-slot-empty ?r))           
            (at start (can-carry-two ?r))               
            (at start (can-pickup ?r ?at))
            (at start (is-type  ?a ?at))
            (at start (no-fragile  ?a))
            (over all (robot-at ?r ?l))
            
            ;; STRICT CONSTRAINT: Only Technological objects for the second slot
            ;; This prevents the Technician from greedily grabbing other stuff
            (at start (is-type ?a technological))
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