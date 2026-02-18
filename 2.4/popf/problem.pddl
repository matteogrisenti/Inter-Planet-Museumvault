(define (problem museumproblem)
  (:domain museum)

  (:objects
    ;; --- Robots ---
    curator - robot
    
    ;; --- Locations ---
    entrance maintenance_tunnel - location
  )

  (:init
    ;; ============================================================
    ;; ROBOT CAPABILITIES & INITIAL STATE
    ;; ============================================================
    
    ;; Curator
    (robot_at curator entrance)
    (can_access curator entrance) (can_access curator maintenance_tunnel) 
    (sealing_mode_off curator)
    
    ;; ============================================================
    ;; WORLD TOPOLOGY & PROPERTIES
    ;; ============================================================
    
    ;; Connections
    (connected entrance maintenance_tunnel) (connected maintenance_tunnel entrance)
  
    
    ;; Pressure & Safety
    (is_unpressurized maintenance_tunnel)
    (is_pressurized entrance)

    ;; Static safe rooms (Hall B is handled via TILs above)
    (is_safe entrance) (is_safe maintenance_tunnel)
    

  )

  ;; ============================================================
  ;; GOAL STATE
  ;; ============================================================
  (:goal (and
      (robot_at curator maintenance_tunnel)
    )   
  )
)
