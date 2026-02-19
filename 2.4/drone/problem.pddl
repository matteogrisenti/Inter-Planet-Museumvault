(define (problem museumproblem)
  (:domain temporal-multi-robot)

  (:objects
    ;; --- Robots ---
    curator technician scientist drone - robot

    ;; --- Pods ---
    pod1 pod2 - pod
    
    ;; --- Locations ---
    entrance maintenance_tunnel - location
    hall_a hall_b cryo_chamber - location
    anti_vibration_pods_room stasis_lab - location

    ;; --- Typologies ---
    technological scientific top_secret - artifact_type
      
    ;; --- Artifacts ---
    mart_nord_core_drill mart_sud_core_drill - artifact
    mart_east_core_drill mart_west_core_drill - artifact
    mart_sand_sample mart_north_pole_ice_sample - artifact
    mart_mysterious_egg mart_laser_gun mart_pink_hat - artifact
    rover_wheel space_suit quantum_chip rusty_lightsaber - artifact
    asteroid_MG04TN_ice_sample asteroid_AD29TV_rock_sample - artifact
    venus_sand_sample venus_rock_sample - artifact
  )

  (:init
    ;; ============================================================
    ;; ROBOT CAPABILITIES & INITIAL STATE
    ;; ============================================================
    
    ;; Curator
    (robot_at curator entrance) (hands_empty_slot_1 curator)
    (can_access curator entrance) (can_access curator maintenance_tunnel) 
    (can_access curator hall_a) ; no hall b
    (can_access curator cryo_chamber) (can_access curator anti_vibration_pods_room)
    (can_pickup curator scientific) (can_pickup curator top_secret)
    (sealing_mode_off curator)

    ;; Technician
    (sealing_mode_off technician)
    (robot_at technician entrance) (hands_empty_slot_1 technician) (can_carry_two technician) (hands_empty_slot_2 technician)
    
    (can_access technician entrance) (can_access technician maintenance_tunnel) 
    (can_access technician hall_a) ; no hall b
    (can_access technician cryo_chamber) (can_access technician anti_vibration_pods_room)
    (can_pickup technician technological)

    ;; Scientist
    (sealing_mode_off scientist)
    (robot_at scientist stasis_lab) (hands_empty_slot_1 scientist)
    (can_access scientist stasis_lab) (can_access scientist maintenance_tunnel)
    (can_pickup scientist scientific) (can_pickup scientist top_secret) (can_pickup scientist technological)

    ;; Drone
    (robot_at drone entrance) (hands_empty_slot_1 drone) (sealing_mode_off drone)
    (can_access drone entrance) (can_access drone maintenance_tunnel) (can_access drone hall_b) (can_access drone anti_vibration_pods_room)
    (can_pickup drone scientific) (can_pickup drone top_secret) (can_pickup drone technological)
    (can_fly drone)
    
    ;; ============================================================
    ;; WORLD TOPOLOGY & PROPERTIES
    ;; ============================================================
    
    ;; Connections
    (connected entrance maintenance_tunnel) (connected maintenance_tunnel entrance)
    (connected maintenance_tunnel hall_a) (connected hall_a maintenance_tunnel)
    (connected maintenance_tunnel hall_b) (connected hall_b maintenance_tunnel)
    (connected maintenance_tunnel cryo_chamber) (connected cryo_chamber maintenance_tunnel)
    (connected maintenance_tunnel anti_vibration_pods_room) (connected anti_vibration_pods_room maintenance_tunnel)
    (connected maintenance_tunnel stasis_lab) (connected stasis_lab maintenance_tunnel)

    ;; Pressure & Safety
    (is_unpressurized maintenance_tunnel)
    (is_pressurized entrance) (is_pressurized hall_a) (is_pressurized hall_b) 
    (is_pressurized cryo_chamber) (is_pressurized anti_vibration_pods_room) (is_pressurized stasis_lab)

    ;; Static safe rooms (Hall B is handled via TILs above)
    (is_safe entrance) (is_safe hall_a) (is_safe cryo_chamber)
    (is_safe anti_vibration_pods_room) (is_safe maintenance_tunnel) (is_safe stasis_lab)
    
    ;; ============================================================
    ;; SEISMIC ACTIVITY (TIMED INITIAL LITERALS)
    ;; ============================================================

    ;; Special Room Properties
    (is_chill_room cryo_chamber)
    (is_seismic hall_b)

    ;; Pods
    (pod_empty pod1) (pod_empty pod2)
    (pod_at pod1 anti_vibration_pods_room) (pod_at pod2 anti_vibration_pods_room)

    ;; ============================================================
    ;; ARTIFACTS BY ROOM (Position, Type, Features)
    ;; ============================================================

    ;; --- HALL A ---
    ;; Martian Core Drills
    (artifact_at mart_nord_core_drill hall_a) (is_type mart_nord_core_drill scientific) (warm mart_nord_core_drill) (no_fragile mart_nord_core_drill)
    ; (artifact_at mart_sud_core_drill hall_a) (is_type mart_sud_core_drill scientific) (warm mart_sud_core_drill) (no_fragile mart_sud_core_drill)
    ; (artifact_at mart_east_core_drill hall_a) (is_type mart_east_core_drill scientific) (warm mart_east_core_drill) (no_fragile mart_east_core_drill)
    ; (artifact_at mart_west_core_drill hall_a) (is_type mart_west_core_drill scientific) (warm mart_west_core_drill) (no_fragile mart_west_core_drill)
    
    ; ;; Hall A: Samples & Mysterious Egg
    (artifact_at mart_north_pole_ice_sample hall_a) (is_type mart_north_pole_ice_sample scientific) (warm mart_north_pole_ice_sample) (no_fragile mart_north_pole_ice_sample)
    (artifact_at mart_mysterious_egg hall_a) (is_type mart_mysterious_egg top_secret) (warm mart_mysterious_egg) (no_fragile mart_mysterious_egg)
    ; (artifact_at asteroid_MG04TN_ice_sample hall_a) (is_type asteroid_MG04TN_ice_sample scientific) (warm asteroid_MG04TN_ice_sample) (no_fragile asteroid_MG04TN_ice_sample)

    ;; --- HALL B ---
    ;; Mission Gear
    ; (artifact_at rover_wheel hall_b) (is_type rover_wheel technological) (warm rover_wheel) (no_fragile rover_wheel)
    (artifact_at space_suit hall_b) (is_type space_suit technological) (warm space_suit) (no_fragile space_suit)
    (artifact_at quantum_chip hall_b) (is_type quantum_chip technological) (warm quantum_chip)
    ; (artifact_at rusty_lightsaber hall_b) (is_type rusty_lightsaber technological) (warm rusty_lightsaber) (no_fragile rusty_lightsaber)

    ;; Hall B: Samples & Civilization Artifacts
    ; (artifact_at mart_sand_sample hall_b) (is_type mart_sand_sample scientific) (warm mart_sand_sample)
    (artifact_at mart_laser_gun hall_b) (is_type mart_laser_gun top_secret) (warm mart_laser_gun)
    ; (artifact_at mart_pink_hat hall_b) (is_type mart_pink_hat top_secret) (warm mart_pink_hat)
    ; (artifact_at asteroid_AD29TV_rock_sample hall_b) (is_type asteroid_AD29TV_rock_sample scientific) (warm asteroid_AD29TV_rock_sample)
    ; (artifact_at venus_sand_sample hall_b) (is_type venus_sand_sample scientific) (warm venus_sand_sample) (no_fragile venus_sand_sample)
    ; (artifact_at venus_rock_sample hall_b) (is_type venus_rock_sample scientific) (warm venus_rock_sample) (no_fragile venus_rock_sample)
  )

  ;; ============================================================
  ;; GOAL STATE
  ;; ============================================================
  (:goal (and
    ;; Final Locations
    (artifact_at mart_nord_core_drill stasis_lab) (cold mart_nord_core_drill)
    ; (artifact_at mart_sud_core_drill stasis_lab) (cold mart_sud_core_drill)
    ; (artifact_at mart_east_core_drill stasis_lab) (cold mart_east_core_drill)
    ; (artifact_at mart_west_core_drill stasis_lab) (cold mart_west_core_drill)
    ; (artifact_at rover_wheel stasis_lab)
    ; (artifact_at space_suit stasis_lab)
    ; (artifact_at quantum_chip stasis_lab) (cold quantum_chip)
    ; (artifact_at rusty_lightsaber stasis_lab) (cold rusty_lightsaber)

    ; (artifact_at mart_north_pole_ice_sample cryo_chamber)
    (artifact_at mart_mysterious_egg cryo_chamber)    
    ; (artifact_at asteroid_MG04TN_ice_sample cryo_chamber)

    ; (artifact_at mart_sand_sample hall_a)
    (artifact_at mart_laser_gun hall_a)
    ; (artifact_at mart_pink_hat hall_a)  
    ; (artifact_at asteroid_AD29TV_rock_sample hall_a)  
    ; (artifact_at venus_sand_sample hall_a)
    ; (artifact_at venus_rock_sample hall_a)

    (robot_at drone entrance)
    )   
  )
)
