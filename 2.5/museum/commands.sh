ros2 run plansys2_terminal plansys2_terminal <<EOF

set instance curator robot
set instance technician robot
set instance scientist robot

set instance pod1 pod
set instance pod2 pod

set instance entrance location
set instance maintenance_tunnel location
set instance hall_a location
set instance hall_b location
set instance cryo_chamber location
set instance anti_vibration_pods_room location
set instance stasis_lab location

set instance technological artifact_type
set instance scientific artifact_type
set instance top_secret artifact_type

set instance mart_nord_core_drill artifact
set instance mart_north_pole_ice_sample artifact
set instance mart_mysterious_egg artifact
set instance space_suit artifact
set instance quantum_chip artifact
set instance mart_laser_gun artifact

set predicate (robot_at curator entrance)
set predicate (hands_empty_slot_1 curator)
set predicate (can_access curator entrance)
set predicate (can_access curator maintenance_tunnel)
set predicate (can_access curator hall_a)
set predicate (can_access curator hall_b)
set predicate (can_access curator cryo_chamber)
set predicate (can_access curator anti_vibration_pods_room)
set predicate (can_pickup curator scientific)
set predicate (can_pickup curator top_secret)
set predicate (sealing_mode_off curator)

set predicate (sealing_mode_off technician)
set predicate (robot_at technician entrance)
set predicate (hands_empty_slot_1 technician)
set predicate (hands_empty_slot_2 technician)
set predicate (can_carry_two technician)
set predicate (can_access technician entrance)
set predicate (can_access technician maintenance_tunnel)
set predicate (can_access technician hall_a)
set predicate (can_access technician hall_b)
set predicate (can_access technician cryo_chamber)
set predicate (can_access technician anti_vibration_pods_room)
set predicate (can_pickup technician technological)

set predicate (sealing_mode_off scientist)
set predicate (robot_at scientist stasis_lab)
set predicate (hands_empty_slot_1 scientist)
set predicate (can_access scientist stasis_lab)
set predicate (can_access scientist maintenance_tunnel)
set predicate (can_pickup scientist scientific)
set predicate (can_pickup scientist top_secret)
set predicate (can_pickup scientist technological)

set predicate (connected entrance maintenance_tunnel)
set predicate (connected maintenance_tunnel entrance)
set predicate (connected maintenance_tunnel hall_a)
set predicate (connected hall_a maintenance_tunnel)
set predicate (connected maintenance_tunnel hall_b)
set predicate (connected hall_b maintenance_tunnel)
set predicate (connected maintenance_tunnel cryo_chamber)
set predicate (connected cryo_chamber maintenance_tunnel)
set predicate (connected maintenance_tunnel anti_vibration_pods_room)
set predicate (connected anti_vibration_pods_room maintenance_tunnel)
set predicate (connected maintenance_tunnel stasis_lab)
set predicate (connected stasis_lab maintenance_tunnel)

set predicate (is_unpressurized maintenance_tunnel)
set predicate (is_pressurized entrance)
set predicate (is_pressurized hall_a)
set predicate (is_pressurized hall_b)
set predicate (is_pressurized cryo_chamber)
set predicate (is_pressurized anti_vibration_pods_room)
set predicate (is_pressurized stasis_lab)

set predicate (is_safe entrance)
set predicate (is_safe hall_a)
set predicate (is_safe hall_b)
set predicate (is_safe cryo_chamber)
set predicate (is_safe anti_vibration_pods_room)
set predicate (is_safe maintenance_tunnel)
set predicate (is_safe stasis_lab)

set predicate (is_chill_room cryo_chamber)

set predicate (pod_empty pod1)
set predicate (pod_empty pod2)
set predicate (pod_at pod1 anti_vibration_pods_room)
set predicate (pod_at pod2 anti_vibration_pods_room)

set predicate (artifact_at mart_nord_core_drill hall_a)
set predicate (is_type mart_nord_core_drill scientific)
set predicate (warm mart_nord_core_drill)
set predicate (no_fragile mart_nord_core_drill)

set predicate (artifact_at mart_north_pole_ice_sample hall_a)
set predicate (is_type mart_north_pole_ice_sample scientific)
set predicate (warm mart_north_pole_ice_sample)
set predicate (no_fragile mart_north_pole_ice_sample)

set predicate (artifact_at mart_mysterious_egg hall_a)
set predicate (is_type mart_mysterious_egg top_secret)
set predicate (warm mart_mysterious_egg)
set predicate (no_fragile mart_mysterious_egg)

set predicate (artifact_at space_suit hall_b)
set predicate (is_type space_suit technological)
set predicate (warm space_suit)
set predicate (no_fragile space_suit)

set predicate (artifact_at quantum_chip hall_b)
set predicate (is_type quantum_chip technological)
set predicate (warm quantum_chip)

set predicate (artifact_at mart_laser_gun hall_b)
set predicate (is_type mart_laser_gun top_secret)
set predicate (warm mart_laser_gun)

set goal (and (artifact_at mart_nord_core_drill stasis_lab) (cold mart_nord_core_drill) (artifact_at space_suit stasis_lab) (artifact_at quantum_chip stasis_lab) (cold quantum_chip) (artifact_at mart_north_pole_ice_sample cryo_chamber) (artifact_at mart_mysterious_egg cryo_chamber))

get plan

run

quit
EOF
