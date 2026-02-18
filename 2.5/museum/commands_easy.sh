ros2 run plansys2_terminal plansys2_terminal <<EOF

set instance curator robot

set instance entrance location
set instance maintenance_tunnel location

set predicate (robot_at curator entrance)
set predicate (can_access curator entrance)
set predicate (can_access curator maintenance_tunnel)
set predicate (sealing_mode_off curator)

set predicate (connected maintenance_tunnel entrance)
set predicate (connected entrance maintenance_tunnel)

set predicate (is_unpressurized maintenance_tunnel)
set predicate (is_pressurized entrance)

set predicate (is_safe entrance)
set predicate (is_safe maintenance_tunnel)

set goal (and (robot_at curator maintenance_tunnel))

get plan

run

quit

EOF