cmake_minimum_required(VERSION 3.5)
project(museum)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(plansys2_executor REQUIRED)

set(CMAKE_CXX_STANDARD 17)

set(dependencies
    rclcpp
    rclcpp_action
    plansys2_msgs
    plansys2_executor
)

add_executable(fly_into_seismic_room src/fly_into_seismic_room.cpp)
ament_target_dependencies(fly_into_seismic_room ${dependencies})
add_executable(move_to_pressurized_room src/move_to_pressurized_room.cpp)
ament_target_dependencies(move_to_pressurized_room ${dependencies})
add_executable(move_to_unpressurized_room src/move_to_unpressurized_room.cpp)
ament_target_dependencies(move_to_unpressurized_room ${dependencies})
add_executable(activate_seal src/activate_seal.cpp)
ament_target_dependencies(activate_seal ${dependencies})
add_executable(pick_up_empty_pod_slot_1 src/pick_up_empty_pod_slot_1.cpp)
ament_target_dependencies(pick_up_empty_pod_slot_1 ${dependencies})
add_executable(pick_up_full_pod_slot_1 src/pick_up_full_pod_slot_1.cpp)
ament_target_dependencies(pick_up_full_pod_slot_1 ${dependencies})
add_executable(drop_pod_slot_1 src/drop_pod_slot_1.cpp)
ament_target_dependencies(drop_pod_slot_1 ${dependencies})
add_executable(pick_up_slot_1 src/pick_up_slot_1.cpp)
ament_target_dependencies(pick_up_slot_1 ${dependencies})
add_executable(put_in_pod_slot_1 src/put_in_pod_slot_1.cpp)
ament_target_dependencies(put_in_pod_slot_1 ${dependencies})
add_executable(release_artifact_slot_1 src/release_artifact_slot_1.cpp)
ament_target_dependencies(release_artifact_slot_1 ${dependencies})
add_executable(release_artifact_from_pod_slot_1 src/release_artifact_from_pod_slot_1.cpp)
ament_target_dependencies(release_artifact_from_pod_slot_1 ${dependencies})
add_executable(cool_artifact_while_carrying_slot_1 src/cool_artifact_while_carrying_slot_1.cpp)
ament_target_dependencies(cool_artifact_while_carrying_slot_1 ${dependencies})
add_executable(cool_artifact_while_carrying_in_pod_slot_1 src/cool_artifact_while_carrying_in_pod_slot_1.cpp)
ament_target_dependencies(cool_artifact_while_carrying_in_pod_slot_1 ${dependencies})
add_executable(pick_up_slot_2 src/pick_up_slot_2.cpp)
ament_target_dependencies(pick_up_slot_2 ${dependencies})
add_executable(release_artifact_slot_2 src/release_artifact_slot_2.cpp)
ament_target_dependencies(release_artifact_slot_2 ${dependencies})

install(DIRECTORY launch pddl DESTINATION share/${PROJECT_NAME})

install(TARGETS
  fly_into_seismic_room
  move_to_pressurized_room
  move_to_unpressurized_room
  activate_seal
  pick_up_empty_pod_slot_1
  pick_up_full_pod_slot_1
  drop_pod_slot_1
  pick_up_slot_1
  put_in_pod_slot_1
  release_artifact_slot_1
  release_artifact_from_pod_slot_1
  cool_artifact_while_carrying_slot_1
  cool_artifact_while_carrying_in_pod_slot_1
  pick_up_slot_2
  release_artifact_slot_2
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
endif()

ament_export_dependencies(${dependencies})

ament_package()