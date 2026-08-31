cmake_minimum_required(VERSION 3.0.2)
project(simnav)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  building_generator_interfaces
  geometry_msgs
  move_base_msgs
  nav_msgs
  rospy
  rosgraph_msgs
  sensor_msgs
  std_msgs
  tf2_ros
  visualization_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/danger_detector_node.py
  scripts/offline_sensor_sim.py
  scripts/lio_localization_bridge_node.py
  scripts/lio_occupancy_node.py
  scripts/lio_health_monitor_node.py
  scripts/localization_bridge_node.py
  scripts/map_views_node.py
  scripts/coverage_explorer_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
