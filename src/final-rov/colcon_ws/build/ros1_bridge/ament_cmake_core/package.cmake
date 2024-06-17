set(_AMENT_PACKAGE_NAME "ros1_bridge")
set(ros1_bridge_VERSION "0.10.3")
set(ros1_bridge_MAINTAINER "Brandon Ong <brandon@openrobotics.org>, Dharini Dutia <dharini@openrobotics.org>, Geoffrey Biggs <geoff@openrobotics.org>")
set(ros1_bridge_BUILD_DEPENDS "builtin_interfaces" "libboost-dev" "pkg-config" "python3-yaml" "rclcpp" "rcpputils" "rcutils" "rmw_implementation_cmake" "std_msgs" "xmlrpcpp")
set(ros1_bridge_BUILDTOOL_DEPENDS "ament_cmake" "ament_index_python" "python3" "python3-catkin-pkg-modules" "rosidl_cmake" "rosidl_parser")
set(ros1_bridge_BUILD_EXPORT_DEPENDS )
set(ros1_bridge_BUILDTOOL_EXPORT_DEPENDS "pkg-config")
set(ros1_bridge_EXEC_DEPENDS "builtin_interfaces" "python3-yaml" "rclcpp" "rcpputils" "rcutils" "std_msgs")
set(ros1_bridge_TEST_DEPENDS "ament_cmake_gtest" "ament_lint_auto" "ament_lint_common" "demo_nodes_cpp" "diagnostic_msgs" "geometry_msgs" "launch" "launch_testing" "launch_testing_ament_cmake" "launch_testing_ros" "ros2run" "sensor_msgs")
set(ros1_bridge_GROUP_DEPENDS "rosidl_interface_packages")
set(ros1_bridge_MEMBER_OF_GROUPS )
set(ros1_bridge_DEPRECATED "")
set(ros1_bridge_EXPORT_TAGS)
list(APPEND ros1_bridge_EXPORT_TAGS "<build_type>ament_cmake</build_type>")