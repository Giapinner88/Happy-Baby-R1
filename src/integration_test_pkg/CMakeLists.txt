cmake_minimum_required(VERSION 3.8)
project(integration_test_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto QUIET)
  if(ament_lint_auto_FOUND)
    # Keep lint optional so Foxy builds remain reproducible even when
    # workspace test packages are not installed from the ROS build farm.
    set(ament_cmake_copyright_FOUND TRUE)
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
  endif()
endif()

add_executable(dummy_node src/main.cpp)
install(TARGETS dummy_node DESTINATION lib/${PROJECT_NAME})

ament_package()
