cmake_minimum_required(VERSION 3.8)
project(so101_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# urdf and xacro are runtime dependencies of the launch files, not build
# dependencies -- this package only installs directories. They are declared as
# <exec_depend> in package.xml instead, so the build does not need them present.

# Install các thư mục tài nguyên
install(
  DIRECTORY urdf meshes launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()