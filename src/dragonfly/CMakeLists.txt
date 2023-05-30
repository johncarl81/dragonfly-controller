cmake_minimum_required(VERSION 3.5)
project(dragonfly)

# Find dependencies
find_package(ament_cmake REQUIRED)

# Define the source directory
set(SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/dragonfly)

# Collect all .py files recursively
file(GLOB_RECURSE PYTHON_SCRIPTS "${SOURCE_DIR}/*.py")

# Install Python scripts while maintaining directory structure
foreach(script ${PYTHON_SCRIPTS})
  file(RELATIVE_PATH relative_path ${SOURCE_DIR} ${script})
  get_filename_component(relative_dir ${relative_path} DIRECTORY)
  install(FILES ${script}
    DESTINATION lib/${PROJECT_NAME}/${relative_dir}
  )
endforeach()

# Install package.xml
install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

