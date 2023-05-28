cmake_minimum_required(VERSION 3.5)
project(dragonfly)

# Find dependencies
find_package(ament_cmake REQUIRED)

# Define a list of Python scripts
set(PYTHON_SCRIPTS
  dragonfly/announce.py
  dragonfly/command.py
  dragonfly/virtualco2publisher.py
  dragonfly/logger.py
  dragonfly/co2publisher.py
  dragonfly/pump.py
)

# Install Python scripts as executables with renamed targets
foreach(script ${PYTHON_SCRIPTS})
  get_filename_component(script_name ${script} NAME_WE)
  install(PROGRAMS ${script}
    DESTINATION lib/${PROJECT_NAME}
    RENAME ${script_name}
  )
endforeach()

# Install package.xml
install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

