cmake_minimum_required(VERSION 3.5)
project(dragonfly)

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install Python scripts as executables
install(PROGRAMS
  dragonfly/announce.py
  dragonfly/command.py
  dragonfly/virtualco2publisher.py
  dragonfly/logger.py
  dragonfly/co2publisher.py
  dragonfly/pump.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install package.xml
install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

