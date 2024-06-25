cmake_minimum_required(VERSION 3.0.2)
project(pwm_driver)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

# Find the wiringPi library
find_library(WIRINGPI_LIB wiringPi)

# Declare a catkin package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pwm_driver
  CATKIN_DEPENDS roscpp std_msgs
)

# Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(library_name ${PROJECT_NAME})

# Add library
add_library(${library_name}
  src/pwm_driver.cpp
  src/PCA9685.cpp
  src/I2CPeripheral.cpp
)

# Link the library with i2c, wiringPi, and catkin libraries
target_link_libraries(${library_name} ${catkin_LIBRARIES} i2c ${WIRINGPI_LIB})

# Add executable
add_executable(pwm_driver_node src/pwm_driver_node.cpp)
target_link_libraries(pwm_driver_node ${library_name} ${catkin_LIBRARIES} i2c ${WIRINGPI_LIB})

# Install targets
install(TARGETS pwm_driver_node ${library_name}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)