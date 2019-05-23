cmake_minimum_required(VERSION 2.8.3)
project(cardsflow_rviz)

SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g")

SET(CMAKE_BUILD_TYPE Debug)

#==============================================================================
# Dependencies
#==============================================================================
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation rviz tf tf_conversions visualization_msgs
		roboy_middleware_msgs common_utilities roboy_simulation_msgs)

# Qt5
# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)
find_package(Qt5 REQUIRED COMPONENTS Widgets Core Gui)

# Eigen
find_package(PkgConfig REQUIRED )
pkg_check_modules(EIGEN3 REQUIRED eigen3)
include_directories(${EIGEN3_INCLUDE_DIRS})

# yaml-cpp
pkg_check_modules( yaml-cpp REQUIRED yaml-cpp)
include_directories(${yaml-cpp_INCLUDE_DIRS})

catkin_package(CATKIN_DEPENDS message_runtime roboy_middleware_msgs common_utilities INCLUDE_DIRS include)

include_directories(
		include/
		include/rviz
		${catkin_INCLUDE_DIRS}
		${Qt5Widgets_INCLUDE_DIRS}
		${Qt5Core_INCLUDE_DIRS}
		${Qt5Gui_INCLUDE_DIRS}
)
#==============================================================================
# Application
#==============================================================================

qt5_wrap_cpp(MOC_FILES include/cardsflow_rviz/cardsflow_rviz.hpp )
add_library( cardsflow_rviz src/cardsflow_rviz.cpp ../common_utilities/src/rviz_visualization.cpp ${MOC_FILES})
target_link_libraries( cardsflow_rviz ${QT_LIBRARIES} ${catkin_LIBRARIES} )

#==============================================================================
# Installation
#==============================================================================
install(TARGETS
		cardsflow_rviz
		LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
		)

install(FILES
		package.xml
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
		)