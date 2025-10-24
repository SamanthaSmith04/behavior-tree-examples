cmake_minimum_required(VERSION 3.19)
project(bt_application LANGUAGES CXX)

# Find packages
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets)
find_package(rclcpp REQUIRED)
find_package(bt_common_nodes)

find_package(MPI REQUIRED)
if(NOT TARGET MPI::MPI_C)
  add_library(MPI::MPI_C INTERFACE IMPORTED)
  set_target_properties(MPI::MPI_C PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${MPI_C_INCLUDE_PATH}"
    INTERFACE_LINK_LIBRARIES "${MPI_C_LIBRARIES}"
  )
endif()
# Qt configuration
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Source files
set(UIS
    src/example_gui.ui
)

qt5_wrap_ui(UI_HEADERS ${UIS})

set(SRCS ${UI_HEADERS}
    src/example_widget.cpp
    include/example_widget.hpp
)

# Common dependencies for all targets
set(COMMON_DEPS
    ament_index_cpp
    behaviortree_cpp
    behaviortree_ros2
    rclcpp
    bt_common_nodes
)

# Shared library
add_library(example_qt_gui_lib SHARED ${SRCS} ${UIS})

target_include_directories(example_qt_gui_lib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${PCL_INCLUDE_DIRS}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
)

ament_target_dependencies(example_qt_gui_lib ${COMMON_DEPS})

target_link_libraries(example_qt_gui_lib
    Qt5::Widgets
)

# Main executable
add_executable(example_qt_gui src/main.cpp)
target_link_libraries(example_qt_gui example_qt_gui_lib)

# Installation
install(TARGETS 
    example_qt_gui
    example_qt_gui_lib
    EXPORT ${PROJECT_NAME}_targets
    RUNTIME DESTINATION lib/${PROJECT_NAME}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

# Package export
ament_export_targets(${PROJECT_NAME}_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(${COMMON_DEPS} Qt5)

ament_package()
