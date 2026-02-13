cmake_minimum_required(VERSION 3.19)
project(bt_common_nodes LANGUAGES CXX)

# Find packages
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets)
find_package(rclcpp REQUIRED)

find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(visualization_msgs REQUIRED)

# grinding specific
find_package(capsen_planning QUIET)
find_package(capsen_vision QUIET)
find_package(plg_msgs QUIET)

# pushcorp specific
find_package(pushcorp_msgs QUIET)

# motoman specific
find_package(motoros2_interfaces QUIET)

# point clouds
find_package(pcl_ros QUIET)
find_package(pcl_conversions QUIET)

find_package(MPI REQUIRED)
if(NOT TARGET MPI::MPI_C)
  add_library(MPI::MPI_C INTERFACE IMPORTED)
  set_target_properties(MPI::MPI_C PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${MPI_C_INCLUDE_PATH}"
    INTERFACE_LINK_LIBRARIES "${MPI_C_LIBRARIES}"
  )
endif()

# Manually pick source files based on available packages
set(SRCS)

# Base files - always included
list(APPEND SRCS
    src/action_nodes.cpp
    src/condition_nodes.cpp
    src/bt_ros_nodes.cpp
)

# Common dependencies for all targets
set(COMMON_DEPS
    ament_index_cpp
    behaviortree_cpp
    behaviortree_ros2
    rclcpp
    std_msgs
    std_srvs
    control_msgs
    yaml-cpp
    visualization_msgs
)

# Conditionally add files based on optional packages
if(capsen_planning_FOUND AND capsen_vision_FOUND AND plg_msgs_FOUND)
    list(APPEND SRCS
        src/pl_grinding_nodes.cpp
    )
    list(APPEND COMMON_DEPS capsen_planning capsen_vision plg_msgs)
    message(STATUS "Including Capsen grinding and vision nodes")
endif()

if(motoros2_interfaces_FOUND)
    list(APPEND COMMON_DEPS motoros2_interfaces)
    message(STATUS "Including Motoman interface dependencies")
endif()

if(pushcorp_msgs_FOUND)
    list(APPEND SRCS
        src/pushcorp_nodes.cpp
    )
    list(APPEND COMMON_DEPS pushcorp_msgs)
    message(STATUS "Including PushCorp nodes")
endif()


if(pcl_ros_FOUND AND pcl_conversions_FOUND)
    list(APPEND COMMON_DEPS pcl_ros pcl_conversions)
    message(STATUS "Including Point Cloud nodes")
endif()



# Create a shared library (optional - only if you have .cpp files)
if(SRCS)
    add_library(${PROJECT_NAME} SHARED ${SRCS})
    
    target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    )
    
    # Set compile definitions based on optional packages found
    if(capsen_planning_FOUND AND capsen_vision_FOUND AND plg_msgs_FOUND)
        target_compile_definitions(${PROJECT_NAME} PUBLIC HAS_CAPSEN_SUPPORT)
        message(STATUS "Capsen support enabled in bt_common_nodes")
    endif()
    
    if(pushcorp_msgs_FOUND)
        target_compile_definitions(${PROJECT_NAME} PUBLIC HAS_PUSHCORP_SUPPORT)
        message(STATUS "PushCorp support enabled in bt_common_nodes")
    endif()
    
    if(motoros2_interfaces_FOUND)
        target_compile_definitions(${PROJECT_NAME} PUBLIC HAS_MOTOMAN_SUPPORT)
        message(STATUS "Motoman support enabled in bt_common_nodes")
    endif()
    
    if(pcl_ros_FOUND AND pcl_conversions_FOUND)
        target_compile_definitions(${PROJECT_NAME} PUBLIC HAS_PCL_SUPPORT)
        message(STATUS "PCL support enabled in bt_common_nodes")
    endif()
    ament_target_dependencies(${PROJECT_NAME} ${COMMON_DEPS})
    
    
# Link Qt5 libraries and other dependencies
target_link_libraries(${PROJECT_NAME}
    Qt5::Core
    Qt5::Gui
    Qt5::Widgets
    ${PCL_LIBRARIES}
    yaml-cpp
)   
 # Installation for library
install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}_targets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)
endif()

# Install header files
install(DIRECTORY include/
    DESTINATION include
    FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME}
    OPTIONAL  # Only install if directories exist
)

# Package export
if(SRCS)
    ament_export_targets(${PROJECT_NAME}_targets HAS_LIBRARY_TARGET)
endif()
ament_export_include_directories(include)
ament_export_dependencies(${COMMON_DEPS} Qt5)

ament_package()