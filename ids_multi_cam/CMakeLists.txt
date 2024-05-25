cmake_minimum_required(VERSION 3.2 FATAL_ERROR)

project ("ids_multi_cam")

message (STATUS "[${PROJECT_NAME}] Processing ${CMAKE_CURRENT_LIST_FILE}")

set (CMAKE_AUTOMOC TRUE)
set (CMAKE_AUTORCC TRUE)

if (MSVC)
    set (SAMPLE_TARGET_DEFINITION_ADDENDUM WIN32)
endif ()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cv_bridge 
  image_transport
  sensor_msgs
  rosbag
)

# Find packages
if (NOT TARGET ids_peak)
    find_package (ids_peak REQUIRED
        HINTS 
            ../../../../../../../lib/
    )
endif ()

if (NOT TARGET ids_peak_ipl)
    find_package (ids_peak_ipl REQUIRED
        HINTS 
            ../../../../../../../lib/
    )
endif()

find_package (Threads REQUIRED)

find_package(OpenCV REQUIRED )

find_package (Qt5 COMPONENTS Widgets Core Gui REQUIRED)
set(QT_INSTALL_PREFIX "${_qt5Core_install_prefix}")


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS roscpp rospy std_msgs cv_bridge image_transport sensor_msgs rosbag
 DEPENDS OpenCV
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}
)

message("**************************************************************")
message(CMAKE_CURRENT_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR})

# Setup target executable with the same name as our project
add_executable (${PROJECT_NAME}_node
    ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/mainwindow.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/displaywindow.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/acquisitionworker.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/chronometer.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/mainwindow.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/displaywindow.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/acquisitionworker.h    
    ${CMAKE_CURRENT_SOURCE_DIR}/include/chronometer.h
)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


# Set include directories
target_include_directories (${PROJECT_NAME}_node
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}
)

# Link against libraries
target_link_libraries (${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ids_peak
    ids_peak_ipl
    ${CMAKE_THREAD_LIBS_INIT}
    ${OpenCV_LIBS}
    Qt5::Widgets
    Qt5::Core
    Qt5::Gui    
)

# Call deploy functions
# These functions will add a post-build steps to your target in order to copy all needed files (e.g. DLL's) to the output directory.
ids_peak_deploy(${PROJECT_NAME})
ids_peak_ipl_deploy(${PROJECT_NAME})

if ((CMAKE_CXX_COMPILER_ID MATCHES GNU) OR (CMAKE_CXX_COMPILER_ID MATCHES Clang))
    # ignore errors in headers from qt widgets include directories
    foreach (INCLUDE_DIR ${Qt5Widgets_INCLUDE_DIRS})
        if (NOT "${INCLUDE_DIR}" STREQUAL "/usr/include")
            set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -isystem ${INCLUDE_DIR}")
        endif ()
    endforeach ()
endif ()

# Set C++ standard to 14 (required for ids_peak)
set_target_properties(${PROJECT_NAME}_node PROPERTIES
    CXX_STANDARD 14
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS NO
)

# Enable multiprocessing for MSVC
# if (MSVC)
#     target_compile_options (${PROJECT_NAME}
#         PRIVATE "/MP"
#     )
# endif ()

# if (WIN32)
#     # Copy Qt files to the output folder
#     message (STATUS "[${PROJECT_NAME}] Add PostBuildStep for copy of the Qt libs. ${QT_INSTALL_PREFIX}")
#     add_custom_command (TARGET ${PROJECT_NAME} POST_BUILD
#         COMMAND "${QT_INSTALL_PREFIX}/bin/windeployqt"
#                 --no-quick-import
#                 --no-translations
#                 --no-system-d3d-compiler
#                 --no-webkit2
#                 --no-angle
#                 --no-opengl-sw
#                 --no-svg
#                 --dir $<TARGET_FILE_DIR:${PROJECT_NAME}>
#                 $<TARGET_FILE:${PROJECT_NAME}>
#     )
# endif ()

# For unix Build we need the environment variable GENICAM_GENTL32_PATH respectivily GENICAM_GENTL64_PATH to find the GenTL producer libraries.
# To set these environment variables a shell script is used. This script can be automatically generated via ids_peak_generate_starter_script.
# The shell script will be saved at ${CMAKE_CURRENT_BINARY_DIR}/${targetName}.sh and automatically copied to the output directory during post-build.

# To run the samples run this script, not the binary.
# if(UNIX)
#     ids_peak_generate_starter_script(${PROJECT_NAME}_node)
# endif()
