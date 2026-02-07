cmake_minimum_required(VERSION 3.3)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
project(slam)
set(ENTRY_FILE "slam_main.cpp")
add_compile_options("-O3")

#file(MAKE_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/bin")
set(INCLUDE_DIRS "./include")

add_executable(slam ${PROJECT_SOURCE_DIR}/src/${ENTRY_FILE} 
  ${PROJECT_SOURCE_DIR}/src/PoseEstimator.cpp
  ${PROJECT_SOURCE_DIR}/src/Frames.cpp)

#depends on where Pangolin is installed
#add_subdirectory(deps/Pangolin)
set(Pangolin_DIR "${CMAKE_SOURCE_DIR}/deps/Pangolin/build")
find_package(Pangolin REQUIRED CONFIG)


if(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "aarch64")
    message("\nbuilding for aarch64")
    #opencv
    find_package( OpenCV 4.0.0 REQUIRED )
    find_package(Eigen3 REQUIRED NO_MODULE)
    target_include_directories(slam PRIVATE ${OpenCV_INCLUDE_DIRS})
    target_include_directories(slam PRIVATE ${INCLUDE_DIRS})
    target_link_libraries(slam ${OpenCV_LIBS} pthread Eigen3::Eigen ${Pangolin_LIBRARIES})
endif()

if(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "x86_64")
    message("\nbuilding for x86_64")
    #RealSense2
    #windows:
    #find_package(realsense2 CONFIG REQUIRED)
    #target_link_libraries(main PRIVATE realsense2::fw realsense2::rsutils realsense2::realsense2 realsense2::realsense-file)
    # opencv
    find_package( OpenCV REQUIRED )
    find_package(Eigen3 REQUIRED NO_MODULE)
    target_include_directories(slam PRIVATE ${OpenCV_INCLUDE_DIRS})
    target_include_directories(slam PRIVATE ${INCLUDE_DIRS})
    target_link_libraries(slam ${OpenCV_LIBS} pthread Eigen3::Eigen ${Pangolin_LIBRARIES})
endif()

#OpenGL
#find_package(OpenGL REQUIRED)

#GLUT
#find_package(GLUT REQUIRED)



# Link stage
#target_link_libraries(slam ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES} realsense2)
# target_link_libraries(slam PUBLIC opengl)
# target_include_directories(slam PUBLIC include)
