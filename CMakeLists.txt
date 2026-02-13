cmake_minimum_required(VERSION 3.3)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
project(slam)
set(ENTRY_FILE "main.cpp")
add_compile_options("-O3")

#file(MAKE_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/bin")
set(INCLUDE_DIRS "./include")
set(LIB_DIRS "./lib")

add_executable(slam ${PROJECT_SOURCE_DIR}/src/${ENTRY_FILE}
${PROJECT_SOURCE_DIR}/src/vehicle_dynamics.cpp
)

if(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64" OR CMAKE_SYSTEM_PROCESSOR STREQUAL "arm64")

    #opencv
    message("\nbuilding for ARM (aarch64/arm64)")
    find_package(OpenCV REQUIRED)
    find_package(Eigen3 REQUIRED)
    target_include_directories(slam PRIVATE ${OpenCV_INCLUDE_DIRS})
    target_include_directories(slam PRIVATE ${EIGEN3_INCLUDE_DIRS})
    target_include_directories(slam PRIVATE ${INCLUDE_DIRS})
    target_include_directories(slam PRIVATE ${LIB_DIRS})
    target_link_libraries(slam PRIVATE ${OpenCV_LIBS} pthread)
    target_link_libraries(slam PRIVATE Eigen3::Eigen)
endif()

if(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "x86_64")
    message("\nbuilding for x86_64")
    #RealSense2
    #windows:
    #find_package(realsense2 CONFIG REQUIRED)
    #target_link_libraries(main PRIVATE realsense2::fw realsense2::rsutils realsense2::realsense2 realsense2::realsense-file)
    # opencv
    find_package( OpenCV REQUIRED )
    find_package(Eigen3 REQUIRED)
    target_include_directories(slam PRIVATE ${OpenCV_INCLUDE_DIRS})
    target_include_directories(slam PRIVATE ${EIGEN3_INCLUDE_DIRS})
    target_include_directories(slam PRIVATE ${INCLUDE_DIRS})
    target_include_directories(slam PRIVATE ${LIB_DIRS})
    # target_link_libraries( slam ${OpenCV_LIBS} pthread)
    target_link_libraries(slam ${OpenCV_LIBS} pthread Eigen3::Eigen)
endif()

#OpenGL
#find_package(OpenGL REQUIRED)

#GLUT
#find_package(GLUT REQUIRED)



# Link stage
#target_link_libraries(slam ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES} realsense2)
# target_link_libraries(slam PUBLIC opengl)
# target_include_directories(slam PUBLIC include)
