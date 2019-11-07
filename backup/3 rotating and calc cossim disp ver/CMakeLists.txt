cmake_minimum_required(VERSION 2.8)
project(ScanContext)
set (CMAKE_CXX_STANDARD 14)


find_package(nanoflann)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

set(INCLUDE_DIR include)
set(SRC_DIR src)
set(SRC
    main.cpp
    ${INCLUDE_DIR}/utils.h ${SRC_DIR}/utils.cpp
    ${INCLUDE_DIR}/scancontext.h ${SRC_DIR}/scancontext.cpp
    ${INCLUDE_DIR}/pointcloud.h 
)
message(${SRC})

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/${INCLUDE_DIR})

# Executable file 
add_executable(main ${SRC})
target_link_libraries (main Eigen3::Eigen)
