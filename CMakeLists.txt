


cmake_minimum_required(VERSION 3.10)

project(convex_hull_subdivision_ply_demo)
find_package(CGAL REQUIRED)
find_package(PCL 1.8 REQUIRED)

include(${CGAL_USE_FILE})
include_directories(${PCL_INCLUDE_DIRS})


link_directories(${PCL_LIBRARY_DIRS})
link_directories(${CGAL_LIBRARY_DIRS})

add_executable (convex_hull_subdivision_ply_demo convex_hull_subdivision_ply_demo.cpp)
target_link_libraries (convex_hull_subdivision_ply_demo ${PCL_LIBRARIES}  ${CGAL_LIBS})

