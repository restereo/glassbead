

cmake_minimum_required(VERSION 2.8)
project( go )
find_package( OpenCV REQUIRED )

INCLUDE_DIRECTORIES(/usr/include/c++)
INCLUDE_DIRECTORIES(${OpenCV_LIBS})

add_executable( go go.cpp )
add_executable( hough hough.cpp )
target_link_libraries( go ${OpenCV_LIBS} )
target_link_libraries( hough ${OpenCV_LIBS} )
