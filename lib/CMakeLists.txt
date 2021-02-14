cmake_minimum_required (VERSION 3.1 FATAL_ERROR)
project(point_transformation_lib)

file (GLOB SOURCES src/*.cpp)

add_library(point_transformation_lib SHARED
  ${SOURCES}
)

include_directories(point_transformation_lib
  include
)

target_include_directories(point_transformation_lib 
  PUBLIC include)

# OpenCV
find_package(OpenCV REQUIRED)

target_link_libraries(point_transformation_lib PUBLIC
  ${OpenCV_LIBS}
)

# Install library
install(DIRECTORY include/
    DESTINATION include)

install(TARGETS point_transformation_lib
  EXPORT export_point_transformation_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)