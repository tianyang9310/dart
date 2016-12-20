###############################################
# apps/swing
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -Wno-deprecated-declarations")
find_package(Caffe)

message(STATUS "Caffe_DEFINITIONS")
message(STATUS ${Caffe_DEFINITIONS})

message(STATUS "Caffe_INCLUDE_DIRS")
message(STATUS ${Caffe_INCLUDE_DIRS})

message(STATUS "Caffe_LIBRARIES")
message(STATUS ${Caffe_LIBRARIES})

file(GLOB swing_srcs "*.cpp")
file(GLOB swing_hdrs "*.h")

add_definitions(${Caffe_DEFINITIONS})    # ex. -DCPU_ONLY
include_directories(${Caffe_INCLUDE_DIRS})

add_executable(swing ${swing_srcs} ${swing_hdrs})
target_link_libraries(swing dart ${Caffe_LIBRARIES})
set_target_properties(swing PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
