###############################################
# apps/CntctLrnin

# glog library
find_package (Glog REQUIRED)
include_directories (${GLOG_INCLUDE_DIR})

# jsoncpp library
file(GLOB_RECURSE dist_json_srcs "dist-json/*.*")
message(STATUS "json src")
message(STATUS ${dist_json_srcs})

# # opencv library
# find_package(OpenCV)
# include_directories(${OpenCV_INCLUDE_DIRS})

# caffe library
find_package(Caffe)
# set(Caffe_DIR /Users/Yang/Material/Research/caffe/build)
message(STATUS "Caffe root path")
message(STATUS ${Caffe_DIR})

message(STATUS "Caffe_DEFINITIONS")
message(STATUS ${Caffe_DEFINITIONS})

message(STATUS "Caffe_INCLUDE_DIRS")
message(STATUS ${Caffe_INCLUDE_DIRS})

message(STATUS "Caffe_LIBRARIES")
message(STATUS ${Caffe_LIBRARIES})

add_definitions(${Caffe_DEFINITIONS})    # ex. -DCPU_ONLY
include_directories(${Caffe_INCLUDE_DIRS})

# original cmake

file(GLOB CntctLrnin_srcs "*.cpp")
file(GLOB CntctLrnin_hdrs "*.h")
file(GLOB MyLemke_srcs "../lemkeFix/myLemke.cpp")
file(GLOB MyLemke_hdrs "../lemkeFix/myLemke.h")

set(SNOPTLCP ${CMAKE_SOURCE_DIR}/apps/QPCC)
link_directories(${SNOPTLCP})

# need to exclude main.cpp
file(GLOB MAIN_CPP "Main.cpp")
list (REMOVE_ITEM CntctLrnin_srcs ${MAIN_CPP})
link_libraries(SNOPTLCP dart snopt_c snprint_c blas_c f2c)
add_library(CntctLrninAPI STATIC ${CntctLrnin_srcs} ${CntctLrnin_hdrs} ${MyLemke_hdrs} ${MyLemke_srcs} ${dist_json_srcs})

add_executable(CntctLrnin ${MAIN_CPP})
target_link_libraries(CntctLrnin CntctLrninAPI SNOPTLCP dart snopt_c snprint_c blas_c f2c ${Caffe_LIBRARIES} glog)
set_target_properties(CntctLrnin PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
