file(GLOB DIR_LIBGPS_CPP "*.cpp")
file(GLOB DIR_LIBGPS_HPP "*.h")
list( APPEND DIR_LIBGPS_SRCS ${DIR_LIBGPS_CPP} ${DIR_LIBGPS_HPP})
#aux_source_directory( . DIR_LIBGPS_SRCS )

find_package(PythonLibs REQUIRED)
# message(${PYTHON_INCLUDE_DIRS})
include_directories(${PYTHON_INCLUDE_DIRS})

link_libraries( DDP dart ${PYTHON_LIBRARIES})

add_library( GPS ${DIR_LIBGPS_SRCS})
