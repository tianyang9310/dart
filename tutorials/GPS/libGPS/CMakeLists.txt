file(GLOB DIR_LIBGPS_CPP "*.cpp")
file(GLOB DIR_LIBGPS_HPP "*.h")
list( APPEND DIR_LIBGPS_SRCS ${DIR_LIBGPS_CPP} ${DIR_LIBGPS_HPP})
#aux_source_directory( . DIR_LIBGPS_SRCS )

link_libraries( DDP dart )

add_library( GPS ${DIR_LIBGPS_SRCS})
