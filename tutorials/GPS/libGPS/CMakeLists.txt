aux_source_directory( . DIR_LIBGPS_SRCS )

file(GLOB_RECURSE DIR_LIBGPS_AGENT_SRCS "agent/*.*")
#aux_source_directory( ./agent DIR_LIBGPS_AGENT_SRCS )
#list( APPEND DIR_LIBGPS_AGENT_SRCS ./agent/DoubleCartPole)

#message( status ${DIR_LIBGPS_AGENT_SRCS})

file(GLOB_RECURSE DIR_LIBGPS_ALGORITHM_SRCS "algorithm/*.*")
#aux_source_directory( ./algorithm DIR_LIBGPS_ALGORITHM_SRCS )

#message( status ${DIR_LIBGPS_SRCS})



file(GLOB_RECURSE DIR_LIBGPS_UTILITY_SRCS "utility/*.*" )

list( APPEND DIR_LIBGPS_SRCS ${DIR_LIBGPS_AGENT_SRCS} ${DIR_LIBGPS_ALGORITHM_SRCS} ${DIR_LIBGPS_UTILITY_SRCS})

#find_package(EIGEN3 3.0.5 REQUIRED)
#find_package(Boost REQUIRED)
#find_package(GLUT REQUIRED)
#include_directories(${EIGEN3_INCLUDE_DIRS})

#find_package(DART 5.0.1 REQUIRED)

link_directories(dart)  

add_library( GPS ${DIR_LIBGPS_SRCS})
