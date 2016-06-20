aux_source_directory( . DIR_LIBGPS_SRCS )

link_libraries( DDP dart )

add_library( GPS ${DIR_LIBGPS_SRCS})
