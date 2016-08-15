aux_source_directory( . DIR_LIBMPC_SRCS )
link_libraries(dart DDP)
add_library( MPC ${DIR_LIBMPC_SRCS})

