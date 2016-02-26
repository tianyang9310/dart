#add_subdirectory( Eigen )
aux_source_directory( . DIR_CONTROLPBP_SRCS )
add_library( ControlPBP ${DIR_CONTROLPBP_SRCS})
