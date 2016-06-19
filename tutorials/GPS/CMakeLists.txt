add_subdirectory( libGPS  )
aux_source_directory( CartPole DIR_AGENT_SRCS )
aux_source_directory( GUI DIR_GUI_SRCS )

add_executable( main main.cpp ${DIR_AGENT_SRCS} ${DIR_GUI_SRCS})
target_link_libraries( main GPS DDP dart)
