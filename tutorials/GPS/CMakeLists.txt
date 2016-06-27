add_subdirectory( libGPS  )
aux_source_directory( CartPole DIR_AGENT_SRCS )
aux_source_directory( GUI DIR_GUI_SRCS )

find_package(PythonLibs REQUIRED)
# message(${PYTHON_INCLUDE_DIRS})
include_directories(${PYTHON_INCLUDE_DIRS} /usr/local/lib/python2.7/dist-packages/numpy/core/include)

add_executable( main main.cpp ${DIR_AGENT_SRCS} ${DIR_GUI_SRCS})
target_link_libraries( main GPS DDP dart ${PYTHON_LIBRARIES})
