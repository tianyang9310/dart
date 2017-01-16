###############################################
# apps/QPCC
#LINK_DIRECTORIES(/usr/local/Cellar/gfortran/4.8.2/gfortran/lib)
file(GLOB QPCC_srcs "*.cpp")
file(GLOB QPCC_hdrs "*.h")
set(SNOPTF2C ${CMAKE_SOURCE_DIR}/snoptf2c)
# include_directories(${SNOPTF2C})
link_directories(${SNOPTF2C})
add_executable(QPCC ${QPCC_srcs} ${QPCC_hdrs})
target_link_libraries(QPCC dart snopt_c snprint_c blas_c f2c)
set_target_properties(QPCC PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
