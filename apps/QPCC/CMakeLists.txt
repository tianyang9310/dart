###############################################
# apps/QPCC
#LINK_DIRECTORIES(/usr/local/Cellar/gfortran/4.8.2/gfortran/lib)
 file(GLOB QPCC_srcs "*.cpp")
file(GLOB QPCC_hdrs "*.h")
add_executable(QPCC ${QPCC_srcs} ${QPCC_hdrs})
target_link_libraries(QPCC dart libsnopt7.a libsqopt7.a libgfortran.a libquadmath.a)
set_target_properties(QPCC PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
