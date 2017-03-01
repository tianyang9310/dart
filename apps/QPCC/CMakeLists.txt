###############################################
# apps/QPCC
#LINK_DIRECTORIES(/usr/local/Cellar/gfortran/4.8.2/gfortran/lib)
file(GLOB QPCC_srcs "*.cpp")
file(GLOB QPCC_hdrs "*.h")

file(GLOB MAIN_CPP "Main.cpp")
set (SNOPTLCP_SRCS "${QPCC_srcs};${QPCC_hdrs}")
list (REMOVE_ITEM SNOPTLCP_SRCS ${MAIN_CPP})
# message("${SNOPTLCP_SRCS}")

add_executable(QPCC ${QPCC_srcs} ${QPCC_hdrs})
target_link_libraries(QPCC dart snopt_c snprint_c blas_c f2c)
set_target_properties(QPCC PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

# Here link_libraries should precede add_library
link_libraries(dart snopt_c snprint_c blas_c f2c)
add_library(SNOPTLCP ${SNOPTLCP_SRCS})

