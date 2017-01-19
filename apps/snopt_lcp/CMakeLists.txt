###############################################
# apps/snopt_interface
file(GLOB snoptlcp_srcs "*.cpp")
file(GLOB snoptlcp_hdrs "*.hpp")

set(SNOPTF2C ${CMAKE_SOURCE_DIR}/snoptf2c)
set(SNOPTLCP ${CMAKE_SOURCE_DIR}/apps/QPCC)
# include_directories(${SNOPTF2C})
link_directories(${SNOPTF2C} ${SNOPTLCP})

add_executable(snoptlcp ${snoptlcp_srcs} ${snoptlcp_hdrs})
target_link_libraries(snoptlcp dart SNOPTLCP snopt_c snprint_c blas_c f2c)
set_target_properties(snoptlcp PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
