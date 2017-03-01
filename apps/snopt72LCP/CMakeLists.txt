###############################################
# apps/snopt_interface
file(GLOB snoptlcp_srcs "*.cpp")
file(GLOB snoptlcp_hdrs "*.hpp")

set(SNOPTLCP ${CMAKE_SOURCE_DIR}/apps/QPCC)
link_directories(${SNOPTLCP})

add_executable(snopt72lcp ${snoptlcp_srcs} ${snoptlcp_hdrs})
target_link_libraries(snopt72lcp dart SNOPTLCP snopt_c snprint_c blas_c f2c)
set_target_properties(snopt72lcp PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
