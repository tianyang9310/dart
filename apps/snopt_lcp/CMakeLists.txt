###############################################
# apps/snopt_interface
file(GLOB snoptlcp_srcs "*.cpp")
file(GLOB snoptlcp_hdrs "*.hpp")
set(SNOPT ${CMAKE_SOURCE_DIR}/snopt)
include_directories(${SNOPT}/include)
link_directories(${SNOPT}/lib)
add_executable(snoptlcp ${snoptlcp_srcs} ${snoptlcp_hdrs})
target_link_libraries(snoptlcp dart snopt7_cpp snopt7)
set_target_properties(snoptlcp PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
