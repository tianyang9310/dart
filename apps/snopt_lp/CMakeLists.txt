###############################################
# apps/snopt__lcp
file(GLOB snopt_lcp_srcs "*.cpp")
file(GLOB snopt_lcp_hdrs "*.hpp")
set(SNOPT ${CMAKE_SOURCE_DIR}/snopt)
include_directories(${SNOPT}/include)
link_directories(${SNOPT}/lib)
add_executable(snopt_lcp ${snopt_lcp_srcs} ${snopt_lcp_hdrs})
target_link_libraries(snopt_lcp dart snopt7_cpp snopt7)
set_target_properties(snopt_lcp PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
