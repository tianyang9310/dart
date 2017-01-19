###############################################
# apps/snopt_interface
file(GLOB snoptinterface_srcs "*.cpp")
file(GLOB snoptinterface_hdrs "*.hpp")
set(SNOPT ${CMAKE_SOURCE_DIR}/snopt)
include_directories(${SNOPT}/include)
link_directories(${SNOPT}/lib)
add_executable(snoptinterface ${snoptinterface_srcs} ${snoptinterface_hdrs})
target_link_libraries(snoptinterface dart snopt7_cpp snopt7)
set_target_properties(snoptinterface PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
