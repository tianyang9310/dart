###############################################
# apps/snopt_interface
file(GLOB snoptinterface_srcs "*.cpp")
file(GLOB snoptinterface_hdrs "*.hpp")

add_executable(snopt75interface ${snoptinterface_srcs} ${snoptinterface_hdrs})
target_link_libraries(snopt75interface dart snopt7_cpp snopt7)
set_target_properties(snopt75interface PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
