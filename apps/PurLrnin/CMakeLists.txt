###############################################
# apps/PurLrnin
file(GLOB PurLrnin_srcs "*.cpp")
file(GLOB PurLrnin_hdrs "*.h")
add_executable(PurLrnin ${PurLrnin_srcs} ${PurLrnin_hdrs})
target_link_libraries(PurLrnin dart)
set_target_properties(PurLrnin PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
