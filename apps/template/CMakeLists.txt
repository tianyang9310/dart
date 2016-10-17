###############################################
# apps/template
file(GLOB template_srcs "*.cpp")
file(GLOB template_hdrs "*.h")
add_executable(template ${template_srcs} ${template_hdrs})
target_link_libraries(template dart)
set_target_properties(template PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
