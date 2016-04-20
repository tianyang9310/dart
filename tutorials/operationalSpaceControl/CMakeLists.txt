###############################################
# apps/operationalSpaceControl
file(GLOB tutorial_operationalSpaceControl_srcs "*.cpp")
file(GLOB tutorial_operationalSpaceControl_hdrs "*.h")

add_executable(tutorial_operationalSpaceControl ${tutorial_operationalSpaceControl_srcs} ${tutorial_operationalSpaceControl_hdrs})
target_link_libraries(tutorial_operationalSpaceControl dart)
set_target_properties(tutorial_operationalSpaceControl PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/tutorials")
