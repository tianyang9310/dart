###############################################
# tutorial/cartpole

file(GLOB cartpole_srcs "*.cpp")
file(GLOB cartpole_hdrs "*.h")
add_executable(cartpole ${cartpole_srcs} ${cartpole_hdrs})
target_link_libraries(cartpole dart)
#set_target_properties(cartpole PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/tutorials/cartpole/..")
