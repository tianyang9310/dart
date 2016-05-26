###############################################
# tutorial/cartpole

find_package(PythonLibs REQUIRED)
file(GLOB cartpole_srcs "*.cpp")
file(GLOB cartpole_hdrs "*.h")
add_executable(cartpole ${cartpole_srcs} ${cartpole_hdrs})
include_directories(${PYTHON_INCLUDE_DIRS})
target_link_libraries(cartpole dart ${PYTHON_LIBRARIES})
#set_target_properties(cartpole PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/tutorials/cartpole/..")
