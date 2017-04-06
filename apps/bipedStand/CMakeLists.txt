###############################################
# apps/bipedStand

# Caffe
find_package(Caffe)

add_definitions(${Caffe_DEFINITIONS})
include_directories(${Caffe_INCLUDE_DIRS})

# orignal
file(GLOB bipedStand_srcs "*.cpp")
file(GLOB bipedStand_hdrs "*.h")

add_executable(bipedStand ${bipedStand_srcs} ${bipedStand_hdrs})
target_link_libraries(bipedStand CntctLrninAPI dart ${Caffe_LIBRARIES})
set_target_properties(bipedStand PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
