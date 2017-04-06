###############################################
# glog library
find_package (Glog REQUIRED)
include_directories (${GLOG_INCLUDE_DIR})
# apps/lemkeFix
file(GLOB lemkeFix_srcs "*.cpp")
file(GLOB lemkeFix_hdrs "*.h")
add_executable(lemkeFix ${lemkeFix_srcs} ${lemkeFix_hdrs})
target_link_libraries(lemkeFix dart glog)
set_target_properties(lemkeFix PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
