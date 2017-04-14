###############################################
# glog library
find_package (Glog REQUIRED)
include_directories (${GLOG_INCLUDE_DIR})
# apps/lemkeFix
file(GLOB lemkeFix_srcs "*.cpp")
file(GLOB lemkeFix_hdrs "*.h")

file(GLOB MyPGS_srcs "../CntctLrnin/ProjectedGSLCPSolver.cpp")
file(GLOB MyPGS_hdrs "../CntctLrnin/ProjectedGSLCPSolver.h")

add_executable(lemkeFix ${lemkeFix_srcs} ${lemkeFix_hdrs} ${MyPGS_srcs} ${MyPGS_hdrs})
target_link_libraries(lemkeFix dart glog)
set_target_properties(lemkeFix PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
