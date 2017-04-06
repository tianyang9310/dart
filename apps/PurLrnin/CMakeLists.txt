###############################################
# glog library
find_package (Glog REQUIRED)
include_directories (${GLOG_INCLUDE_DIR})
# apps/PurLrnin
file(GLOB PurLrnin_srcs "*.cpp")
file(GLOB PurLrnin_hdrs "*.h")
file(GLOB MyLemke_srcs "../lemkeFix/myLemke.cpp")
file(GLOB MyLemke_hdrs "../lemkeFix/myLemke.h")
add_executable(PurLrnin ${PurLrnin_srcs} ${PurLrnin_hdrs} ${MyLemke_hdrs} ${MyLemke_srcs})
target_link_libraries(PurLrnin dart glog)
set_target_properties(PurLrnin PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
