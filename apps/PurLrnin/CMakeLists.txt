###############################################
# apps/PurLrnin
file(GLOB PurLrnin_srcs "*.cpp")
file(GLOB PurLrnin_hdrs "*.h")
file(GLOB MyLemke_srcs "../Lemke_Fix/MyLemke.cpp")
file(GLOB MyLemke_hdrs "../Lemke_Fix/MyLemke.h")
add_executable(PurLrnin ${PurLrnin_srcs} ${PurLrnin_hdrs} ${MyLemke_hdrs} ${MyLemke_srcs})
target_link_libraries(PurLrnin dart)
set_target_properties(PurLrnin PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
