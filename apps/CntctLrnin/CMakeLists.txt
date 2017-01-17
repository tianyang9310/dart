###############################################
# apps/CntctLrnin
file(GLOB CntctLrnin_srcs "*.cpp")
file(GLOB CntctLrnin_hdrs "*.h")
file(GLOB MyLemke_srcs "../Lemke_Fix/MyLemke.cpp")
file(GLOB MyLemke_hdrs "../Lemke_Fix/MyLemke.h")

set(SNOPTF2C ${CMAKE_SOURCE_DIR}/snoptf2c)
set(SNOPTLCP ${CMAKE_SOURCE_DIR}/apps/QPCC)
link_directories(${SNOPTF2C} ${SNOPTLCP})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -Wno-deprecated-declarations")
add_executable(CntctLrnin ${CntctLrnin_srcs} ${CntctLrnin_hdrs} ${MyLemke_hdrs} ${MyLemke_srcs})
target_link_libraries(CntctLrnin SNOPTLCP dart snopt_c snprint_c blas_c f2c)
set_target_properties(CntctLrnin PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
