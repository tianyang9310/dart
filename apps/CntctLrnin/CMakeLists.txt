###############################################
# apps/CntctLrnin
file(GLOB CntctLrnin_srcs "*.cpp")
file(GLOB CntctLrnin_hdrs "*.h")
file(GLOB MyLemke_srcs "../Lemke_Fix/MyLemke.cpp")
file(GLOB MyLemke_hdrs "../Lemke_Fix/MyLemke.h")

set(SNOPTLCP ${CMAKE_SOURCE_DIR}/apps/QPCC)
link_directories(${SNOPTLCP})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -Wno-deprecated-declarations")
add_executable(CntctLrnin ${CntctLrnin_srcs} ${CntctLrnin_hdrs} ${MyLemke_hdrs} ${MyLemke_srcs})
target_link_libraries(CntctLrnin SNOPTLCP dart snopt_c snprint_c blas_c f2c)
set_target_properties(CntctLrnin PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

# need to exclude main.cpp
file(GLOB MAIN_CPP "Main.cpp")
list (REMOVE_ITEM CntctLrnin_srcs ${MAIN_CPP})
link_libraries(SNOPTLCP dart snopt_c snprint_c blas_c f2c)
add_library(CntctLrninAPI STATIC ${CntctLrnin_srcs} ${CntctLrnin_hdrs} ${MyLemke_hdrs} ${MyLemke_srcs})
