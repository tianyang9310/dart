###############################################
# apps/CntctLrnin
file(GLOB CntctLrnin_srcs "*.cpp")
file(GLOB CntctLrnin_hdrs "*.h")
file(GLOB MyLemke_srcs "../lemkeFix/myLemke.cpp")
file(GLOB MyLemke_hdrs "../lemkeFix/myLemke.h")

set(SNOPTLCP ${CMAKE_SOURCE_DIR}/apps/QPCC)
link_directories(${SNOPTLCP})

# need to exclude main.cpp
file(GLOB MAIN_CPP "Main.cpp")
list (REMOVE_ITEM CntctLrnin_srcs ${MAIN_CPP})
link_libraries(SNOPTLCP dart snopt_c snprint_c blas_c f2c)
add_library(CntctLrninAPI STATIC ${CntctLrnin_srcs} ${CntctLrnin_hdrs} ${MyLemke_hdrs} ${MyLemke_srcs})

add_executable(CntctLrnin ${MAIN_CPP})
target_link_libraries(CntctLrnin CntctLrninAPI SNOPTLCP dart snopt_c snprint_c blas_c f2c)
set_target_properties(CntctLrnin PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
