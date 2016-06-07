file(GLOB gps_double_cartpole_srcs "*.cpp")
file(GLOB gps_double_cartpole_hdrs "*.h")
add_executable(gps_double_cartpole ${gps_double_cartpole_srcs} ${gps_double_cartpole_hdrs})
target_link_libraries(gps_double_cartpole dart GPS DDP)
