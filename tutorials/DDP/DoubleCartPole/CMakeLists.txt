###############################################
# tutorial/cartpole

file(GLOB double_cartpole_srcs "*.cpp")
file(GLOB double_cartpole_hdrs "*.h")
add_executable(double_cartpole ${double_cartpole_srcs} ${double_cartpole_hdrs})
target_link_libraries(double_cartpole dart DDP)
