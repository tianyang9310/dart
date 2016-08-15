# tutorial/cartpole

file(GLOB mpc_cartpole_srcs "*.cpp")
file(GLOB mpc_cartpole_hdrs "*.h")
add_executable(mpc_cartpole ${mpc_cartpole_srcs} ${mpc_cartpole_hdrs})
target_link_libraries(mpc_cartpole dart MPC DDP)
