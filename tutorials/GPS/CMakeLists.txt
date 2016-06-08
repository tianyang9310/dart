add_subdirectory( libGPS  )
add_subdirectory( GPS_DoubleCartPole  )

#link_directories( libGPS )
add_executable( main main.cpp)
target_link_libraries( main GPS)
