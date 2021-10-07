
# ###############################################################################
# # Library of complete quadcopter controllers
# ###############################################################################

add_library(attitude_controllers_lib INTERFACE)

target_link_libraries(attitude_controllers_lib INTERFACE
roll_pitch_controllers_lib
# yaw_controllers_lib
# full_attitude_controllers_lib
)

add_subdirectory(roll_pitch_controllers)
# add_subdirectory(yaw_controllers)
# add_subdirectory(full_attitude_controllers)