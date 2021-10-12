
# ###############################################################################
# # Library of complete quadcopter controllers
# ###############################################################################

add_library(2d_controllers_lib INTERFACE)

target_link_libraries(2d_controllers_lib INTERFACE
basic_pid_cascaded_2d
comm_lib
)


add_subdirectory(basic_pid_cascaded)
# add_subdirectory(3d)
