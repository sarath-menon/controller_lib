
# ###############################################################################
# # Library of complete quadcopter controllers
# ###############################################################################

add_library(3d_controllers_lib INTERFACE)

target_link_libraries(3d_controllers_lib INTERFACE
pid_cascaded_3d
pid_position
comm_lib
)


add_subdirectory(pid_cascaded)
add_subdirectory(pid_position)

