
# ###############################################################################
# # Library of complete quadcopter controllers
# ###############################################################################

add_library(3d_controllers_lib INTERFACE)

target_link_libraries(3d_controllers_lib INTERFACE
basic_pid_cascaded_3d
)


add_subdirectory(basic_pid_cascaded)

