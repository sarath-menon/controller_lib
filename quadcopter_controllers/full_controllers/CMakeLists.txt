
# ###############################################################################
# # Library of complete quadcopter controllers
# ###############################################################################

add_library(full_controllers_lib INTERFACE)

target_link_libraries(full_controllers_lib INTERFACE
2d_controllers_lib
3d_controllers_lib
)


add_subdirectory(2d)
add_subdirectory(3d)
