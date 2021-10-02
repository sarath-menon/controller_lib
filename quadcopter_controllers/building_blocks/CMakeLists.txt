# ###############################################################################
# # Library of building blocks
# ###############################################################################

add_library(building_blocks_lib INTERFACE)

target_link_libraries(building_blocks_lib INTERFACE
# decoupled_pid_controllers
controller_architectures_lib
quadcopter_properties
basic_controllers_lib
axes_controllers_lib
)

# add_subdirectory(decoupled_pid_controllers)
add_subdirectory(controller_architectures)
add_subdirectory(quadcopter_properties)
add_subdirectory(axes_controllers)