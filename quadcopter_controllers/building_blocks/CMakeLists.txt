
# ###############################################################################
# # Library of complete quadcopter controllers
# ###############################################################################

add_library(building_blocks_lib INTERFACE)

target_link_libraries(building_blocks_lib INTERFACE
axes_controllers_lib
controller_architectures_lib
others_lib
)

add_subdirectory(axes_controllers)
add_subdirectory(controller_architectures)
add_subdirectory(others)
