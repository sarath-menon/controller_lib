
# ###############################################################################
# # Library of complete quadcopter controllers
# ###############################################################################

add_library(building_blocks_lib INTERFACE)

target_link_libraries(building_blocks_lib INTERFACE
controller_architectures_lib
attitude_controllers_lib
position_controllers_lib
common_lib
others_lib
)

add_subdirectory(controller_architectures)
add_subdirectory(attitude_controllers)
add_subdirectory(position_controllers)
add_subdirectory(common)
add_subdirectory(others)
