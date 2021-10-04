

# ###############################################################################
# # Library of complete quadcopter controllers
# ###############################################################################

add_library(quadcopter_controllers_lib INTERFACE)

target_link_libraries(quadcopter_controllers_lib INTERFACE
building_blocks_lib
full_controllers_lib
)

add_subdirectory(building_blocks)
add_subdirectory(full_controllers)
