
# ###############################################################################
# # Library of complete quadcopter controllers
# ###############################################################################

add_library(controller_architectures_lib INTERFACE)

target_link_libraries(controller_architectures_lib INTERFACE
cascaded
building_blocks_lib
)

add_subdirectory(cascaded)
