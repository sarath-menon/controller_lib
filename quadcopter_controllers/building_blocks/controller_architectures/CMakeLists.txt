
# ###############################################################################
# # Library of complete quadcopter controllers
# ###############################################################################


add_library(controller_architectures_lib STATIC
cascaded/cascaded.cpp
)

target_include_directories(controller_architectures_lib PUBLIC cascaded)

target_link_libraries(controller_architectures_lib PRIVATE
basic_controllers_lib
building_blocks_lib
yaml-cpp
)

