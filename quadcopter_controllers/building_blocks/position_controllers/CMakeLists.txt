
# ###############################################################################
# # Library of complete quadcopter controllers
# ###############################################################################

add_library(position_controllers_lib INTERFACE)

target_link_libraries(position_controllers_lib INTERFACE
horizontal_controllers_lib
vertical_controllers_lib
)

add_subdirectory(horizontal_controllers)
add_subdirectory(vertical_controllers)
