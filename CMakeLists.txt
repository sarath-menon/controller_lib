# ###############################################################################
# # Library of controllers
# ###############################################################################


add_library(controller_lib INTERFACE)

target_link_libraries(controller_lib INTERFACE
basic_controllers_lib
quadcopter_controllers_lib 
quadcopter_mixers_lib
utilities_lib
)


add_subdirectory(quadcopter_controllers)
add_subdirectory(quadcopter_mixers)
add_subdirectory(basic_controllers)
