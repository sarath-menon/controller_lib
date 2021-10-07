

# ###############################################################################
# # Library of complete common elements
# ###############################################################################

add_library(common_lib INTERFACE)

target_link_libraries(common_lib INTERFACE
basic_controllers_lib
)

add_subdirectory(axis_controllers)

