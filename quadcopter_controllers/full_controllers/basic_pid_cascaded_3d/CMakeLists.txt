
add_library(basic_pid_cascaded STATIC
src/cascaded_controller.cpp
src/set_values.cpp
)

target_include_directories(basic_pid_cascaded PUBLIC include)

target_link_libraries(basic_pid_cascaded PRIVATE
basic_controllers_lib
building_blocks_lib
yaml-cpp
)

