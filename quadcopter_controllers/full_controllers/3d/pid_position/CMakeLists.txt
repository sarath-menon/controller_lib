
add_library(pid_position STATIC
src/position_controller.cpp
src/z_position_controller.cpp
src/horizontal_controllers.cpp
src/set_values.cpp
)

target_include_directories(pid_position PUBLIC include)

target_link_libraries(pid_position PRIVATE
basic_controllers_lib
comm_lib
yaml-cpp
)

