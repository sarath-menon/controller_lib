
add_library(basic_pid_cascaded_2d STATIC
src/cascaded_controller.cpp
src/z_position_controller.cpp
src/y_position_controller.cpp
src/roll_angle_controller.cpp
src/set_values.cpp
)

target_include_directories(basic_pid_cascaded_2d PUBLIC include)

target_link_libraries(basic_pid_cascaded_2d PRIVATE
basic_controllers_lib
yaml-cpp
)

