
add_library(pid_cascaded_3d STATIC
src/cascaded_controller.cpp
src/z_position_controller.cpp
src/horizontal_controllers.cpp
src/angle_controllers.cpp
src/set_values.cpp
)

target_include_directories(pid_cascaded_3d PUBLIC include)

target_link_libraries(pid_cascaded_3d PRIVATE
basic_controllers_lib
comm_lib
yaml-cpp
)

