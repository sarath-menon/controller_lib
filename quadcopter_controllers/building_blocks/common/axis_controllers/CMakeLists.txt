
add_library(axis_controllers_lib STATIC
basic_axis_controller/basic_axis_controller.cpp
)

target_include_directories(axis_controllers_lib PUBLIC basic_axis_controller)

target_link_libraries(axis_controllers_lib PRIVATE
basic_controllers_lib
)

