
add_library(axes_controllers_lib STATIC
decoupled_pid/decoupled_pid.cpp
)

target_include_directories(axes_controllers_lib PUBLIC
decoupled_pid
)

target_link_libraries(axes_controllers_lib PRIVATE
basic_controllers_lib
)

