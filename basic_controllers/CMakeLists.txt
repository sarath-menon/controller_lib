
add_library(basic_controllers_lib STATIC
pid/pid.cpp
)

target_include_directories(basic_controllers_lib PUBLIC
pid
)

# target_link_libraries(basic_controllers PRIVATE
# basic_controllers_lib
# yaml-cpp
# )

