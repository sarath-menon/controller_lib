
add_library(others_lib STATIC
quadcopter_properties/quadcopter_properties.cpp
)

target_include_directories(others_lib PUBLIC quadcopter_properties)

target_link_libraries(others_lib PRIVATE
yaml-cpp
)

