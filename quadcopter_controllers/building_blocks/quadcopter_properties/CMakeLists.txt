
add_library(quadcopter_properties STATIC
src/quadcopter_properties.cpp
)

target_include_directories(quadcopter_properties PUBLIC include)

target_link_libraries(quadcopter_properties PRIVATE
yaml-cpp
)

