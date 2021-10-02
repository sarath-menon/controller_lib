
add_library(cascaded STATIC
src/cascaded.cpp
)

target_include_directories(cascaded PUBLIC include)

target_link_libraries(cascaded PRIVATE
basic_controllers_lib
building_blocks_lib
yaml-cpp
)

