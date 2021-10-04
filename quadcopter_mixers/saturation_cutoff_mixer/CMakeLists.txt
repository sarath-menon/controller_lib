
add_library(saturation_cutoff_mixer STATIC
src/saturation_cutoff_mixer.cpp
src/set_values.cpp
)

target_include_directories(saturation_cutoff_mixer PUBLIC include)

target_link_libraries(saturation_cutoff_mixer PRIVATE
yaml-cpp
)

