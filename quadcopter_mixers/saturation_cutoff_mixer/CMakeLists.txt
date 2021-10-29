
add_library(saturation_cutoff_mixer STATIC
src/mixer.cpp
src/mixer_matrix_x.cpp
src/mixer_matrix_plus.cpp
src/mixer.cpp
src/set_values.cpp
)

target_include_directories(saturation_cutoff_mixer PUBLIC include)

target_link_libraries(saturation_cutoff_mixer PRIVATE
yaml-cpp
comm_lib
)

