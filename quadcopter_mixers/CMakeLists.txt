# ###############################################################################
# # Library of quadcopter mixers
# ###############################################################################

add_library(quadcopter_mixers_lib INTERFACE)

target_link_libraries(quadcopter_mixers_lib INTERFACE
saturation_cutoff_mixer
)

add_subdirectory(saturation_cutoff_mixer)