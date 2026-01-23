# =============================================================================
# QiVision Samples
# =============================================================================

# Helper function to add sample executable
function(add_sample name source_file)
    add_executable(${name} ${source_file})
    target_link_libraries(${name} PRIVATE QiVision)
    target_include_directories(${name} PRIVATE ${CMAKE_SOURCE_DIR}/include)
    set_target_properties(${name} PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/samples
    )
endfunction()

# Matching - Shape matching
add_sample(matching_shape_match matching/shape_match.cpp)
add_sample(matching_test_create_model matching/test_create_model.cpp)
add_sample(matching_test_scaled_match matching/test_scaled_match.cpp)

# Measure - Metrology & Caliper
add_sample(measure_circle_metrology measure/circle_metrology.cpp)
add_sample(measure_caliper_pairs measure/caliper_pairs.cpp)
add_sample(measure_caliper_circle_manual measure/caliper_circle_manual.cpp)

# Blob - Blob analysis
add_sample(blob_analysis blob/blob_analysis.cpp)

# Build all samples
add_custom_target(samples DEPENDS
    matching_shape_match
    matching_test_create_model
    matching_test_scaled_match
    measure_circle_metrology
    measure_caliper_pairs
    measure_caliper_circle_manual
    blob_analysis
)
