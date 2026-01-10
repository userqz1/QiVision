# =============================================================================
# QiVision Samples
# =============================================================================

# Sample executables (only include samples with stable API)
set(SAMPLES
    01_basic_image
    06_contour_segment
    07_shape_match_draw
    08_shape_match_large
    09_performance_benchmark
)

foreach(sample ${SAMPLES})
    add_executable(${sample} ${sample}.cpp)
    target_link_libraries(${sample} PRIVATE QiVision)
    target_include_directories(${sample} PRIVATE ${CMAKE_SOURCE_DIR}/include)

    # Set output directory
    set_target_properties(${sample} PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/samples
    )
endforeach()

# Create a custom target to build all samples
add_custom_target(samples DEPENDS ${SAMPLES})
