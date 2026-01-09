cmake_minimum_required(VERSION 3.16)

project(QiVision
    VERSION 0.1.0
    DESCRIPTION "Industrial Machine Vision Library"
    LANGUAGES CXX
)

# =============================================================================
# C++ Standard
# =============================================================================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# =============================================================================
# Build Options
# =============================================================================
option(QIVISION_BUILD_TESTS "Build unit tests" ON)
option(QIVISION_BUILD_SAMPLES "Build sample programs" ON)
option(QIVISION_ENABLE_SIMD "Enable SIMD optimizations" ON)
option(QIVISION_ENABLE_OPENMP "Enable OpenMP for parallelization" ON)

# =============================================================================
# Compiler Settings
# =============================================================================
if(MSVC)
    add_compile_options(/W4 /utf-8)
    add_compile_definitions(_USE_MATH_DEFINES)  # Enable M_PI etc.
    add_compile_definitions(NOMINMAX)           # Prevent Windows min/max macros
    if(QIVISION_ENABLE_SIMD)
        add_compile_options(/arch:AVX2)
    endif()
else()
    add_compile_options(-Wall -Wextra -Wpedantic)
    if(QIVISION_ENABLE_SIMD)
        include(CheckCXXCompilerFlag)
        check_cxx_compiler_flag(-mavx2 COMPILER_SUPPORTS_AVX2)
        check_cxx_compiler_flag(-msse4.1 COMPILER_SUPPORTS_SSE4)
        if(COMPILER_SUPPORTS_AVX2)
            add_compile_options(-mavx2 -mfma)
        elseif(COMPILER_SUPPORTS_SSE4)
            add_compile_options(-msse4.1)
        endif()
    endif()
endif()

# =============================================================================
# OpenMP Support
# =============================================================================
if(QIVISION_ENABLE_OPENMP)
    find_package(OpenMP)
    if(OpenMP_CXX_FOUND)
        message(STATUS "OpenMP found: ${OpenMP_CXX_VERSION}")
    else()
        message(WARNING "OpenMP not found, disabling parallel optimization")
        set(QIVISION_ENABLE_OPENMP OFF)
    endif()
endif()

# =============================================================================
# Output Directories
# =============================================================================
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# =============================================================================
# Main Library
# =============================================================================
add_subdirectory(src)

# =============================================================================
# Tests
# =============================================================================
if(QIVISION_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

# =============================================================================
# Samples
# =============================================================================
if(QIVISION_BUILD_SAMPLES)
    add_subdirectory(samples)
endif()

# =============================================================================
# Installation
# =============================================================================
include(GNUInstallDirs)

install(DIRECTORY include/QiVision
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(TARGETS QiVision
    EXPORT QiVisionTargets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# =============================================================================
# Summary
# =============================================================================
message(STATUS "")
message(STATUS "QiVision ${PROJECT_VERSION}")
message(STATUS "  Build type:    ${CMAKE_BUILD_TYPE}")
message(STATUS "  C++ Standard:  ${CMAKE_CXX_STANDARD}")
message(STATUS "  SIMD:          ${QIVISION_ENABLE_SIMD}")
message(STATUS "  Tests:         ${QIVISION_BUILD_TESTS}")
message(STATUS "  Samples:       ${QIVISION_BUILD_SAMPLES}")
message(STATUS "")
