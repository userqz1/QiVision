# =============================================================================
# QiVision Library
# =============================================================================

set(QIVISION_SOURCES
    # Core
    Core/Types.cpp
    Core/QImage.cpp
    Core/QRegion.cpp
    Core/QMatrix.cpp
    Core/QContour.cpp
    Core/QContourArray.cpp
    # Platform
    Platform/Memory.cpp
    Platform/SIMD.cpp
    Platform/Random.cpp
    Platform/Thread.cpp
    Platform/Timer.cpp
    Platform/FileIO.cpp
    # Internal
    Internal/Gaussian.cpp
    Internal/Interpolate.cpp
    Internal/Gradient.cpp
    Internal/Convolution.cpp
    Internal/Edge1D.cpp
    Internal/Hessian.cpp
    Internal/Steger.cpp
    Internal/NonMaxSuppression.cpp
    Internal/EdgeLinking.cpp
    Internal/Canny.cpp
    Internal/Pyramid.cpp
    Internal/Profiler.cpp
    Internal/Histogram.cpp
    Internal/Threshold.cpp
    Internal/Matrix.cpp
    Internal/Solver.cpp
    Internal/Fitting.cpp
    Internal/SubPixel.cpp
    Internal/Geometry2d.cpp
    Internal/Distance.cpp
    Internal/Intersection.cpp
    Internal/GeomConstruct.cpp
    Internal/GeomRelation.cpp
    Internal/ContourProcess.cpp
    Internal/ContourAnalysis.cpp
    Internal/ContourSelect.cpp
    Internal/ContourConvert.cpp
    Internal/ContourSegment.cpp
    Internal/RLEOps.cpp
    Internal/StructElement.cpp
    Internal/MorphBinary.cpp
    Internal/MorphGray.cpp
    Internal/ConnectedComponent.cpp
    Internal/DistanceTransform.cpp
    Internal/RegionFeatures.cpp
    Internal/AffineTransform.cpp
    Internal/Homography.cpp
    Internal/Eigen.cpp
    Internal/Hough.cpp
    Internal/AnglePyramid.cpp
    # Internal/LinemodPyramid.cpp  # Archived - using XLD method
    # Internal/ResponseMap.cpp     # Archived - using XLD method
    Internal/IntegralImage.cpp
    Internal/PolarTransform.cpp
    Internal/CornerRefine.cpp
    # Matching (Feature Layer)
    Matching/ShapeModel.cpp
    Matching/ShapeModelCreate.cpp
    Matching/ShapeModelSearch.cpp
    Matching/ShapeModelScore.cpp
    Matching/NCCModel.cpp
    Matching/NCCModelCreate.cpp
    Matching/NCCModelSearch.cpp
    Matching/NCCModelScore.cpp
    # Measure (Feature Layer)
    Measure/MeasureHandle.cpp
    Measure/Caliper.cpp
    Measure/CaliperArray.cpp
    Measure/Metrology.cpp
    # IO (Feature Layer)
    IO/ImageIO.cpp
    # Color (Feature Layer)
    Color/ColorConvert.cpp
    # Filter (Feature Layer)
    Filter/Filter.cpp
    # Morphology (Feature Layer)
    Morphology/Morphology.cpp
    # Blob (Feature Layer)
    Blob/Blob.cpp
    # Segment (Feature Layer)
    Segment/Segment.cpp
    # Display (Feature Layer)
    Display/Display.cpp
    Display/Draw.cpp
    # GUI (Feature Layer)
    GUI/Window.cpp
    # Calib (Feature Layer)
    Calib/CameraModel.cpp
    Calib/Undistort.cpp
    Calib/CalibBoard.cpp
    Calib/CameraCalib.cpp
    # Transform (Feature Layer)
    Transform/PolarTransform.cpp
)

set(QIVISION_HEADERS
    # Main header
    ${CMAKE_SOURCE_DIR}/include/QiVision/QiVision.h
    # Core
    ${CMAKE_SOURCE_DIR}/include/QiVision/Core/Types.h
    ${CMAKE_SOURCE_DIR}/include/QiVision/Core/Constants.h
    ${CMAKE_SOURCE_DIR}/include/QiVision/Core/Exception.h
    ${CMAKE_SOURCE_DIR}/include/QiVision/Core/QImage.h
    ${CMAKE_SOURCE_DIR}/include/QiVision/Core/QRegion.h
    ${CMAKE_SOURCE_DIR}/include/QiVision/Core/QMatrix.h
    ${CMAKE_SOURCE_DIR}/include/QiVision/Core/QContour.h
    ${CMAKE_SOURCE_DIR}/include/QiVision/Core/QContourArray.h
    ${CMAKE_SOURCE_DIR}/include/QiVision/Core/Draw.h
    # Platform
    ${CMAKE_SOURCE_DIR}/include/QiVision/Platform/Memory.h
    ${CMAKE_SOURCE_DIR}/include/QiVision/Platform/SIMD.h
    ${CMAKE_SOURCE_DIR}/include/QiVision/Platform/Random.h
    ${CMAKE_SOURCE_DIR}/include/QiVision/Platform/Thread.h
    ${CMAKE_SOURCE_DIR}/include/QiVision/Platform/Timer.h
    ${CMAKE_SOURCE_DIR}/include/QiVision/Platform/FileIO.h
    # Internal
    ${CMAKE_SOURCE_DIR}/include/QiVision/Internal/Gaussian.h
    ${CMAKE_SOURCE_DIR}/include/QiVision/Internal/Interpolate.h
    ${CMAKE_SOURCE_DIR}/include/QiVision/Internal/Gradient.h
    ${CMAKE_SOURCE_DIR}/include/QiVision/Internal/Convolution.h
    ${CMAKE_SOURCE_DIR}/include/QiVision/Internal/Edge1D.h
    ${CMAKE_SOURCE_DIR}/include/QiVision/Internal/Hessian.h
    ${CMAKE_SOURCE_DIR}/include/QiVision/Internal/Steger.h
    ${CMAKE_SOURCE_DIR}/include/QiVision/Internal/NonMaxSuppression.h
    ${CMAKE_SOURCE_DIR}/include/QiVision/Internal/EdgeLinking.h
    ${CMAKE_SOURCE_DIR}/include/QiVision/Internal/Canny.h
    ${CMAKE_SOURCE_DIR}/include/QiVision/Internal/Pyramid.h
    ${CMAKE_SOURCE_DIR}/include/QiVision/Internal/Profiler.h
    ${CMAKE_SOURCE_DIR}/include/QiVision/Internal/Histogram.h
    ${CMAKE_SOURCE_DIR}/include/QiVision/Internal/Threshold.h
    ${CMAKE_SOURCE_DIR}/include/QiVision/Internal/Matrix.h
    ${CMAKE_SOURCE_DIR}/include/QiVision/Internal/Solver.h
    ${CMAKE_SOURCE_DIR}/include/QiVision/Internal/Fitting.h
    ${CMAKE_SOURCE_DIR}/include/QiVision/Internal/SubPixel.h
    ${CMAKE_SOURCE_DIR}/include/QiVision/Internal/Geometry2d.h
    ${CMAKE_SOURCE_DIR}/include/QiVision/Internal/Distance.h
    ${CMAKE_SOURCE_DIR}/include/QiVision/Internal/Intersection.h
    ${CMAKE_SOURCE_DIR}/include/QiVision/Internal/GeomConstruct.h
    ${CMAKE_SOURCE_DIR}/include/QiVision/Internal/GeomRelation.h
    ${CMAKE_SOURCE_DIR}/include/QiVision/Internal/ContourProcess.h
    ${CMAKE_SOURCE_DIR}/include/QiVision/Internal/ContourAnalysis.h
    ${CMAKE_SOURCE_DIR}/include/QiVision/Internal/ContourSelect.h
    ${CMAKE_SOURCE_DIR}/include/QiVision/Internal/ContourConvert.h
    ${CMAKE_SOURCE_DIR}/include/QiVision/Internal/ContourSegment.h
    ${CMAKE_SOURCE_DIR}/include/QiVision/Internal/RLEOps.h
    ${CMAKE_SOURCE_DIR}/include/QiVision/Internal/StructElement.h
    ${CMAKE_SOURCE_DIR}/include/QiVision/Internal/MorphBinary.h
    ${CMAKE_SOURCE_DIR}/include/QiVision/Internal/MorphGray.h
    ${CMAKE_SOURCE_DIR}/include/QiVision/Internal/ConnectedComponent.h
    ${CMAKE_SOURCE_DIR}/include/QiVision/Internal/DistanceTransform.h
    ${CMAKE_SOURCE_DIR}/include/QiVision/Internal/RegionFeatures.h
    ${CMAKE_SOURCE_DIR}/include/QiVision/Internal/AffineTransform.h
    ${CMAKE_SOURCE_DIR}/include/QiVision/Internal/Homography.h
    ${CMAKE_SOURCE_DIR}/include/QiVision/Internal/Eigen.h
    ${CMAKE_SOURCE_DIR}/include/QiVision/Internal/Hough.h
    ${CMAKE_SOURCE_DIR}/include/QiVision/Internal/AnglePyramid.h
    # ${CMAKE_SOURCE_DIR}/include/QiVision/Internal/LinemodPyramid.h  # Archived
    # ${CMAKE_SOURCE_DIR}/include/QiVision/Internal/ResponseMap.h     # Archived
    ${CMAKE_SOURCE_DIR}/include/QiVision/Internal/IntegralImage.h
    ${CMAKE_SOURCE_DIR}/include/QiVision/Internal/PolarTransform.h
    ${CMAKE_SOURCE_DIR}/include/QiVision/Internal/CornerRefine.h
    # Matching (Feature Layer)
    ${CMAKE_SOURCE_DIR}/include/QiVision/Matching/MatchTypes.h
    ${CMAKE_SOURCE_DIR}/include/QiVision/Matching/ShapeModel.h
    ${CMAKE_SOURCE_DIR}/include/QiVision/Matching/NCCModel.h
    # Matching internal headers (in src/)
    ${CMAKE_SOURCE_DIR}/src/Matching/ShapeModelImpl.h
    ${CMAKE_SOURCE_DIR}/src/Matching/NCCModelImpl.h
    # Measure (Feature Layer)
    ${CMAKE_SOURCE_DIR}/include/QiVision/Measure/MeasureTypes.h
    ${CMAKE_SOURCE_DIR}/include/QiVision/Measure/MeasureHandle.h
    ${CMAKE_SOURCE_DIR}/include/QiVision/Measure/Caliper.h
    ${CMAKE_SOURCE_DIR}/include/QiVision/Measure/CaliperArray.h
    ${CMAKE_SOURCE_DIR}/include/QiVision/Measure/Metrology.h
    # IO (Feature Layer)
    ${CMAKE_SOURCE_DIR}/include/QiVision/IO/ImageIO.h
    # Color (Feature Layer)
    ${CMAKE_SOURCE_DIR}/include/QiVision/Color/ColorConvert.h
    # Filter (Feature Layer)
    ${CMAKE_SOURCE_DIR}/include/QiVision/Filter/Filter.h
    # Morphology (Feature Layer)
    ${CMAKE_SOURCE_DIR}/include/QiVision/Morphology/Morphology.h
    # Blob (Feature Layer)
    ${CMAKE_SOURCE_DIR}/include/QiVision/Blob/Blob.h
    # Segment (Feature Layer)
    ${CMAKE_SOURCE_DIR}/include/QiVision/Segment/Segment.h
    # Display (Feature Layer)
    ${CMAKE_SOURCE_DIR}/include/QiVision/Display/Display.h
    ${CMAKE_SOURCE_DIR}/include/QiVision/Display/Draw.h
    # GUI (Feature Layer)
    ${CMAKE_SOURCE_DIR}/include/QiVision/GUI/Window.h
    # Calib (Feature Layer)
    ${CMAKE_SOURCE_DIR}/include/QiVision/Calib/CameraModel.h
    ${CMAKE_SOURCE_DIR}/include/QiVision/Calib/Undistort.h
    ${CMAKE_SOURCE_DIR}/include/QiVision/Calib/CalibBoard.h
    ${CMAKE_SOURCE_DIR}/include/QiVision/Calib/CameraCalib.h
    # Transform (Feature Layer)
    ${CMAKE_SOURCE_DIR}/include/QiVision/Transform/PolarTransform.h
)

# Create library
add_library(QiVision ${QIVISION_SOURCES} ${QIVISION_HEADERS})

# Include directories
target_include_directories(QiVision
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${CMAKE_SOURCE_DIR}/third_party
)

# Link libraries
if(QIVISION_ENABLE_OPENMP)
    find_package(OpenMP)
    if(OpenMP_CXX_FOUND)
        target_link_libraries(QiVision PUBLIC OpenMP::OpenMP_CXX)
    endif()
endif()

# =============================================================================
# Platform-specific GUI support
# =============================================================================

if(WIN32)
    # Windows: Win32 GDI (built-in, no extra libs needed)
    message(STATUS "Platform: Windows - GUI support enabled (Win32 GDI)")

elseif(APPLE)
    # macOS: Cocoa support (optional, requires .mm compilation)
    # Currently using stub - full Cocoa support requires Window.mm
    message(STATUS "Platform: macOS/iOS - GUI stub (Cocoa requires .mm)")
    target_compile_definitions(QiVision PRIVATE QIVISION_NO_GUI)
    # Future: Enable Cocoa support
    # find_library(COCOA_LIBRARY Cocoa)
    # find_library(APPKIT_LIBRARY AppKit)
    # if(COCOA_LIBRARY)
    #     target_link_libraries(QiVision PRIVATE ${COCOA_LIBRARY})
    #     target_compile_definitions(QiVision PRIVATE QIVISION_HAS_COCOA)
    # endif()

elseif(ANDROID)
    # Android: GUI requires Java layer (Activity/View)
    message(STATUS "Platform: Android - GUI stub (requires Java layer)")
    target_compile_definitions(QiVision PRIVATE QIVISION_NO_GUI)

elseif(UNIX)
    # Linux: X11 (optional)
    find_package(X11 QUIET)
    if(X11_FOUND)
        target_link_libraries(QiVision PRIVATE ${X11_LIBRARIES})
        target_include_directories(QiVision PRIVATE ${X11_INCLUDE_DIR})
        target_compile_definitions(QiVision PRIVATE QIVISION_HAS_X11)
        message(STATUS "Platform: Linux - GUI support enabled (X11)")
    else()
        message(STATUS "Platform: Linux - GUI stub (X11 not found)")
        target_compile_definitions(QiVision PRIVATE QIVISION_NO_GUI)
    endif()

else()
    # Unknown platform
    message(STATUS "Platform: Unknown - GUI stub")
    target_compile_definitions(QiVision PRIVATE QIVISION_NO_GUI)
endif()

# Compiler definitions
target_compile_definitions(QiVision
    PRIVATE
        $<$<CONFIG:Debug>:QIVISION_DEBUG>
)

# Set library properties
set_target_properties(QiVision PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
    DEBUG_POSTFIX "d"
)
