cmake_minimum_required(VERSION 3.22)
project(mini_vio LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)   # build/compile_commands.json for IDEs and clangd

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif()

# All dependencies are installed in the container image (see Containerfile).
find_package(Eigen3 3.4 REQUIRED NO_MODULE)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs calib3d video features2d)
find_package(yaml-cpp REQUIRED)
find_package(GTSAM REQUIRED)        # 4.3: IncrementalFixedLagSmoother is in gtsam, not gtsam_unstable
find_package(Ceres REQUIRED)
find_package(rerun_sdk REQUIRED)

# Compiler warnings for our own code only. Dependencies are imported targets,
# so their headers are treated as system headers and do not trigger warnings.
add_library(mini_vio_warnings INTERFACE)
target_compile_options(mini_vio_warnings INTERFACE -Wall -Wextra -Wpedantic)

# Everything a mini-VIO target needs: C++20, warnings and all dependencies.
# The core library (src/, include/mini_vio/) will link this once stage 1 adds
# its first sources:
#   add_library(mini_vio_core src/...)
#   target_include_directories(mini_vio_core PUBLIC include)
#   target_link_libraries(mini_vio_core PUBLIC mini_vio_deps)
add_library(mini_vio_deps INTERFACE)
target_link_libraries(mini_vio_deps INTERFACE
    mini_vio_warnings
    Eigen3::Eigen
    ${OpenCV_LIBS}
    yaml-cpp::yaml-cpp
    gtsam
    Ceres::ceres
    rerun_sdk
)

enable_testing()
add_subdirectory(tests)
