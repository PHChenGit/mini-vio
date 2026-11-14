# Mini-VIO build environment.
#
# Ubuntu 24.04 (GCC 13, C++20) + OpenCV + Eigen + yaml-cpp + Ceres + GoogleTest
# from apt, GTSAM and the Rerun C++ SDK built from source at pinned versions,
# evo in a Python venv for trajectory evaluation.
#
# Build:  scripts/podman.sh build

FROM docker.io/library/ubuntu:24.04

ARG DEBIAN_FRONTEND=noninteractive
ARG GTSAM_VERSION=4.3.0
ARG RERUN_VERSION=0.43.2
ARG EVO_VERSION=1.38.1

RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential cmake ninja-build git ca-certificates curl unzip pkg-config gdb \
        libeigen3-dev libopencv-dev libyaml-cpp-dev \
        libboost-graph-dev libboost-serialization-dev libboost-program-options-dev \
        libboost-random-dev libboost-timer-dev libboost-chrono-dev \
        python3 python3-venv python3-pip \
    && rm -rf /var/lib/apt/lists/*

# GTSAM. Notes on the flags:
# - System Eigen so OpenCV, GTSAM and our code all share one Eigen.
# - No -march=native: the image must run on any x86-64 host, and mixing
#   native/non-native builds causes Eigen alignment crashes.
# - No TBB: version 1 is single-threaded and the KPIs are defined that way.
# - gtsam_unstable is kept in case IncrementalFixedLagSmoother lives there.
RUN git clone --depth 1 --branch ${GTSAM_VERSION} https://github.com/borglab/gtsam.git /tmp/gtsam \
    && cmake -S /tmp/gtsam -B /tmp/gtsam/build -G Ninja \
        -DCMAKE_BUILD_TYPE=Release \
        -DGTSAM_USE_SYSTEM_EIGEN=ON \
        -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
        -DGTSAM_WITH_TBB=OFF \
        -DGTSAM_BUILD_UNSTABLE=ON \
        -DGTSAM_BUILD_TESTS=OFF \
        -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
        -DGTSAM_BUILD_PYTHON=OFF \
    && cmake --build /tmp/gtsam/build \
    && cmake --install /tmp/gtsam/build \
    && rm -rf /tmp/gtsam

# Rerun C++ SDK. Its CMake downloads and statically builds a pinned Arrow,
# which ends up inside the rerun_sdk install. Use with find_package(rerun_sdk).
RUN curl -fsSL -o /tmp/rerun_cpp_sdk.zip \
        https://github.com/rerun-io/rerun/releases/download/${RERUN_VERSION}/rerun_cpp_sdk.zip \
    && unzip -q /tmp/rerun_cpp_sdk.zip -d /tmp \
    && cmake -S /tmp/rerun_cpp_sdk -B /tmp/rerun_cpp_sdk/build -G Ninja -DCMAKE_BUILD_TYPE=Release \
    && cmake --build /tmp/rerun_cpp_sdk/build --target rerun_sdk \
    && cmake --install /tmp/rerun_cpp_sdk/build \
    && rm -rf /tmp/rerun_cpp_sdk /tmp/rerun_cpp_sdk.zip

# Packages added after the first image was built. Kept in their own layer so
# adding them does not invalidate the cached GTSAM and Rerun builds above.
# - Ceres Solver 2.2.0 (same system Eigen as GTSAM and OpenCV):
#   find_package(Ceres), link Ceres::ceres.
# - GoogleTest: find_package(GTest), link GTest::gtest_main.
RUN apt-get update && apt-get install -y --no-install-recommends \
        libceres-dev libgtest-dev \
    && rm -rf /var/lib/apt/lists/*

RUN ldconfig

# evo for ATE evaluation (Ubuntu 24.04 forbids system-wide pip, so use a venv).
ENV VIRTUAL_ENV=/opt/venv
RUN python3 -m venv ${VIRTUAL_ENV} \
    && ${VIRTUAL_ENV}/bin/pip install --no-cache-dir evo==${EVO_VERSION}
ENV PATH=${VIRTUAL_ENV}/bin:${PATH} \
    MPLBACKEND=Agg

WORKDIR /workspace
