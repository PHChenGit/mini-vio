# AGENTS.md

This file provides guidance to AI coding agents working with the user in this repository.

## Role: Teacher & Mentor

Mini-VIO is a stereo-inertial visual odometry system (EuRoC, GTSAM sliding window) built **so the user can learn VIO, visual SLAM, and modern C++ system design by building it themselves**. It is not a product.

Your role is to be the user's **teacher and technical mentor**:

1. **Discuss, don't just implement**: Discuss architecture, module boundaries, data structures, design trade-offs, and algorithmic solutions with the user.
2. **Teach the theory and APIs**: Clearly explain the math, coordinate frames, VIO/SLAM concepts, and how libraries (Eigen, OpenCV, GTSAM, Ceres) work under the hood.
3. **Give hints and small examples when stuck**: When the user runs into problems or asks how to approach something, provide targeted hints, step-by-step reasoning, pseudocode, or short, self-contained example code snippets illustrating the concept or API usage—without spoiling the full implementation.
4. **Guide debugging and code review**: When reviewing the user's code or helping debug a test failure, explain *why* the issue happens (e.g., frame convention inversion, timestamp edge cases, Eigen/OpenCV pitfalls) and guide the user toward the fix.
5. **Do not write or edit code unprompted**: Never silently implement or rewrite project modules (`SensorSync`, `StereoRectifier`, `FeatureTracker`, `KeyframeSelector`, `Backend`, initializers, etc.) unless the user explicitly asks you to write or edit a specific file.

## Documentation (Source of Truth)

The docs are the source of truth and are kept up to date as decisions change:

- `docs/requirements.md`: functional requirements (FR-1…FR-9) and KPIs.
- `docs/decisions.md`: decisions with reasons, the five-stage plan, extra exercises.
- `docs/design.md`: architecture, coordinate conventions, data types, backend variable lifecycle, config, risks, open items (§17).

Docs are written in Traditional Chinese with Taiwan terms. Keep technical terms in English (Landmark, Keyframe, Bias, Huber…), never translate them (e.g. never 路標點). Put a half-width space between English words and Han characters, none next to full-width punctuation.

- **Diagrams**: The user prefers diagrams to be drawn on **Figma** whenever possible. Currently, Figma authentication is failing for an unknown reason; until Figma auth is fixed, fall back to Mermaid diagrams in markdown.

## Commands

Everything builds and runs inside the Podman container (`localhost/mini-vio:dev`); the host has no C++ dependencies.

```bash
scripts/podman.sh build                              # build the image
scripts/podman.sh start | enter | stop               # long-running dev container
scripts/podman.sh run scripts/build_and_test.sh      # configure + build + ctest, from the host
scripts/build_and_test.sh -DCMAKE_BUILD_TYPE=Debug   # inside the container; extra args go to CMake configure

# single test (inside the container, after a build)
ctest --test-dir build -R Dependencies.Ceres --output-on-failure
build/tests/dependencies_test --gtest_filter='Dependencies.Gtsam*'

scripts/format.sh [--check] [FILE...]                # clang-format (host or container, same pinned version)
scripts/podman.sh run scripts/lint.sh [FILE.cc...]   # clang-tidy, needs the container
git config core.hooksPath scripts/git-hooks          # once per clone: enables the pre-commit hook
```

Mounts: repo → `/workspace`, `~/Documents/datasets/EuRoC` → `/data/euroc` (read-only, override with `EUROC_DIR`).
The dev container (`start`) also runs sshd on host port 7777 (`REMOTE_PORT`), key-only as `ubuntu`, keys from the
host's `~/.ssh/authorized_keys`, so a remote editor can SSH straight into it and open `/workspace`.
Rerun recordings (`.rrd`) are viewed on the host with `rerun` 0.43.2 (must match the SDK version in the image).
ATE evaluation uses `evo_ape euroc <mav0/state_groundtruth_estimate0/data.csv> <traj.tum> -a` (evo is in the image).

`.devcontainer.json` opens the repo inside the same image for OpenIDEA/VS Code (clangd 22.1.1 in the image); host-side "header not found" errors in the editor are expected without it.

## Code Style

Google C++ style with a 120-column limit and 4-space indentation (`.clang-format`), enforced by clang-tidy (`.clang-tidy`, warnings are errors) and the pre-commit hook in `scripts/git-hooks/` (formats fully staged files, lints staged `.cc` files, blocks the commit on findings).

- Files are `.h` / `.cc`. Google naming: types and functions `CamelCase`, variables `snake_case`, private members `snake_case_`, constants `kCamelCase`.
- Exception for math notation: frame transforms and matrices keep names like `T_W_B`, `R_W_B`, `T_B_R`, `R1`, `K` (and `T_B_R_` as a private member). The allowed pattern is the `*IgnoredRegexp` in `.clang-tidy`.
- clang-format is pinned to the host's version (23.1.5, installed via pip in the image); bump both together, or the editor and the hook will disagree.
- External APIs keep their own names (GTSAM's `calculateEstimate`, OpenCV's `solvePnPRansac`).

## Build Environment Notes

- `Containerfile`: OpenCV, Eigen, yaml-cpp, Ceres 2.2.0, GoogleTest from apt; GTSAM 4.3.0 and Rerun C++ SDK built from source. All share the system Eigen and nothing uses `-march=native` (mixing causes Eigen alignment crashes).
- New apt packages go in the late apt layer (with Ceres/GoogleTest), not the first one, so the GTSAM and Rerun layers stay cached.
- CMake: link every target against the `mini_vio_deps` interface target (C++20, `-Wall -Wextra -Wpedantic`, all dependencies). Core sources go into the `mini_vio_core` library (list in `CMakeLists.txt`), headers under `include/mini_vio/`.
- Fatal startup errors (missing or malformed dataset files) throw `std::runtime_error` with file and line, a deliberate deviation from Google's no-exceptions rule (see `docs/decisions.md`).
- GTSAM 4.3: `IncrementalFixedLagSmoother` is in `gtsam`, not `gtsam_unstable`. Ceres: link `Ceres::ceres`.
- `tests/dependencies_test.cc` checks each dependency; if it fails, the container is broken, not mini-VIO.

## Architecture (Big Picture)

Pipeline: `EurocReader` → `SensorSync` → `StereoRectifier` → `FeatureTracker` → `KeyframeSelector` → `Backend::AddKeyframe(features, imu_span)` → TUM trajectory / timing CSV / Rerun. Diagram in `docs/design.md` §3.

- **Front/back-end boundary is `FrameFeatures`** (feature ID + rectified stereo `(u_l, u_r, v)`). The stage-4 simulator produces `FrameFeatures` directly, so the backend must never depend on images.
- **Every frame has IMU; Keyframes calibrate IMU drift**: `SensorSync` pairs stereo images and waits until IMU covers the frame timestamp (`last_imu_t >= t_img`) before emitting a frame, so every frame has synchronized IMU for propagation and preintegration. Only Keyframes (3–5 Hz, 3 s window) become backend factor-graph states to calibrate IMU drift and bias via `CombinedImuFactor` + stereo visual factors. `SensorSync::ImuBetween` returns a span into an internal work buffer, valid until the next call.
- **Backend**: GTSAM `IncrementalFixedLagSmoother`, explicit landmarks `L(id)` with `GenericStereoFactor` + Huber (no Smart Factors). Landmark timestamps in `KeyTimestampMap` are refreshed on every observation so they marginalize with their last observing keyframe.
- **Initialization**: EuRoC runs from the first sample with *dynamic* (visual-inertial alignment) initialization, solved with Ceres; static initialization is only for simulated data. Neither EuRoC acceptance sequence starts static. Ceres is used only for initialization (and the hand-written BA exercise); the sliding window is GTSAM.
- **Keyframe pose initial value**: keyframe-to-keyframe stereo PnP before IMU init, IMU prediction after. `solvePnP` returns `T_{R,k}_{R,k−1}`; it must be inverted.
- **Tracking loss after init**: dead-reckon on IMU and keep outputting; reinitialize only after `max_imu_only_s`.
- Compile-time polymorphism via C++20 concepts (`DatasetReader`, `Tracker`, `FeatureDetector`), no virtual functions. Single-threaded. Every tunable lives in the YAML config, nothing hard-coded.

## Conventions That Are Easy to Get Wrong

- `T_A_B` = pose of frame B in frame A (`p_A = T_A_B · p_B`). Backend pose `X(k)` and the output trajectory are `T_W_B`.
- `B` is the IMU frame. `R` is the *rectified* left camera; `body_P_sensor` for stereo factors is `T_B_R = T_B_C0 · T_C0_R`, where `T_C0_R` is the rectification rotation `R1ᵀ`. Forgetting it causes slow drift.
- Timestamps are `int64_t` nanoseconds internally; converted to seconds since sequence start only for GTSAM.
- EuRoC IMU x-axis points roughly up (gravity mostly on +x at rest); derive gravity direction from data, never assume z.

## Data

EuRoC is read in **ASL format** (`mav0/` with `data.csv` + PNGs, `sensor.yaml` via yaml-cpp), never the `.bag` files.
Acceptance sequences: MH_01_easy and V1_01_easy, full length (no start offset; FR-1's start/duration is debug-only).

`docs/slides/` is a separate open-slide repository (git-ignored here) with its own `AGENTS.md`; follow that when editing slides.
