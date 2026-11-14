# Mini-VIO

A minimal stereo-inertial visual odometry system for learning VIO and visual SLAM.
Goals and plan: [docs/requirements.md](docs/requirements.md), [docs/decisions.md](docs/decisions.md).

## Development environment (Podman)

All building and running happens inside a Podman container. The image
([Containerfile](Containerfile)) contains:

| Component | Version | Source |
|---|---|---|
| Ubuntu / GCC | 24.04 / 13 (C++20) | base image |
| OpenCV, Eigen, yaml-cpp, Boost | Ubuntu 24.04 packages | apt |
| GTSAM | 4.3.0 | built from source (system Eigen, no TBB, no `-march=native`) |
| Ceres Solver | 2.2.0 | apt (`libceres-dev`), `find_package(Ceres)`, link `Ceres::ceres` |
| GoogleTest | Ubuntu 24.04 package | apt (`libgtest-dev`) |
| Rerun C++ SDK | 0.43.2 | built from source, `find_package(rerun_sdk)` |
| evo | 1.38.1 | Python venv at `/opt/venv`, already on `PATH` |

[scripts/podman.sh](scripts/podman.sh) wraps the Podman commands. Run it from the repo root.

### 1. Build the image

```bash
scripts/podman.sh build
```

This takes a while the first time, because GTSAM and the Rerun SDK (and its Arrow dependency) are compiled from source.
It produces the image `localhost/mini-vio:dev`.

### 2. Start the container

```bash
scripts/podman.sh start
```

This starts a long-running container named `mini-vio` in the background with these mounts:

| Host | Container | Mode |
|---|---|---|
| this repo | `/workspace` | read-write |
| `~/Documents/datasets/EuRoC` | `/data/euroc` | read-only |

To use a different dataset location, set `EUROC_DIR=/path/to/EuRoC` when you run `start`. The container runs with
`--userns=keep-id`, so build outputs in `/workspace` are owned by your host user.

### 3. Enter (and re-enter) the container

```bash
scripts/podman.sh enter
```

This opens a bash shell in `/workspace`. Leaving the shell with `exit` or Ctrl+D does not stop the container. Run
`scripts/podman.sh enter` again to get back in, from as many terminals as you like. Build directories and shell history
inside `/workspace` are kept between sessions.

After a host reboot the container is stopped but not deleted. `scripts/podman.sh start` starts it again.

### 4. Stop the container

```bash
scripts/podman.sh stop
```

This stops and removes the container. Nothing in `/workspace` is lost, because it is the repo on the host. Anything
installed outside `/workspace` (for example an extra `apt install`) is lost. Put permanent dependencies in the
Containerfile instead.

### Build and test

One command configures, builds and runs every test:

```bash
scripts/podman.sh run scripts/build_and_test.sh    # from the host
scripts/build_and_test.sh                          # inside the container
```

The build goes to `build/` (default type `RelWithDebInfo`; pass `-DCMAKE_BUILD_TYPE=Debug` to change it).
`tests/dependencies_test.cc` checks that Eigen, OpenCV, yaml-cpp, GTSAM, Ceres and the Rerun SDK all compile, link and
run. If it fails, fix the container before debugging mini-VIO. `build/compile_commands.json` is generated for editors.

In CMake, link a new target against `mini_vio_deps`. It carries C++20, the warning flags (`-Wall -Wextra -Wpedantic`)
and every dependency.

### Editing inside the container (OpenIDEA / Dev Containers)

`.devcontainer.json` lets an editor that speaks the Dev Containers spec (OpenIDEA, VS Code) open the repo inside the
container, so clangd sees GTSAM, OpenCV, Eigen and the other headers. On the host they are missing, and the editor
shows false "file not found" errors.

- It reuses `localhost/mini-vio:dev`; run `scripts/podman.sh build` first.
- Same mounts as `scripts/podman.sh` (`/workspace`, `/data/euroc` read-only) and `--userns=keep-id`, so files stay
  owned by your host user.
- After the container starts, it configures CMake so `build/compile_commands.json` exists. clangd 22.1.1 is in the
  image (the newest on PyPI).
- `docker` on this host is podman-docker, so the Dev Containers CLI drives Podman. If the editor cannot reach it, start
  the socket with `systemctl --user enable --now podman.socket`.

### Code style: format and lint

Google C++ style with a 120-column limit. `.clang-format` defines the layout, `.clang-tidy` the lint rules (Google
naming, with an exception for math names such as `T_W_B` and `R1`). Files are `.h` / `.cc`.

```bash
git config core.hooksPath scripts/git-hooks          # once per clone: enable the pre-commit hook
scripts/format.sh                                    # format all C++ files in place
scripts/format.sh --check                            # only report
scripts/podman.sh run scripts/lint.sh                # clang-tidy on all .cc files
```

The pre-commit hook formats the staged C++ files (fully staged ones are fixed and re-staged; partially staged ones are
only checked) and runs clang-tidy on the staged `.cc` files. Any finding blocks the commit; `git commit --no-verify`
skips it. The hook runs the tools through `scripts/podman.sh run`, so the image must be built.

The image pins clang-format to the same version as the host (23.1.5), so the editor's format-on-save and the hook
agree. If the host's clang-format is upgraded, update `CLANG_FORMAT_VERSION` in the Containerfile too.

### One-off commands

To run a single command without the long-running container, use `run`. It uses a throwaway container:

```bash
scripts/podman.sh run cmake -B build -G Ninja
scripts/podman.sh run cmake --build build
```

### Plain Podman equivalents

If you prefer not to use the script:

```bash
podman build -t localhost/mini-vio:dev -f Containerfile .

podman run -d --name mini-vio --userns=keep-id \
    -v "$PWD:/workspace" -v "$HOME/Documents/datasets/EuRoC:/data/euroc:ro" \
    -w /workspace localhost/mini-vio:dev sleep infinity

podman exec -it mini-vio bash      # enter / re-enter
podman start mini-vio              # after a reboot
podman rm -f mini-vio              # stop and remove
```

### Viewing Rerun recordings

The program writes Rerun recordings (`.rrd`) to disk inside the container. Open them on the host with a viewer of
the same version as the SDK:

```bash
cargo binstall rerun-cli@0.43.2      # installs the viewer to ~/.cargo/bin
rerun path/to/recording.rrd
```

`rerun --version` must report the same version as the SDK in the container (0.43.2). Without Rust,
`pip install rerun-sdk==0.43.2` installs the same viewer.
