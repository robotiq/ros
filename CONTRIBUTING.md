# Contributing

Thanks for your interest in improving this repository. Robotiq maintains these
packages, and outside contributions are welcome — bug reports, fixes and
features all land the same way: as a pull request. Robotiq members have write
access and push a branch to this repository; everyone else works from a fork.
Please do not fork if you are a member: a fork PR cannot be based on another
in-repo branch, so stacked changes end up showing each other's commits.

## Before you start

- **Small fix or feature?** Open the PR directly.
- **Anything larger** — a new feature, a change to the launch or action surface,
  a new package — please [open an issue](https://github.com/robotiq/ros/issues)
  first so we can agree on the shape before you spend the time. Some constraints
  are not obvious from the code (see [Compatibility](#compatibility) below).
- **Bug report?** [Open an issue](https://github.com/robotiq/ros/issues) — the
  bug template asks for the ROS distro, hardware, exact launch command and
  bringup output that make a driver problem placeable.

## Getting the source

Both the grippers and tactile_sensors SDKs arrive as submodules, so clone recursively:

```bash
git clone --recurse-submodules https://github.com/robotiq/ros.git
```

Replace `robotiq` with your GitHub user if you are working from a fork.

If you already cloned without them: `git submodule update --init`.

The container is the fastest way to *run* the workspace, but its default image
is a slim runtime stage — no compiler, no colcon, tests not built — so it is
for trying the drivers, not developing them:

```bash
cd ros
./docker/run.sh gripper          # or: sensor, both
```

For development, work in an ordinary ROS 2 workspace (or build the
full-toolchain `builder` image — see [Docker](README.md#docker)): clone into
`src/` and follow the **Existing ROS 2 workspace** steps in the
[README](README.md#migrating-from-pickniks-ros2_robotiq_gripper), skipping the
`rm -rf` lines unless you are actually migrating off the PickNik packages.
`rosdep install` resolves every dependency, `libserialport` included.

## Formatting and linting

One root [`.pre-commit-config.yaml`](.pre-commit-config.yaml) is the formatting
authority — its allow-list covers `grippers/`, `robotiq_tsf/`, `docker/`,
`.github/` and the root-level files: clang-format 19 for C++, black and flake8
for Python, codespell, and the usual whitespace and YAML hooks. Install the
hook once and it runs on every commit:

```bash
pip install pre-commit
pre-commit install
pre-commit run -a        # or run it across the repo by hand
```

CI runs the same hooks on PRs touching those trees
([`ci-format.yml`](.github/workflows/ci-format.yml)), so skipping them locally
just means the PR comes back red on formatting. The `extern/` submodules are
deliberately out of scope — don't reformat them.

`ament_cpplint`, `ament_copyright` and `ament_lint_cmake` also run on the C++
packages. Where cpplint and clang-format disagree, clang-format wins and the
cpplint check is suppressed in CI — don't reformat code to satisfy cpplint.

## Building and testing

```bash
source /opt/ros/jazzy/setup.bash    # or humble / lyrical
colcon build
colcon test
colcon test-result --verbose
```

Scope to one package with `--packages-select <package>` on both `build` and
`test`; [Testing](README.md#testing) in the README covers running a single test
binary for `--gtest_filter`.

**New code needs tests.** Unit tests live in each package's `test/` (or
`tests/`) directory and must run without hardware. A behavioural change with no
test change is the most common reason a PR needs another round.

`robotiq_hardware_tests` is the exception: it registers no ctest tests — it
builds `full_test`, a manual check run against a real gripper with
`ros2 run robotiq_hardware_tests full_test`. If your change touches the
driver's interaction with hardware, say in the PR what you tested it on —
gripper model, firmware version and how it behaved.

## Compatibility

Two constraints shape most reviews here:

**All three LTS distros build from one branch.** Humble, Jazzy and Lyrical each
get a CI job, and all three gate the merge. Rolling is built too but does not
block. Code that compiles only on the newest distro will fail the matrix — the
`__has_include` feature detection in `robotiq_driver`'s
`ros2_control_compat.hpp` shows the pattern. Kilted and other non-LTS releases
are not targeted.

The one place the distros genuinely differ is the gripper controller — see
[Supported ROS 2 distros](README.md#supported-ros-2-distros). Keep changes to
controller configuration working on both sides of that split.

**Humble stays a drop-in replacement for PickNik's `humble` branch** (see
[Migrating from PickNik's ros2_robotiq_gripper](README.md#migrating-from-pickniks-ros2_robotiq_gripper)).
The names that migration relies on — packages, controllers, topics and actions,
the xacro macro arguments — are a compatibility surface, not implementation
detail. If your change needs to break one of them, raise it in an issue first;
the usual answer is to put the new behaviour behind a parameter or launch
argument that defaults to the old behaviour.

## Commits and pull requests

Commit messages follow [Conventional Commits](https://www.conventionalcommits.org/)
with the package as the scope, and `!` for a breaking change:

```
feat(robotiq_driver): expose object detection and effort as state interfaces
fix(robotiq_description): drop reference to non-existent update_rate config
fix(robotiq_driver)!: slave_address reads decimal as decimal
```

For the PR itself: describe what changes and why, keep it to one logical change
where you can, and note anything you could not test. Rationale belongs in the
commit message and PR description rather than in comments explaining what the
code used to do.

Two things to leave alone:

- **Versions.** The repository is versioned as a whole, and a pre-commit hook
  fails any commit whose `package.xml` versions drift from `VERSION` (see
  [Versioning](README.md#versioning)). Don't bump them in a PR — a maintainer
  does that when cutting a release.
- **The `extern/` submodules.** They track their own repositories and are pinned
  here deliberately. Changes to the gripper or tactile SDK go to
  [Robotiq/grippers](https://github.com/Robotiq/grippers) or
  [Robotiq/tactile_sensors](https://github.com/Robotiq/tactile_sensors); a
  maintainer moves the pin.

## Licensing and copyright

The repository is BSD-3-Clause. By contributing you agree your contribution is
released under that license.

New source files need the BSD-3-Clause header block — copy one from a neighbour
and put your own name or your employer's on the copyright line;
`ament_copyright` checks that it is there. When editing an existing file, leave
the copyright lines that are already in it alone: several files carry PickNik's
original header alongside Robotiq's, and both are kept.

## Review

Robotiq reviews and merges, and maintains the code afterwards. CI must be green
before merge; changes touching the package trees build and test on all three
supported distros. On a first-time contribution a
maintainer has to approve the workflow run, so CI may sit idle for a moment
before it starts — that's normal, not a problem with your PR.
