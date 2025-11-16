# CLAUDE.md - AI Assistant Guide for Autoware

This document provides comprehensive guidance for AI assistants (like Claude) working with the Autoware codebase. It covers repository structure, development workflows, coding conventions, and best practices.

## Table of Contents

- [Repository Overview](#repository-overview)
- [Repository Structure](#repository-structure)
- [Technology Stack](#technology-stack)
- [Development Setup](#development-setup)
- [Build Process](#build-process)
- [Code Conventions](#code-conventions)
- [Testing](#testing)
- [CI/CD](#cicd)
- [Common Development Tasks](#common-development-tasks)
- [Important Files and Locations](#important-files-and-locations)
- [AI Assistant Guidelines](#ai-assistant-guidelines)

---

## Repository Overview

**Autoware** is the world's leading open-source software stack for autonomous driving, built on **ROS 2 (Robot Operating System 2)**. This repository is a **meta-repository** that uses `.repos` files (vcstool format) to manage dependencies from multiple upstream repositories.

### Key Characteristics

- **Build System**: ROS 2 with colcon (ament_cmake and ament_python)
- **Primary Language**: C++17 with Python 3 for tooling
- **ROS Distribution**: ROS 2 Humble
- **License**: Apache 2.0
- **Architecture**: Modular, plugin-based architecture with domain separation

### Related Repositories

- **autoware_core**: High-quality, stable ROS packages
- **autoware_universe**: Experimental, cutting-edge packages
- **autoware_launch**: Launch configurations and parameter files
- **autoware_msgs**: Message definitions
- **autoware-documentation**: User and developer documentation

---

## Repository Structure

### Top-Level Directories

```
autoware_harry/
├── src/                  # Source code (imported via .repos files)
│   ├── core/            # Core stable packages
│   ├── universe/        # Experimental packages
│   ├── launcher/        # Launch configurations
│   ├── sensor_component/# Sensor drivers
│   └── middleware/      # Middleware components
├── ansible/             # Ansible roles for environment setup
├── docker/              # Docker configurations and build scripts
├── .github/             # GitHub workflows and actions
├── .devcontainer/       # VSCode devcontainer configurations
├── *.repos              # vcstool repository manifests
├── setup-dev-env.sh     # Primary setup script
└── [config files]       # Various linting/formatting configs
```

### Source Code Organization

#### `/src/core/` - Core Packages

Organized by domain and purpose:

- **autoware_msgs/**: ROS message definitions (control, planning, perception, sensing, vehicle)
- **autoware_adapi_msgs/**: AD API message definitions
- **autoware_internal_msgs/**: Internal communication messages
- **autoware_cmake/**: Build system utilities and macros
- **autoware_utils/**: Common utility packages
- **autoware_lanelet2_extension/**: Lanelet2 map format extensions
- **autoware_rviz_plugins/**: Visualization plugins for RViz
- **autoware_core/**: Main autonomous driving modules
  - `common/`: Shared utilities (interpolation, Kalman filter, motion utils, etc.)
  - `control/`: Vehicle control algorithms
  - `localization/`: Localization modules (EKF, gyro odometry, etc.)
  - `map/`: Map loading and processing
  - `perception/`: Object detection and tracking
  - `planning/`: Path and motion planning
  - `sensing/`: Sensor data processing
  - `vehicle/`: Vehicle interface
  - `testing/`: Testing utilities and frameworks

#### `/src/universe/` - Universe Packages

- **autoware_universe/**: Experimental and extended functionality
  - Similar domain organization to core (common, control, evaluator, localization, etc.)
  - Additional domains: simulator, system, visualization, tools
- **external/**: External dependencies (bevdet, cuda_blackboard, eagleye, glog, etc.)

#### `/src/launcher/` - Launch System

- **autoware_launch/**: Launch files, parameter configurations, vehicle/sensor kit definitions

#### `/src/sensor_component/` - Sensor Drivers

- **nebula/**: Lidar driver
- **ros2_socketcan/**: CAN bus driver
- **transport_drivers/**: Network transport drivers
- **sensor_component_description/**: Sensor descriptions

#### `/src/middleware/` - Middleware

- **agnocast/**: Agnocast middleware for communication

---

## Technology Stack

### Programming Languages

- **C++17**: Primary language for performance-critical components
  - Standard enforced via CMake (`CMAKE_CXX_STANDARD 17`)
  - Compiled with `-Wall -Wextra -Wpedantic -Werror`
- **Python 3**: Used for tools, scripts, and some ROS 2 nodes
- **Shell Scripts**: Build and setup automation

### ROS 2 Framework

- **Distribution**: ROS 2 Humble
- **Build System**:
  - `colcon` - Build tool
  - `ament_cmake` - C++ package build system
  - `ament_python` - Python package build system
- **Custom Macros**: `autoware_package()` from `/src/core/autoware_cmake/`
  - Standardizes C++17, compile flags, dependency discovery
  - Auto-finds dependencies via `ament_auto_find_build_dependencies()`

### Build and Development Tools

- **vcstool**: Repository dependency management (`.repos` files)
- **rosdep**: System dependency installation
- **CMake**: C++ build configuration
- **pytest**: Python testing
- **gtest/gmock**: C++ testing frameworks

### Infrastructure

- **Docker**: Multi-stage builds with development and runtime images
- **Ansible**: Environment setup and configuration management
- **GitHub Actions**: CI/CD automation
- **pre-commit**: Code quality enforcement

---

## Development Setup

### Initial Setup

**1. Clone and import dependencies:**

```bash
# Clone the repository
git clone https://github.com/autowarefoundation/autoware.git
cd autoware

# Import dependencies using vcstool
vcs import src < autoware.repos
```

**2. Environment setup:**

```bash
# Run the setup script (uses Ansible)
./setup-dev-env.sh -y

# Options:
# -y              : Non-interactive mode
# -v              : Verbose output
# --no-nvidia     : Skip NVIDIA tools
# --no-cuda-drivers : Skip CUDA drivers
# --runtime       : Runtime-only packages
# --data-dir DIR  : Set data directory
```

**3. Install ROS dependencies:**

```bash
# Install all dependencies from package.xml files
rosdep install --from-paths src --ignore-src -r -y
```

### Docker Development

**Development containers** (with build tools):
```bash
docker pull ghcr.io/autowarefoundation/autoware:universe-devel
docker pull ghcr.io/autowarefoundation/autoware:universe-devel-cuda
```

**Runtime containers**:
```bash
docker pull ghcr.io/autowarefoundation/autoware:universe
docker pull ghcr.io/autowarefoundation/autoware:universe-cuda
```

### VSCode DevContainer

Pre-configured devcontainer settings are available in `.devcontainer/`. Open the repository in VSCode and select "Reopen in Container".

---

## Build Process

### Standard Build Workflow

```bash
# Clean build (recommended for first build)
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Incremental build (faster for development)
colcon build --symlink-install --packages-select <package_name>

# Build with debug symbols
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug

# Source the workspace
source install/setup.bash
```

### Build System Details

**The `autoware_package()` Macro:**

Located in `/src/core/autoware_cmake/autoware_cmake/cmake/autoware_package.cmake`, this macro:

- Sets C++17 standard
- Adds compile flags: `-Wall -Wextra -Wpedantic -Werror`
- Handles ROS distro-specific macros
- Auto-discovers dependencies from `package.xml`

**Example CMakeLists.txt:**

```cmake
cmake_minimum_required(VERSION 3.14)
project(my_autoware_package)

find_package(autoware_cmake REQUIRED)
autoware_package()

# Dependencies are auto-found from package.xml
# ament_auto_find_build_dependencies()

ament_auto_add_library(my_library SHARED
  src/my_source.cpp
)

ament_auto_add_executable(my_node
  src/my_node.cpp
)

# Testing
if(BUILD_TESTING)
  ament_add_ros_isolated_gtest(test_my_package
    test/test_my_package.cpp
  )
endif()

ament_auto_package()
```

### Dependency Management

**Repository manifests (`.repos` files):**

- `autoware.repos`: Main dependencies
- `autoware-nightly.repos`: Nightly/development versions
- `simulator.repos`: Simulator components
- `tools.repos`: Development tools

**Import dependencies:**

```bash
vcs import src < autoware.repos
```

---

## Code Conventions

### C++ Conventions

#### Formatting (.clang-format)

- **Style Base**: Google
- **Line Length**: 100 characters
- **Pointer Alignment**: Middle (`int * ptr`)
- **Indentation**: 2 spaces
- **Brace Wrapping**: After class, function, namespace, struct
- **Include Order**: Local → Package → Messages → Boost → C system → C++ system

**Format code:**
```bash
clang-format -i <file>.cpp
```

#### Linting (.clang-tidy)

**Key checks enabled:**
- `bugprone-*`: Bug-prone code patterns
- `cppcoreguidelines-*`: C++ Core Guidelines
- `modernize-*`: Modern C++ idioms
- `performance-*`: Performance optimizations
- `readability-*`: Code readability

**Naming conventions:**
- Namespaces: `lower_case`
- Classes/Structs: `CamelCase`
- Functions: `lower_case`
- Variables: `lower_case`
- Private Members: Suffix with `_`
- Global Constants: Prefix with `g_`

**Run clang-tidy:**
```bash
clang-tidy <file>.cpp -- -std=c++17
```

#### Additional C++ Conventions

- **Line length**: 100 characters (enforced by cpplint)
- **Warnings as errors**: All warnings must be fixed
- **Include guards**: Use `#pragma once`
- **Const correctness**: Prefer `const` where applicable
- **Smart pointers**: Use `std::shared_ptr`, `std::unique_ptr`
- **Auto keyword**: Use when type is obvious from context

### Python Conventions

**Formatting:**
- **Tool**: black
- **Line length**: 100 characters
- **Import sorting**: isort (force single line imports)

**Linting:**
- **Tool**: flake8
- **Max line length**: 100

**Format code:**
```bash
black <file>.py
isort <file>.py
```

### YAML Conventions

- **Line length**: Enforced by Prettier, not yamllint
- **Indentation**: 2 spaces
- **No document start markers**: `---` not required
- **Truthy values**: `on` allowed for GitHub Actions

### Package Naming Conventions

**Pattern**: `autoware_{domain}_{functionality}`

**Examples:**
- `autoware_motion_utils`
- `autoware_behavior_velocity_planner`
- `autoware_ekf_localizer`
- `autoware_perception_msgs`

**Message packages**: `autoware_{scope}_msgs`
- `autoware_adapi_v1_msgs`
- `autoware_control_msgs`
- `autoware_planning_msgs`

### Package Structure

**C++ Package (ament_cmake):**
```
my_autoware_package/
├── CMakeLists.txt          # Build configuration
├── package.xml             # Package metadata and dependencies
├── include/                # Public headers (optional)
│   └── my_autoware_package/
├── src/                    # Implementation files
│   ├── my_node.cpp
│   └── my_library.cpp
├── test/                   # Unit tests
│   └── test_my_package.cpp
├── config/                 # Configuration files
│   └── params.yaml
├── launch/                 # Launch files
│   └── my_launch.xml
├── schema/                 # JSON schemas for parameters
│   └── params.schema.json
├── README.md               # Package documentation
└── CHANGELOG.rst           # Version history
```

**Python Package (ament_python):**
```
my_autoware_py_package/
├── setup.py                # Package setup
├── setup.cfg               # Additional config
├── package.xml             # Package metadata
├── my_autoware_py_package/ # Python module
│   ├── __init__.py
│   └── my_node.py
├── test/                   # Tests
│   └── test_my_module.py
├── launch/                 # Launch files
├── config/                 # Configuration files
├── resource/               # Resource marker
│   └── my_autoware_py_package
└── README.md
```

---

## Testing

### Unit Testing

**C++ Testing (gtest/gmock):**

```cmake
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_ros_isolated_gtest(test_my_package
    test/test_my_class.cpp
  )

  target_link_libraries(test_my_package
    my_library
  )
endif()
```

**Python Testing (pytest):**

```python
# test/test_my_module.py
import pytest
from my_autoware_py_package import my_module

def test_my_function():
    result = my_module.my_function()
    assert result == expected_value
```

### Running Tests

```bash
# Build with tests
colcon build --cmake-args -DBUILD_TESTING=ON

# Run all tests
colcon test

# Run tests for specific package
colcon test --packages-select my_autoware_package

# View test results
colcon test-result --verbose
```

### Integration Testing

- **Scenario tests**: End-to-end testing with planning simulator
- **Launch tests**: ROS 2 launch file testing
- Located in `.github/workflows/scenario-test.yaml`

### Code Quality Checks

**Pre-commit hooks** (run before commit):
```bash
# Install pre-commit hooks
pre-commit install

# Run manually on all files
pre-commit run --all-files

# Run on staged files
pre-commit run
```

**Checks include:**
- Code formatting (clang-format, black, prettier)
- Linting (clang-tidy, cpplint, flake8, yamllint)
- JSON/XML/YAML validation
- Markdown linting
- Shell script checking (shellcheck, shfmt)
- Spell checking
- Package dependency verification

---

## CI/CD

### GitHub Workflows

**Main workflows** (`.github/workflows/`):

1. **health-check.yaml**
   - Builds Docker images (amd64 and arm64)
   - Runs daily or on PR with specific label
   - Tests stable and nightly builds

2. **pre-commit.yaml**
   - Runs pre-commit hooks on PRs
   - Uses `autowarefoundation/autoware-github-actions/pre-commit@v1`
   - Only for private repositories (public repos use pre-commit.ci)

3. **scenario-test.yaml**
   - End-to-end planning simulator tests
   - Downloads maps and scenarios
   - Uses self-hosted runners

4. **docker-build-and-push.yaml**
   - Multi-stage Docker image building
   - Publishes to GitHub Container Registry (ghcr.io)
   - Separate images for components and all-in-one

5. **semantic-pull-request.yaml**
   - Validates PR titles follow conventional commit format

6. **spell-check-*.yaml**
   - Daily and differential spell checking

7. **sync-files.yaml**
   - Synchronizes common files from template repository

### Quality Gates

**All changes must pass:**
- Pre-commit hooks (formatting, linting)
- Build verification (colcon build succeeds)
- Unit tests (all tests pass)
- Static analysis (clang-tidy, cppcheck)
- Code coverage (Codecov integration)
- DCO (Developer Certificate of Origin) check

### Multi-Architecture Support

- **amd64** (x86_64): Primary development platform
- **arm64** (aarch64): Supported for embedded platforms
- Platform-specific environment files: `amd64.env`, `arm64.env`

---

## Common Development Tasks

### Creating a New Package

**C++ Package:**

```bash
cd src/core/autoware_core/common/  # or appropriate location
ros2 pkg create --build-type ament_cmake \
  --dependencies rclcpp std_msgs \
  --node-name my_node \
  my_autoware_package
```

Then update `CMakeLists.txt` to use `autoware_package()`.

**Python Package:**

```bash
cd src/core/autoware_core/common/
ros2 pkg create --build-type ament_python \
  --dependencies rclpy std_msgs \
  --node-name my_node \
  my_autoware_py_package
```

### Modifying Existing Code

1. **Find the relevant package:**
   ```bash
   # Search for functionality
   grep -r "function_name" src/

   # Find package by name
   find src/ -name "package_name"
   ```

2. **Make changes** following code conventions

3. **Build and test:**
   ```bash
   colcon build --packages-select my_autoware_package
   colcon test --packages-select my_autoware_package
   ```

4. **Run pre-commit checks:**
   ```bash
   pre-commit run --files path/to/modified/file.cpp
   ```

### Adding Dependencies

**1. Update `package.xml`:**

```xml
<depend>new_dependency</depend>
<!-- or -->
<build_depend>new_build_dependency</build_depend>
<exec_depend>new_runtime_dependency</exec_depend>
```

**2. For ament_cmake packages**, dependencies are auto-discovered by `autoware_package()`

**3. For system dependencies:**

```bash
# Install via rosdep
rosdep install --from-paths src/my_package --ignore-src -r -y
```

### Debugging

**1. Build with debug symbols:**
```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

**2. Use GDB:**
```bash
gdb --args ros2 run my_package my_node
```

**3. Enable logging:**
```bash
ros2 run my_package my_node --ros-args --log-level debug
```

**4. Use RViz for visualization:**
```bash
ros2 run rviz2 rviz2
```

---

## Important Files and Locations

### Configuration Files

| File | Purpose |
|------|---------|
| `.clang-format` | C++ code formatting rules |
| `.clang-tidy` | C++ static analysis configuration |
| `CPPLINT.cfg` | Cpplint configuration |
| `.pre-commit-config.yaml` | Pre-commit hook definitions |
| `.prettierrc.yaml` | JavaScript/YAML/JSON formatting |
| `.yamllint.yaml` | YAML linting rules |
| `.markdownlint.yaml` | Markdown linting rules |
| `setup.cfg` | Python tool configuration (flake8, isort) |

### Repository Manifests

| File | Purpose |
|------|---------|
| `autoware.repos` | Main stable dependencies |
| `autoware-nightly.repos` | Nightly/development versions |
| `simulator.repos` | Simulator components |
| `tools.repos` | Development tools |

### Build Scripts

| File | Purpose |
|------|---------|
| `setup-dev-env.sh` | Primary environment setup script |
| `amd64.env` | amd64 platform environment variables |
| `arm64.env` | arm64 platform environment variables |

### Documentation

| Location | Purpose |
|----------|---------|
| `README.md` | Repository overview |
| `CONTRIBUTING.md` | Contribution guidelines (links to docs) |
| `CODE_OF_CONDUCT.md` | Community code of conduct |
| `DISCLAIMER.md` | Legal disclaimers |
| Package-level `README.md` | Per-package documentation |

### Important Directories

| Directory | Purpose |
|-----------|---------|
| `/src/core/autoware_cmake/` | Build system macros and utilities |
| `/src/core/autoware_msgs/` | Message definitions |
| `/src/core/autoware_core/common/` | Common utility libraries |
| `/ansible/roles/` | Ansible setup roles |
| `/docker/` | Docker build configurations |
| `/.github/workflows/` | CI/CD pipeline definitions |

---

## AI Assistant Guidelines

### When Working with This Codebase

1. **Always Use `autoware_package()` Macro**
   - For C++ packages, use `autoware_package()` instead of manual `project()` setup
   - This ensures consistent build configuration and dependency discovery

2. **Follow Strict Code Quality Standards**
   - Code must pass clang-format, clang-tidy, and pre-commit hooks
   - Run `pre-commit run --all-files` before suggesting changes complete
   - All compiler warnings are treated as errors (`-Werror`)

3. **Respect Domain Organization**
   - Place code in appropriate domain directories (common, control, localization, etc.)
   - Keep core packages stable; experimental work goes in universe

4. **Use ROS 2 Best Practices**
   - Prefer `ament_auto_*` macros for dependency management
   - Use component-based node design where appropriate
   - Follow ROS 2 parameter declaration patterns

5. **Testing is Mandatory**
   - Add unit tests for all new functionality
   - Ensure existing tests still pass after changes
   - Use `ament_add_ros_isolated_gtest` for C++ tests

6. **Documentation**
   - Update package README.md when adding features
   - Add JSON schemas for configurable parameters (in `schema/` directory)
   - Include docstrings/comments for complex logic

7. **Dependencies**
   - Minimize external dependencies
   - All dependencies must be in `package.xml`
   - Check if functionality exists in existing autoware utility packages before adding new dependencies

8. **Performance Considerations**
   - This is real-time autonomous driving software
   - Avoid unnecessary copies, dynamic allocations in hot paths
   - Profile performance-critical code
   - Consider multi-threading implications

9. **Safety-Critical Code**
   - This code controls vehicles - prioritize safety
   - Validate inputs thoroughly
   - Handle error conditions gracefully
   - Add appropriate safety checks and assertions

10. **Build Verification**
    - Always verify code builds with: `colcon build --packages-select <package>`
    - Test in both Debug and Release modes
    - Check for warnings (treated as errors)

### Useful Commands for AI Assistants

```bash
# Search for functionality
grep -r "pattern" src/

# Find package locations
find src/ -name "package_name"

# Build specific package
colcon build --packages-select package_name

# Run tests for package
colcon test --packages-select package_name

# Check code format (dry-run)
clang-format --dry-run --Werror path/to/file.cpp

# Run pre-commit on specific files
pre-commit run --files path/to/file

# View package dependencies
rosdep keys --from-paths src/package_name --ignore-src

# List all packages in workspace
colcon list

# Search for message definitions
find src/ -name "*.msg" -o -name "*.srv" -o -name "*.action"
```

### Common Pitfalls to Avoid

1. **Not using `autoware_package()`** - Leads to inconsistent builds
2. **Ignoring pre-commit hooks** - Code will fail CI
3. **Hardcoding paths** - Use ROS 2 parameters and package resources
4. **Not testing edge cases** - Critical for safety
5. **Breaking API compatibility** - Check downstream dependencies
6. **Incorrect include guards** - Use `#pragma once`
7. **Memory leaks** - Use RAII and smart pointers
8. **Thread safety issues** - Protect shared state
9. **Not checking return values** - Handle errors explicitly
10. **Poor variable naming** - Follow naming conventions strictly

### Reference Documentation

- **Autoware Documentation**: https://autowarefoundation.github.io/autoware-documentation/main/
- **ROS 2 Documentation**: https://docs.ros.org/en/humble/
- **Contribution Guidelines**: https://autowarefoundation.github.io/autoware-documentation/main/contributing/
- **Autoware Discord**: https://discord.gg/Q94UsPvReQ

---

## Summary

This Autoware repository is a well-organized, professional autonomous driving software stack with:

- **Strict code quality standards** enforced through automated tooling
- **Comprehensive testing** requirements at multiple levels
- **Clear organizational structure** with domain separation
- **Modern C++17 and ROS 2** best practices
- **Multi-architecture support** (amd64, arm64)
- **Extensive CI/CD** automation

When contributing or making changes, always prioritize **safety, quality, and consistency** with existing patterns. The build system, linting tools, and CI/CD pipeline are designed to maintain high standards - work with them, not against them.

For questions or issues, refer to the official documentation or join the Autoware community on Discord.

---

**Document Version**: 1.0
**Last Updated**: 2025-11-16
**Autoware Version**: Based on autoware.repos stable versions
