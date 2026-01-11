# Changelog

All notable changes to the RobotOps configuration will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

<!-- Versions below this line -->

## [0.4.14] - 2026-01-11

### Fixed

- **CMake Installation**: Added installation of defaults.hpp header file
  - defaults.hpp contains helper functions like CreateDefaultConfig()
  - Now properly installed to /opt/ros/jazzy/include/robotops/config/v1/
  - Integration test updated to verify defaults.hpp accessibility

- **Code Generator**: Fixed C++ defaults generation for repeated fields
  - Skip generating setters for repeated and map fields (not supported in C++)
  - Fixed string escaping to handle quotes in default values
  - Prevents invalid C++ code generation

## [0.4.13] - 2026-01-11

### Fixed

- **Integration Test**: Fixed test.cpp to use correct protobuf field names
  - Changed from `set_version()`/`version()` to `set_schema_version()`/`schema_version()`
  - Matches actual Config message schema definition
- **CI Validation**: Added libprotobuf-dev installation before running integration tests
  - Required for compiling test executables that include protobuf headers
  - Prevents "No such file or directory" errors for protobuf includes

## [0.4.12] - 2026-01-11

### Fixed

- **CMake Target Namespace**: Corrected exported target namespace from `robotops::` to `robotops-config::`
  - Target is now correctly named `robotops-config::robotops-config` instead of `robotops::robotops-config`
  - Fixes integration test failures and downstream package consumption
  - Matches expected CMake package naming conventions

## [0.4.11] - 2026-01-11

### Fixed

- **CMake Integration**: Fixed protobuf linking to use variable-based approach for compatibility
  - Changed from `protobuf::libprotobuf` to `${Protobuf_LIBRARIES}` for broader CMake version support
  - Added `${Protobuf_INCLUDE_DIRS}` to target include directories
  - Resolves "target not found" errors in ROS2 Jazzy environment
  - Compatible with both old and new FindProtobuf module versions

### Added

- **CI/CD Validation**: Added CMake integration tests to release workflow
  - Validates package installation before publishing to Cloudsmith
  - Tests CMake target existence and properties
  - Verifies protobuf linkage and header accessibility
  - Compiles and runs minimal test program using robotops-config
  - Prevents broken packages from reaching production

## [0.4.10] - 2026-01-11

### Fixed

- **CMake Integration**: Fixed protobuf linking to use variable-based approach for compatibility
  - Changed from `protobuf::libprotobuf` to `${Protobuf_LIBRARIES}` for broader CMake version support
  - Added `${Protobuf_INCLUDE_DIRS}` to target include directories
  - Resolves "target not found" errors in ROS2 Jazzy environment
  - Compatible with both old and new FindProtobuf module versions

### Added

- **CI/CD Validation**: Added CMake integration tests to release workflow
  - Validates package installation before publishing to Cloudsmith
  - Tests CMake target existence and properties
  - Verifies protobuf linkage and header accessibility
  - Compiles and runs minimal test program using robotops-config
  - Prevents broken packages from reaching production

## [0.4.9] - 2026-01-11

### Fixed

- **CMake Config**: Fixed CMAKE_CURRENT_DIR to CMAKE_CURRENT_LIST_DIR in robotops-configConfig.cmake.in template
  - Resolves "file not found" error when consuming robotops-config package
  - CMAKE_CURRENT_DIR is not a valid CMake variable and expands to empty string
  - CMAKE_CURRENT_LIST_DIR correctly points to the directory containing the config file

## [0.4.8] - 2026-01-11

### Fixed

- **Release Workflows**: Fixed architecture-specific binary installation
  - Install correct just binary for each architecture (x86_64 for amd64, aarch64 for arm64)
  - Prevents "cannot execute binary file" errors on arm64 runners
  - Run arch-independent steps (Rust publishing, YAML upload) only on amd64
  - Applied to both release.yml and release-dev.yml workflows

## [0.4.7] - 2026-01-11

### Changed

- **Release Workflows**: Added architecture matrix for multi-platform builds
  - Build Debian packages for both amd64 and arm64 architectures
  - Use native runners: ubuntu-24.04 for amd64, ubuntu-24.04-arm for arm64
  - Each architecture produces separate .deb packages for Cloudsmith
  - Satisfies downstream dependency requirements for multiple platforms

## [0.4.6] - 2026-01-11

### Fixed

- **ROS2 Package Build**: Fixed CMakeLists.txt template for correct Debian package generation
  - Added VERSION to project() call - required by write_basic_package_version_file()
  - Fixed C++ source file paths to match buf-generated proto/ subdirectory structure
  - Fixed include directories to match protobuf-generated include paths
  - Enabled version template substitution in CMakeLists.txt generation
- **CI Workflow**: Fixed buf breaking check to specify proto/ subdirectory when comparing against remote branches
  - Added subdir=proto parameter to --against URL for proper directory comparison

### Changed

- **Build System**: Refactored ROS2 package file generation
  - Moved package.xml and CMakeLists.txt generation from repository root to generated/ros2/
  - Templates remain tracked in tools/templates/ros2/ for visibility
  - Files are copied to root only during Docker build in CI/CD workflows
  - Removed debian/ directory from git tracking (generated by bloom-generate)
  - Updated .gitignore to exclude debian/ and .obj-*/ build artifacts
  - No generated files committed to repository root anymore

## [0.4.5] - 2026-01-11

### Fixed

- **GitHub Actions Release Workflow**: Fixed Debian package build failure in release workflows
  - Added proper Docker volume mount to make generated protobuf code accessible during package build
  - Container now mounts workspace to both `/ws/src/robotops-config` (source) and `/output` (artifacts)
  - Resolves "No packages found in path" error from bloom-generate
  - Applied fix to both production (release.yml) and development (release-dev.yml) workflows

## [0.4.4] - 2026-01-11

### Fixed

- **Protobuf Version Compatibility & CI/CD Consistency**: Generate protobuf code inside ros:jazzy Docker container to ensure protoc 3.21.12 compatibility and unified build environment
  - Local development, CI/CD workflows, and Docker builds now use consistent protoc 3.21.12 (from ROS2 Jazzy)
  - Resolves generated C++ code incompatibility with ROS2 Jazzy's protobuf version
  - Updated `justfile` `generate` command to run buf inside Docker container
  - Updated all GitHub Actions workflows (release.yml, release-dev.yml, ci.yml) to use just commands for full reproducibility locally:
    - `just lint` for protocol buffer linting and validation
    - `just generate` for code generation (all languages: Rust, C++, Python)
    - `just validate` for comprehensive validation tasks
    - All CI jobs use same justfile commands as local development, enabling single-command reproduction
  - All CI jobs build and use the same Docker container for consistency
  - Added buf v1.28.1 to Dockerfile with automatic architecture detection (x86_64/aarch64)
  - Updated buf.yaml with lint and breaking change rules, configured for proto directory structure
  - Updated buf.gen.yaml to v1 format (compatible with buf v1.28.1)
  - Fixed Python code generation to properly create `__init__.py` files in both proto/ subdirectory and root paths
  - Updated CI workflow to specify proto directory for buf lint and breaking checks

### Changed

- Dockerfile base image: Changed from ubuntu:24.04 to ros:jazzy to include ROS2 dependencies and correct protobuf version

## [0.4.3] - 2026-01-10

### Changed

- Docker builds 

## [0.4.2] - 2026-01-10

### Changed

- Docker dependencies updated

## [0.4.1] - 2026-01-10

### Changed

- Github Actions CD improvements

## [0.4.0] - 2026-01-10

### Added

- **ROS2 Debian Package Support**: robotops-config now distributes as a ROS2 system package
  - Created `package.xml` for ROS2 package metadata (format 3)
  - Created `CMakeLists.txt` for ament_cmake build system integration
  - Compiles protobuf library from `config.pb.cc` into installable shared library
  - Exports CMake targets (`robotops-config::robotops-config`) for downstream consumer use
  - Properly installs headers to `/include/robotops` for C++ consumers
  - Enables standard ROS2 workflow: `rosdep install` + `colcon build`

### Changed

- **Build System Migration**: Eliminated Conan dependency for C++ package distribution
  - Removed Conan-specific build files and publishing logic from CI/CD
  - Protobuf now sourced from Ubuntu package repositories (no SSL certificate issues)
  - Release workflow now builds and publishes Debian source packages via Cloudsmith
  - Customers no longer need to install Conan or conancenter
- **Code Generation Pipeline**: ROS2 files now template-based
  - Added `tools/templates/ros2/` directory with package.xml and CMakeLists.txt templates
  - Version automatically substituted from VERSION file during code generation
  - Generated files properly gitignored (not committed to repository)
  - Integrates with existing `just generate` and `just bump-version` workflows
- **Release Workflow**: Updated for Debian distribution
  - Changed from multi-architecture Conan builds to single source Debian package
  - Uses bloom to generate package metadata
  - Uses dpkg-buildpackage for building
  - Publishes to Cloudsmith as "deb" format (architecture-independent source package)

### Removed

- **Conan Integration**: Removed all Conan-related code
  - Deleted `tools/templates/cpp/conanfile.py` template
  - Removed `ConanfileGenerator` class from code generation pipeline
  - Removed C++ Conan package publishing (now distributed as ROS2 Debian package)
  - Removed multi-architecture Conan builds from release workflow

## [0.3.13] - 2026-01-10

### Changed

- Made protobuf a build dependency only

## [0.3.12] - 2026-01-09

### Changed

- Protobuf version 3.21.12

## [0.3.11] - 2026-01-09

### Changed

- Build dependencies from source

## [0.3.10] - 2026-01-09

### Changed

- Upgraded protobuf version to 6.32.1

## [0.3.9] - 2026-01-09

### Changed

- Matrix build creates armv8 and x86_64 Conan builds
- Platform independent build for Cargo

## [0.3.8] - 2026-01-09

### Changed

- Corrected enums for architectures of builds 
- Conan builds on both armv8 and x86_64

## [0.3.7] - 2026-01-09

### Changed

- Conan builds now build and publish on both architectures

## [0.3.6] - 2026-01-09

### Changed

- Rust bindings with Serde support

## [0.3.5] - 2026-01-07

### Changed

- Conan user/channel removed

## [0.3.4] - 2026-01-07

### Changed

- Set up Conan profile in CD

## [0.3.3] - 2026-01-07

### Changed

- Fixed Cloudsmith upload syntax

## [0.3.2] - 2026-01-07

### Changed

- Fixed CD permissions issue

## [0.3.1] - 2026-01-07

### Changed

- Fixed CD error with tags

## [0.3.0] - 2026-01-07

### Added

- **Schema Validation**: Example YAML files are now validated against protobuf schema in CI
  - Created `tools/validate-examples.py` for automated validation
  - Validates that all example files conform to protobuf structure
  - Includes snake_case to camelCase conversion for protobuf compatibility
- **Development Release System**: Automated pre-release workflow for testing
  - New `.github/workflows/release-dev.yml` for development releases
  - Publishes to separate dev registries (`robotops-config-rust-dev`, `robotops-config-cpp-dev`)
  - Version format: `{VERSION}-development-{SHORT_SHA}` (e.g., `0.3.0-development-abc1234`)
  - Manual trigger from `development` branch only
  - Creates GitHub pre-releases for testing before production deployment
- **Build System Templates**: Template-based approach for Cargo.toml and conanfile.py
  - Created `tools/templates/` directory with Rust and C++ templates
  - Allows validation with standard tooling (cargo check, conan create)
  - Enables proper dependency management and version locking
  - Prevents syntax errors in generated build files

### Changed

- **Workflow Protections**: Enforced branch restrictions for releases
  - Production releases (Release workflow) can only run from `main` branch
  - Development releases can only run from `development` branch
  - Both workflows fail with clear error messages if triggered from wrong branch
- **YAML Generation**: Fixed formatting issues in generated default.yaml
  - Fixed array syntax (now generates `["item"]` instead of `"["item"]"`)
  - Improved blank line management (max 2 consecutive blank lines)
  - Proper handling of nested message structures
- **Example Files**: Fixed field name in trace_rate configuration
  - Changed `default` to `default_rate` to match protobuf schema

### Documentation

- Updated README with development release workflow
- Added template maintenance documentation in `tools/templates/README.md`
- Clarified branch restrictions for production vs development releases

## [0.2.0] - 2026-01-07

### Changed

- Versioning process and validations. Schema unchanged.
- Added VERSION file as single source of truth
- `just validate` enforces in CI that version must be bumped if changes are made to yaml files or schema
- `just bump-version [major|minor|patch]` now available to increment version and auto gen changelog entry

## [0.1.1] - 2026-01-06

### Added

- Initial release of robotops_config repository
- Configuration schema version field (`schema_version`)
- Comprehensive distributed tracing configuration (`tracing:` section)
  - Master enable switch with underlying RMW selection
  - Head-based trace sampling with per-topic overrides
  - Cross-process correlation settings
  - Clock synchronization configuration
  - Diagnostics publishing settings
  - Performance tuning parameters
- Full robot_agent configuration sections:
  - Robot identity and fleet management
  - Deployment version tracking
  - Backend authentication and connectivity
  - Topic discovery and filtering
  - Metrics collection configuration
  - Adaptive message capture strategies
  - TF (transform tree) monitoring
  - Action/service monitoring
  - Logging and log retention
  - System logs via journald
- Minimal configuration example (`config/minimal.yaml`)
- Usage-specific configuration examples:
  - High-bandwidth configuration for camera-heavy robots
  - Low-resource configuration for constrained devices
  - Nav2-optimized configuration for autonomous navigation
- JSON Schema for configuration validation (`schema/config.schema.json`)
- Comprehensive configuration reference documentation (`docs/REFERENCE.md`)
- CI workflow for automated config validation
- Apache 2.0 license

### Documentation

- README with installation and usage instructions
- Versioning policy documentation (major lock-step across components)
- Configuration reference covering all options
- Environment variable override documentation

[0.1.1]: https://github.com/RobotOpsInc/robotops_config/releases/tag/v0.1.1
