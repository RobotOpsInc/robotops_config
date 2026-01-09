# Changelog

All notable changes to the RobotOps configuration will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

<!-- Versions below this line -->

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
