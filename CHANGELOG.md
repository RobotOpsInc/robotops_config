# Changelog

All notable changes to the RobotOps configuration will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

<!-- Versions below this line -->

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
