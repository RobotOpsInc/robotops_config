# RobotOps Configuration

Shared configuration for RobotOps distributed tracing and monitoring infrastructure.

## Overview

This repository provides a single source of truth for RobotOps configuration consumed by:

- **`rmw_robotops`** - Public ROS 2 RMW implementation with distributed tracing
- **`robot_agent`** - Private telemetry collection and export agent

By maintaining configuration in a dedicated public repository, we ensure:

- Transparent schema for open source contributors
- No dependency on private repositories from public projects
- Version-controlled configuration updates across the ecosystem

## Installation

### Recommended: System-wide Installation

The default configuration is installed to `/etc/robotops/config.yaml` when you install the `ros-jazzy-rmw-robotops` Debian package:

```bash
sudo apt install ros-jazzy-rmw-robotops
```

### Manual Installation

Download the latest configuration directly:

```bash
# Create directory
sudo mkdir -p /etc/robotops

# Download latest config from main branch
curl -L https://raw.githubusercontent.com/RobotOpsInc/robotops_config/main/config/default.yaml \
  | sudo tee /etc/robotops/config.yaml > /dev/null

# Or download a specific version
curl -L https://raw.githubusercontent.com/RobotOpsInc/robotops_config/v1.0.0/config/default.yaml \
  | sudo tee /etc/robotops/config.yaml > /dev/null
```

### CI/CD Usage

In automated pipelines, download the config as needed:

```yaml
- name: Download RobotOps config
  run: |
    mkdir -p config
    curl -LO https://raw.githubusercontent.com/RobotOpsInc/robotops_config/v1.0.0/config/default.yaml
```

## Usage

Both `rmw_robotops` and `robot_agent` read configuration from `/etc/robotops/config.yaml` by default.

You can override the location using the `ROBOTOPS_CONFIG_PATH` environment variable:

```bash
export ROBOTOPS_CONFIG_PATH=/path/to/custom/config.yaml
ros2 run my_package my_node
```

## Configuration Reference

See the [default configuration](config/default.yaml) for detailed documentation on all available options.

Key sections include:

- **Robot Identity** - Unique robot ID, fleet assignment, environment tags
- **Deployment Version** - Track software version across your fleet
- **Authentication** - API key configuration for backend communication
- **Discovery** - Topic filtering and polling configuration
- **Metrics** - System and ROS metrics collection settings
- **Subscriptions** - Adaptive capture strategies for bandwidth optimization
- **TF Monitoring** - Transform tree snapshot configuration
- **Logging** - Agent and system log capture settings

## Versioning

This repository uses **semantic versioning** expressed via git tags (e.g., `v1.0.0`, `v1.2.3`).

### Lockstep Major Versions

**Major versions** move in lockstep across the entire RobotOps ecosystem:

- `robot_agent`
- `rmw_robotops`
- `robotops_msgs`
- `robotops_config` (this repository)

When any component introduces a breaking change, all components bump to the next major version together (e.g., all move from `v1.x.x` to `v2.0.0`).

### Independent Minor and Patch Versions

Between major version boundaries, each component evolves independently:

- **Minor versions** (e.g., `v1.1.0` → `v1.2.0`) add backward-compatible features
- **Patch versions** (e.g., `v1.2.0` → `v1.2.1`) fix bugs without breaking compatibility

**Backward compatibility is maintained by design** for all minor and patch releases within the same major version.

### Examples

```
robotops_config v1.0.0   ←  Initial stable release
robotops_config v1.1.0   ←  Added new config section (backward compatible)
robotops_config v1.1.1   ←  Fixed typo in default.yaml

robot_agent v1.2.0       ←  May be ahead (added new feature)
rmw_robotops v1.0.3      ←  May be behind (only bug fixes)

robotops_config v2.0.0   ←  Breaking change: all components bump major version
```

**Recommendation:** Always reference a specific version tag in production deployments for stability.

## Development

### Versioning Workflow

This repository uses a `VERSION` file as the single source of truth for the schema version.

**Bumping versions:**

```bash
# Install just (https://just.systems)
brew install just  # or: cargo install just

# Bump version (updates VERSION, all config files, and CHANGELOG)
just bump-version patch  # or: minor, major
```

**Validation:**

```bash
# Run all validations (schema, versions, linting) via Docker
just validate

# Or run natively (requires ajv-cli and yamllint)
just validate-native
```

### Creating Releases

Releases are created manually by maintainers via GitHub Actions:

1. Ensure `VERSION` and `CHANGELOG.md` are updated on `main`
2. Go to **Actions** → **Release** → **Run workflow**
3. (Optional) Enable "Dry run" to preview without creating
4. Click **Run workflow**

The release will:
- Create a git tag matching the VERSION (e.g., `v0.2.0`)
- Auto-populate release notes from CHANGELOG.md

## Maintenance

This repository is maintained by the **Robot Ops team** and is publicly available for the ROS 2 community.

For questions, issues, or contributions, please refer to our main documentation or contact the Robot Ops team.

## License

Apache 2.0
