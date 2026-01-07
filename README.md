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

Download the versioned configuration from GitHub releases:

```bash
# Create directory
sudo mkdir -p /etc/robotops

# Download a specific version (recommended - use latest release version)
curl -L -o /etc/robotops/config.yaml \
  https://github.com/RobotOpsInc/robotops_config/releases/download/v0.2.0/config.yaml

# Or get the latest release
LATEST_VERSION=$(curl -s https://api.github.com/repos/RobotOpsInc/robotops_config/releases/latest | grep '"tag_name"' | cut -d'"' -f4)
curl -L -o /etc/robotops/config.yaml \
  "https://github.com/RobotOpsInc/robotops_config/releases/download/${LATEST_VERSION}/config.yaml"
```

### CI/CD Usage

In automated pipelines, download the config from releases:

```yaml
- name: Download RobotOps config
  run: |
    curl -L -o config.yaml \
      https://github.com/RobotOpsInc/robotops_config/releases/download/v0.2.0/config.yaml
```

The predictable URL format is:
```
https://github.com/RobotOpsInc/robotops_config/releases/download/{tag}/config.yaml
```

## Usage

Both `rmw_robotops` and `robot_agent` read configuration from `/etc/robotops/config.yaml` by default.

You can override the location using the `ROBOTOPS_CONFIG_PATH` environment variable:

```bash
export ROBOTOPS_CONFIG_PATH=/path/to/custom/config.yaml
ros2 run my_package my_node
```

## Configuration Reference

See the [auto-generated default configuration](generated/yaml/default.yaml) for detailed documentation on all available options. For example configurations, see the [examples/](examples/) directory.

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

## Architecture

### Protobuf-Based Configuration

As of v0.3.0, RobotOps configuration uses **Protocol Buffers** as the single source of truth with auto-generated SDKs for Rust and C++.

```
robotops_config/
├── proto/
│   └── robotops/config/v1/
│       └── config.proto          # Schema with @default, @env annotations
├── tools/
│   └── robotops-codegen/         # Custom defaults generator
├── generated/
│   ├── rust/
│   │   ├── robotops.config.v1.rs # prost-generated types
│   │   └── defaults.rs           # Generated Default impls
│   ├── cpp/
│   │   ├── config.pb.h/cc        # protobuf-generated types
│   │   └── defaults.hpp          # Generated factory functions
│   └── yaml/
│       └── default.yaml          # Human-readable config with docs
├── buf.yaml                      # Buf configuration
└── buf.gen.yaml                  # Code generation config
```

### Benefits

- **Single source of truth**: Proto schema defines types, defaults, and documentation
- **Generated SDKs**: Auto-generated Rust and C++ bindings with `Default` implementations
- **Versioned packages**: Published to Cloudsmith for consumption by `robot_agent` and `rmw_robotops`
- **Human-readable YAML**: Auto-generated `default.yaml` with inline documentation
- **Breaking change detection**: `buf breaking` prevents unintentional API breakage

### Code Generation

The build process:

1. **Proto → Rust/C++**: `buf generate` uses [prost](https://github.com/tokio-rs/prost) and protoc to generate base types
2. **Proto comments → Defaults**: `robotops-codegen` parses structured comments to generate:
   - Rust `Default` trait implementations
   - C++ factory functions
   - YAML config file with documentation

**Prerequisites:**

```bash
# Install just (https://github.com/casey/just)
brew install just               # macOS
cargo install just              # Via Rust
# or download from https://github.com/casey/just/releases

# Install buf (https://buf.build/docs/installation)
brew install bufbuild/buf/buf  # macOS
# or download from https://github.com/bufbuild/buf/releases

# Install Protocol Buffers compiler
brew install protobuf           # macOS
sudo apt install protobuf-compiler  # Ubuntu/Debian

# Python 3.11+ (for robotops-codegen)
python3 --version  # Should be 3.11 or higher
```

**Generate code locally:**

```bash
just generate
```

This runs both `buf generate` (for protobuf code) and `robotops-codegen` (for defaults, YAML, package files).

**Clean generated code:**

```bash
just clean
```

### Structured Comment Annotations

The proto schema uses structured comments to encode metadata:

```protobuf
// Master enable switch for distributed tracing.
// @default true
// @env ROBOTOPS_TRACING_ENABLED
bool enabled = 1;

// Default sampling rate for all topics (0.0 - 1.0).
// @default 1.0
// @min 0.0
// @max 1.0
double default_rate = 2;

// The RMW implementation to wrap.
// @default "rmw_fastrtps_cpp"
// @enum rmw_fastrtps_cpp
// @enum rmw_cyclonedds_cpp
// @env ROBOTOPS_UNDERLYING_RMW
string underlying_rmw = 3;
```

**Supported annotations:**

| Annotation | Purpose | Example |
|------------|---------|---------|
| `@default` | Default value | `@default true`, `@default "rmw_fastrtps_cpp"` |
| `@env` | Environment variable | `@env ROBOTOPS_TRACING_ENABLED` |
| `@min` | Minimum value | `@min 0.0` |
| `@max` | Maximum value | `@max 1.0` |
| `@enum` | Allowed values | `@enum rmw_fastrtps_cpp` |
| `@unit` | Unit of measurement | `@unit nanoseconds` |
| `@example` | Example value | `@example "^/camera/.*/image_raw$"` |
| `@section` | YAML section header | `@section Distributed Tracing` |

### Consuming Generated SDKs

**Rust (robot_agent):**

```toml
# Cargo.toml
[dependencies]
robotops-config = { version = "0.3", registry = "cloudsmith" }
```

```rust
use robotops_config::robotops::config::v1::*;

let config = Config::default();
assert_eq!(config.schema_version, "0.2.0");
```

**C++ (rmw_robotops):**

```cmake
# CMakeLists.txt
find_package(robotops-config REQUIRED)
target_link_libraries(your_target robotops-config::robotops-config)
```

```cpp
#include <robotops/config/v1/config.pb.h>
#include <robotops/config/v1/defaults.hpp>

auto config = robotops::config::v1::CreateDefaultConfig();
```

## Development

### Versioning Workflow

This repository uses a `VERSION` file as the single source of truth for the schema version.

**Version Synchronization:**

The following locations must **always** have matching versions:
- `VERSION` file (e.g., `0.2.0`)
- Proto schema: `@default "0.2.0"` annotation on `Config.schema_version` field
- Generated YAML: `generated/yaml/default.yaml` schema_version field
- Example YAML files (if they include schema_version)

Use `just validate-versions` to verify all versions are synchronized.

**Bumping versions:**

```bash
# Install just (https://just.systems)
brew install just  # or: cargo install just

# Bump version (automatically updates VERSION, YAML configs, proto @default, and regenerates code)
just bump-version patch  # or: minor, major

# Verify all versions match
just validate-versions
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
