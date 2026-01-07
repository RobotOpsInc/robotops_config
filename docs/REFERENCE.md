# RobotOps Configuration Reference

Complete documentation for all RobotOps configuration options.

## Table of Contents

- [Configuration Version](#configuration-version)
- [Robot Identity](#robot-identity)
- [Deployment Version](#deployment-version)
- [Authentication](#authentication)
- [Backend Configuration](#backend-configuration)
- [Distributed Tracing](#distributed-tracing)
- [Discovery](#discovery)
- [Metrics](#metrics)
- [Subscriptions](#subscriptions)
- [TF (Transform Tree)](#tf-transform-tree)
- [Action/Service Monitoring](#actionservice-monitoring)
- [Logging](#logging)
- [Log Retention](#log-retention)
- [System Logs](#system-logs)

---

## Configuration Version

### `schema_version`

**Type:** String (semver format: `MAJOR.MINOR.PATCH`)
**Required:** Yes
**Default:** N/A

The configuration schema version for compatibility checking. Components validate that the major version matches their supported version.

```yaml
schema_version: "0.1.1"
```

**Version Compatibility:**
- All `v1.x.x` configs are compatible with all `v1.x.x` components
- Major version changes indicate breaking changes
- Minor version changes add backward-compatible features
- Patch version changes are documentation or default value updates

---

## Robot Identity

Uniquely identifies this robot within your organization for billing (robot-hours) and fleet organization.

### `robot.id`

**Type:** String
**Pattern:** Alphanumeric, hyphens, underscores; 3-128 characters
**Default:** Auto-generated as `{hostname}-{short_hash}`
**Environment Variable:** `ROBOT_OPS_AGENT_ROBOT_ID`

Unique identifier for this robot.

```yaml
robot:
  id: "amr-warehouse-007"
```

### `robot.nickname`

**Type:** String
**Default:** None
**Environment Variable:** `ROBOT_OPS_AGENT_ROBOT_NICKNAME`

Human-friendly display name (no validation constraints).

```yaml
robot:
  nickname: "Warehouse AMR #7"
```

### `robot.fleet_id`

**Type:** String
**Default:** `"default"`
**Environment Variable:** `ROBOT_OPS_AGENT_ROBOT_FLEET_ID`

Fleet assignment for organizing robots.

```yaml
robot:
  fleet_id: "warehouse-fleet-east"
```

### `robot.environment`

**Type:** String
**Default:** `"development"`
**Environment Variable:** `ROBOT_OPS_AGENT_ENVIRONMENT`

Deployment environment. Common values: `development`, `staging`, `production`, `simulation`.

```yaml
robot:
  environment: "production"
```

### `robot.tags`

**Type:** Object (key-value pairs)
**Constraints:**
- Keys: Alphanumeric, hyphens, underscores; 1-64 characters
- Values: Any string; 1-256 characters
- Maximum 20 tags per robot
**Environment Variable:** `ROBOT_OPS_AGENT_TAGS` (JSON format)

Custom tags for filtering and organization.

```yaml
robot:
  tags:
    region: us-west-2
    building: warehouse-7
    team: navigation
    hardware_revision: v2.1
```

---

## Deployment Version

### `deployment.version`

**Type:** String
**Default:** Empty
**Environment Variable:** `ROBOTOPS_DEPLOYMENT_VERSION` (takes precedence)

Software version identifier for your ROS2 deployment. Supports environment variable templating using `${VAR_NAME}`.

```yaml
deployment:
  version: "2.1.0"
  # Or with templating:
  # version: "v2.1.0-${BUILD_NUMBER}"
```

---

## Authentication

### `auth.api_key`

**Type:** String
**Pattern:** Must start with `sk_`
**Environment Variable:** `ROBOTOPS_API_KEY` (highest precedence)
**Security:** Never logged; use restricted file permissions (chmod 600)

API key for RobotOps backend authentication.

```yaml
auth:
  api_key: "sk_your_api_key_here"
```

### `auth.api_key_file`

**Type:** String (file path)
**Environment Variable:** `ROBOT_OPS_AGENT_AUTH_API_KEY_FILE` (not yet implemented)

Alternative: Read API key from file. Takes precedence over inline `api_key` but not over `ROBOTOPS_API_KEY`.

```yaml
auth:
  api_key_file: "/run/secrets/robotops_api_key"
```

---

## Backend Configuration

### `backend.url`

**Type:** String (URI)
**Default:** `"https://api.robotops.com"`
**Environment Variable:** `ROBOT_OPS_AGENT_BACKEND_URL`

Backend gRPC endpoint URL.

### `backend.heartbeat_interval_secs`

**Type:** Integer
**Default:** `30`
**Environment Variable:** `ROBOT_OPS_AGENT_BACKEND_HEARTBEAT_INTERVAL_SECS`

Heartbeat interval for connectivity checks (seconds).

### `backend.max_retry_attempts`

**Type:** Integer
**Default:** `5`
**Environment Variable:** `ROBOT_OPS_AGENT_BACKEND_MAX_RETRY_ATTEMPTS`

Maximum retry attempts before entering offline mode.

### `backend.initial_backoff_secs`

**Type:** Integer
**Default:** `1`
**Environment Variable:** `ROBOT_OPS_AGENT_BACKEND_INITIAL_BACKOFF_SECS`

Initial backoff duration for exponential backoff (seconds).

### `backend.max_backoff_secs`

**Type:** Integer
**Default:** `60`
**Environment Variable:** `ROBOT_OPS_AGENT_BACKEND_MAX_BACKOFF_SECS`

Maximum backoff duration for exponential backoff (seconds).

---

## Distributed Tracing

Configuration for rmw_robotops distributed tracing and robot_agent trace correlation.

### `tracing.enabled`

**Type:** Boolean
**Default:** `true`
**Environment Variable:** `ROBOTOPS_TRACING_ENABLED`

Master enable switch. When false, rmw_robotops acts as a pure passthrough with zero overhead.

```yaml
tracing:
  enabled: true
```

### `tracing.underlying_rmw`

**Type:** String
**Default:** `"rmw_fastrtps_cpp"`
**Options:** `rmw_fastrtps_cpp`, `rmw_cyclonedds_cpp`, `rmw_connextdds`
**Environment Variable:** `ROBOTOPS_UNDERLYING_RMW`

The RMW implementation that rmw_robotops wraps and delegates to.

```yaml
tracing:
  underlying_rmw: "rmw_fastrtps_cpp"
```

### Trace Sampling

#### `tracing.trace_rate.default`

**Type:** Number (0.0 - 1.0)
**Default:** `1.0`

Default sampling rate for all topics.

- `1.0`: Every publish creates a trace (100% sampling)
- `0.1`: 10% of publishes create traces
- `0.0`: No traces created

```yaml
tracing:
  trace_rate:
    default: 1.0
```

#### `tracing.trace_rate.overrides`

**Type:** Array of pattern/rate objects
**Default:** `[]`

Per-topic sampling rate overrides. First matching pattern wins.

```yaml
tracing:
  trace_rate:
    overrides:
      - pattern: "^/camera/.*/image_raw$"
        rate: 0.01  # 1% sampling
      - pattern: "^/cmd_vel$"
        rate: 1.0   # 100% sampling
```

### Correlation

#### `tracing.correlation.timestamp_tolerance_ns`

**Type:** Integer (nanoseconds)
**Default:** `10000000` (10ms)

Maximum time difference between publish and subscribe timestamps for correlation matching.

#### `tracing.correlation.window_secs`

**Type:** Integer (seconds)
**Default:** `30`

How long to keep publish events in the correlation window.

#### `tracing.correlation.hash_enabled`

**Type:** Boolean
**Default:** `true`

Use content hash (xxHash64) for disambiguation when multiple publish events have similar timestamps.

### Clock Synchronization

#### `tracing.clock.max_acceptable_skew_ns`

**Type:** Integer (nanoseconds)
**Default:** `10000000` (10ms)

Maximum acceptable clock skew before warning.

#### `tracing.clock.check_interval_secs`

**Type:** Integer (seconds)
**Default:** `60`

How often to perform clock synchronization checks.

### Diagnostics Publishing

#### `tracing.diagnostics.enabled`

**Type:** Boolean
**Default:** `true`

Enable/disable diagnostics topic publishing to `/robotops/diagnostics`.

#### `tracing.diagnostics.interval_secs`

**Type:** Integer (seconds)
**Default:** `10`

How often to publish diagnostics.

### Performance Tuning

#### `tracing.performance.queue_size`

**Type:** Integer
**Default:** `1024`

Internal queue depth for trace events before publishing to ROS2 topic.

#### `tracing.performance.failure_threshold`

**Type:** Integer
**Default:** `100`

Auto-disable tracing after this many consecutive failures. Set to 0 to disable auto-disable.

---

## Discovery

### `discovery.poll_interval_secs`

**Type:** Integer (seconds)
**Default:** `5`

How often to poll for new topics/nodes.

### `discovery.topic_filters`

**Type:** Array of strings (regex patterns)
**Optional**

If specified, only matching topics are subscribed.

```yaml
discovery:
  topic_filters:
    - "^/camera/.*"
    - "^/diagnostics$"
```

### `discovery.topic_blacklist`

**Type:** Array of strings (regex patterns)
**Optional**

Topics matching these patterns are excluded.

```yaml
discovery:
  topic_blacklist:
    - "^/rosout$"
```

---

## Metrics

### `metrics.enabled`

**Type:** Boolean
**Default:** `true`

Master enable switch for all metrics collection.

### `metrics.collection_interval_secs`

**Type:** Integer (seconds)
**Default:** `10`

Emit metrics snapshots at this interval.

### `metrics.autodetect_interval_secs`

**Type:** Integer (seconds)
**Default:** `60`

Re-check optional sensors (battery/thermal) at this interval.

### Individual Metric Categories

All default to `true`:

- `metrics.topic_metrics`
- `metrics.action_service_metrics`
- `metrics.infrastructure_metrics`
- `metrics.process_metrics`
- `metrics.temperature_metrics`
- `metrics.battery_metrics`
- `metrics.network_quality_metrics`

### `metrics.process_top_n`

**Type:** Integer
**Default:** `25`

Number of top processes to include in metrics.

### `metrics.include_process_cmdline`

**Type:** Boolean
**Default:** `false`

Include full command line for processes.

---

## Subscriptions

### `subscriptions.default_strategy`

**Type:** String
**Options:** `adaptive`, `all`, `on_change`, `rate_limit`, `sample`
**Default:** `adaptive`

Default capture strategy for message payloads.

### `subscriptions.default_target_hz`

**Type:** Number
**Default:** `10.0`

Default target Hz for rate limiting.

### Adaptive Strategy

#### `subscriptions.adaptive.high_bandwidth_threshold_bps`

**Type:** Integer (bytes/second)
**Default:** `1048576` (1 MB/s)

Topics above this bandwidth are considered high-bandwidth sensors.

#### `subscriptions.adaptive.high_bandwidth_target_hz`

**Type:** Number
**Default:** `2.0`

Downsample high-bandwidth topics to this rate.

#### `subscriptions.adaptive.low_bandwidth_threshold_bps`

**Type:** Integer (bytes/second)
**Default:** `10240` (10 KB/s)

Topics below this bandwidth are considered low-bandwidth state.

#### `subscriptions.adaptive.low_bandwidth_strategy`

**Type:** String
**Options:** `all`, `on_change`, `rate_limit`, `sample`
**Default:** `"on_change"`

Strategy for low-bandwidth topics.

#### `subscriptions.adaptive.medium_bandwidth_target_hz`

**Type:** Number
**Default:** `10.0`

Medium-bandwidth topics use this rate.

### Topic Overrides

```yaml
subscriptions:
  topic_overrides:
    - pattern: "^/camera/.*/image_raw$"
      strategy: "rate_limit"
      target_hz: 2.0
    - pattern: "^/cmd_vel$"
      strategy: "all"
```

---

## TF (Transform Tree)

### `tf.enabled`

**Type:** Boolean
**Default:** `true`

Enable/disable TF monitoring.

### `tf.snapshot_hz`

**Type:** Number
**Default:** `10.0`

Snapshot frequency (Hz).

### `tf.max_transforms`

**Type:** Integer
**Default:** `10000`

Maximum number of transforms kept in memory.

### `tf.verbose`

**Type:** Boolean
**Default:** `false`

Enable verbose TF debugging logs.

---

## Action/Service Monitoring

### Actions

#### `monitoring.actions.enabled`

**Type:** Boolean
**Default:** `true`

#### `monitoring.actions.strategy`

**Type:** String
**Options:** `all`, `on_change`, `rate_limit`, `sample`
**Default:** `"rate_limit"`

#### `monitoring.actions.target_hz`

**Type:** Number
**Default:** `50.0`

#### `monitoring.actions.sample_percentage`

**Type:** Number (0-100)
**Default:** `10.0`

#### `monitoring.actions.max_payload_bytes`

**Type:** Integer
**Default:** `16384`

### Services

Similar structure to actions, with different defaults:

- `target_hz`: `20.0`
- `max_payload_bytes`: `65536`

---

## Logging

### `logging.capture_rosout`

**Type:** Boolean
**Default:** `true`

Capture ROS2 logs from /rosout topic.

### `logging.level`

**Type:** String
**Options:** `trace`, `debug`, `info`, `warn`, `error`
**Default:** `"info"`

Agent log level.

### `logging.format`

**Type:** String
**Options:** `json`, `human`
**Default:** `"json"`

Default output format for logs.

### Output Destinations

#### `logging.outputs.stdout.enabled`

**Type:** Boolean
**Default:** `true`

#### `logging.outputs.file.enabled`

**Type:** Boolean
**Default:** `false`

#### `logging.outputs.file.path`

**Type:** String
**Default:** `"/var/log/robot_ops_agent"`

#### `logging.outputs.file.max_size_mb`

**Type:** Integer
**Default:** `100`

#### `logging.outputs.file.max_files`

**Type:** Integer
**Default:** `5`

#### `logging.outputs.journald.enabled`

**Type:** Boolean or String
**Options:** `true`, `false`, `"auto"`
**Default:** `"auto"`

Auto-enable if journald is available.

---

## Log Retention

### `log_retention.delete_after_upload`

**Type:** Boolean
**Default:** `true`

Delete logs after successful upload.

### `log_retention.max_local_storage_mb`

**Type:** Integer
**Default:** `500`

Maximum local storage for logs.

### `log_retention.max_age_days`

**Type:** Integer
**Default:** `30`

Maximum age for logs before deletion.

---

## System Logs

Best-effort host system log collection via journald.

### `system_logs.enabled`

**Type:** Boolean
**Default:** `true`

### `system_logs.on_unavailable`

**Type:** String
**Options:** `warn`, `ignore`
**Default:** `"warn"`

Behavior when journald is unavailable.

### Journald Configuration

#### `system_logs.journald.min_priority`

**Type:** String
**Options:** `emerg`, `alert`, `crit`, `err`, `warning`, `notice`, `info`, `debug`
**Default:** `"warning"`

#### Unit Filtering

```yaml
system_logs:
  journald:
    units:
      include: ["*.service"]
      exclude:
        - "user@*"
        - "session-*"
```

#### Source Categories

Each source (`kernel`, `systemd`, `network`, `auth`) has:
- `enabled`: Boolean
- `min_priority`: String (log level)

#### Transports

Boolean flags for:
- `kernel`
- `syslog`
- `stdout`
- `journal`
- `audit`

#### Rate Limiting

- `max_logs_per_minute`: Integer (default: 100)
- `burst_allowance`: Integer (default: 200)

#### Deduplication

- `enabled`: Boolean (default: true)
- `window_seconds`: Integer (default: 60)

---

## Examples

See the `/config/examples/` directory for complete configuration examples:

- **high-bandwidth.yaml** - For camera-heavy robots
- **low-resource.yaml** - For constrained devices
- **nav2.yaml** - Tuned for Nav2 workloads

---

## Environment Variable Priority

When both config file and environment variable are specified, environment variables take precedence:

1. Environment variable (highest priority)
2. Config file value
3. Default value (lowest priority)
