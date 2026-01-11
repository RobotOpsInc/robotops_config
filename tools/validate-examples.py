#!/usr/bin/env python3
"""
Validate example YAML files against protobuf schema.

This script ensures example configurations conform to the protobuf schema
by attempting to parse them into the generated protobuf message types.
"""

import sys
import yaml
from pathlib import Path
from google.protobuf import text_format
from google.protobuf.json_format import ParseDict, ParseError

# Add generated protobuf to path
sys.path.insert(0, str(Path(__file__).parent.parent / "generated" / "sdks" / "python"))

try:
    from proto.robotops.config.v1 import config_pb2
except ImportError:
    print("Error: Could not import generated protobuf types.")
    print("Run 'just generate' first to generate Python protobuf code.")
    sys.exit(1)


def snake_to_camel(snake_str: str) -> str:
    """Convert snake_case to camelCase"""
    components = snake_str.split('_')
    return components[0] + ''.join(x.title() for x in components[1:])


def convert_keys_to_camel_case(data):
    """Recursively convert all dictionary keys from snake_case to camelCase"""
    if isinstance(data, dict):
        return {snake_to_camel(k): convert_keys_to_camel_case(v) for k, v in data.items()}
    elif isinstance(data, list):
        return [convert_keys_to_camel_case(item) for item in data]
    else:
        return data


def validate_yaml_file(yaml_path: Path) -> bool:
    """Validate a YAML file against the protobuf schema."""
    print(f"Validating {yaml_path}...")

    try:
        # Load YAML file
        with open(yaml_path) as f:
            data = yaml.safe_load(f)

        if not isinstance(data, dict):
            print(f"  ❌ ERROR: YAML file is not a dictionary")
            return False

        # Convert snake_case keys to camelCase for protobuf compatibility
        camel_data = convert_keys_to_camel_case(data)

        # Try to parse into protobuf Config message
        config = config_pb2.Config()
        ParseDict(camel_data, config, ignore_unknown_fields=False)

        print(f"  ✅ Valid")
        return True

    except yaml.YAMLError as e:
        print(f"  ❌ YAML parsing error: {e}")
        return False
    except ParseError as e:
        print(f"  ❌ Schema validation error: {e}")
        return False
    except Exception as e:
        print(f"  ❌ Unexpected error: {e}")
        return False


def main():
    """Validate all example YAML files."""
    repo_root = Path(__file__).parent.parent
    examples_dir = repo_root / "examples"

    if not examples_dir.exists():
        print(f"Error: {examples_dir} does not exist")
        sys.exit(1)

    # Find all YAML files in examples/
    yaml_files = sorted(examples_dir.glob("*.yaml"))

    if not yaml_files:
        print(f"Warning: No YAML files found in {examples_dir}")
        sys.exit(0)

    print(f"Validating {len(yaml_files)} example file(s)...\n")

    all_valid = True
    for yaml_file in yaml_files:
        if not validate_yaml_file(yaml_file):
            all_valid = False

    print()
    if all_valid:
        print(f"✅ All {len(yaml_files)} example files are valid")
        sys.exit(0)
    else:
        print("❌ Some example files failed validation")
        sys.exit(1)


if __name__ == "__main__":
    main()
