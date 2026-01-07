#!/usr/bin/env python3
"""
RobotOps Code Generator

Parses proto files with structured comment annotations and generates:
1. Rust defaults.rs - Default trait implementations
2. C++ defaults.hpp - Factory functions
3. YAML default.yaml - Human-readable config with inline docs

Annotation syntax in proto comments:
  @default <value>   - Default value for field
  @env <VAR_NAME>    - Environment variable override
  @min <value>       - Minimum value (numeric)
  @max <value>       - Maximum value (numeric)
  @enum <val1>       - Allowed enum value (can have multiple)
  @unit <unit>       - Unit of measurement
  @example <value>   - Example value
  @section <title>   - Section header for YAML output
"""

import re
import sys
import os
from pathlib import Path
from typing import Dict, List, Optional, Any
from dataclasses import dataclass, field as dataclass_field

@dataclass
class Annotation:
    """Parsed annotation from proto comments"""
    default: Optional[str] = None
    env: Optional[str] = None
    min: Optional[str] = None
    max: Optional[str] = None
    enums: List[str] = dataclass_field(default_factory=list)
    unit: Optional[str] = None
    examples: List[str] = dataclass_field(default_factory=list)
    section: Optional[str] = None

@dataclass
class Field:
    """Parsed proto field"""
    name: str
    proto_type: str
    number: int
    comments: List[str]
    annotations: Annotation
    is_repeated: bool = False
    is_map: bool = False
    map_key_type: Optional[str] = None
    map_value_type: Optional[str] = None

@dataclass
class Message:
    """Parsed proto message"""
    name: str
    fields: List[Field]
    comments: List[str]
    annotations: Annotation
    nested_messages: List['Message'] = dataclass_field(default_factory=list)


class ProtoParser:
    """Parser for proto files with structured comment annotations"""

    FIELD_PATTERN = re.compile(
        r'^\s*(?:repeated\s+)?(map<(\w+),\s*(\w+)>|[\w.]+)\s+(\w+)\s*=\s*(\d+);'
    )
    MESSAGE_PATTERN = re.compile(r'^\s*message\s+(\w+)\s*\{')

    def __init__(self):
        self.messages: List[Message] = []
        self.current_comments: List[str] = []
        self.current_annotations = Annotation()

    def parse_file(self, proto_path: Path) -> List[Message]:
        """Parse a proto file and return all top-level messages"""
        with open(proto_path, 'r') as f:
            lines = f.readlines()

        self._parse_lines(lines)
        return self.messages

    def _parse_lines(self, lines: List[str]) -> None:
        """Parse lines from proto file"""
        i = 0
        while i < len(lines):
            line = lines[i].rstrip()

            # Handle comments
            if line.strip().startswith('//'):
                comment = line.strip()[2:].strip()
                self.current_comments.append(comment)
                self._parse_annotation(comment)
                i += 1
                continue

            # Handle message definitions
            msg_match = self.MESSAGE_PATTERN.match(line)
            if msg_match:
                msg_name = msg_match.group(1)
                # Parse message body
                i += 1
                msg_fields, nested, i = self._parse_message_body(lines, i)

                msg = Message(
                    name=msg_name,
                    fields=msg_fields,
                    comments=self.current_comments.copy(),
                    annotations=self.current_annotations,
                    nested_messages=nested
                )
                self.messages.append(msg)
                self._reset_context()
                continue

            i += 1

    def _parse_message_body(self, lines: List[str], start: int) -> tuple:
        """Parse the body of a message definition"""
        fields = []
        nested = []
        i = start
        depth = 1

        while i < len(lines) and depth > 0:
            line = lines[i].rstrip()

            if '{' in line:
                depth += 1
            if '}' in line:
                depth -= 1
                if depth == 0:
                    break

            # Handle comments
            if line.strip().startswith('//'):
                comment = line.strip()[2:].strip()
                self.current_comments.append(comment)
                self._parse_annotation(comment)
                i += 1
                continue

            # Handle nested messages
            msg_match = self.MESSAGE_PATTERN.match(line)
            if msg_match:
                msg_name = msg_match.group(1)
                i += 1
                msg_fields, sub_nested, i = self._parse_message_body(lines, i)

                msg = Message(
                    name=msg_name,
                    fields=msg_fields,
                    comments=self.current_comments.copy(),
                    annotations=self.current_annotations,
                    nested_messages=sub_nested
                )
                nested.append(msg)
                self._reset_context()
                continue

            # Handle field definitions
            field_match = self.FIELD_PATTERN.match(line)
            if field_match:
                groups = field_match.groups()
                is_repeated = 'repeated' in line

                # Check if it's a map field
                if groups[1] and groups[2]:  # map<K, V>
                    proto_type = groups[0]
                    field_name = groups[3]
                    field_num = int(groups[4])

                    f = Field(
                        name=field_name,
                        proto_type=proto_type,
                        number=field_num,
                        comments=self.current_comments.copy(),
                        annotations=self.current_annotations,
                        is_map=True,
                        map_key_type=groups[1],
                        map_value_type=groups[2]
                    )
                else:  # regular field
                    proto_type = groups[0]
                    field_name = groups[3]
                    field_num = int(groups[4])

                    f = Field(
                        name=field_name,
                        proto_type=proto_type,
                        number=field_num,
                        comments=self.current_comments.copy(),
                        annotations=self.current_annotations,
                        is_repeated=is_repeated
                    )

                fields.append(f)
                self._reset_context()

            i += 1

        return fields, nested, i

    def _parse_annotation(self, comment: str) -> None:
        """Parse structured annotations from a comment"""
        # @default <value>
        if match := re.match(r'@default\s+(.+)$', comment):
            self.current_annotations.default = match.group(1).strip()

        # @env <VAR_NAME>
        elif match := re.match(r'@env\s+(\S+)', comment):
            self.current_annotations.env = match.group(1).strip()

        # @min <value>
        elif match := re.match(r'@min\s+(.+)$', comment):
            self.current_annotations.min = match.group(1).strip()

        # @max <value>
        elif match := re.match(r'@max\s+(.+)$', comment):
            self.current_annotations.max = match.group(1).strip()

        # @enum <value>
        elif match := re.match(r'@enum\s+(.+)$', comment):
            self.current_annotations.enums.append(match.group(1).strip())

        # @unit <unit>
        elif match := re.match(r'@unit\s+(.+)$', comment):
            self.current_annotations.unit = match.group(1).strip()

        # @example <value>
        elif match := re.match(r'@example\s+(.+)$', comment):
            self.current_annotations.examples.append(match.group(1).strip())

        # @section <title>
        elif match := re.match(r'@section\s+(.+)$', comment):
            self.current_annotations.section = match.group(1).strip()

    def _reset_context(self) -> None:
        """Reset current comment/annotation context"""
        self.current_comments = []
        self.current_annotations = Annotation()


class RustGenerator:
    """Generate Rust defaults.rs"""

    def generate(self, messages: List[Message], output_dir: Path) -> None:
        """Generate Rust default implementations"""
        lines = []
        lines.append("// Auto-generated defaults for robotops.config.v1")
        lines.append("// DO NOT EDIT - generated by robotops-codegen")
        lines.append("")
        lines.append("use super::robotops::config::v1::*;")
        lines.append("")

        for msg in messages:
            self._generate_message_defaults(msg, lines)

        output_file = output_dir / "rust" / "defaults.rs"
        output_file.parent.mkdir(parents=True, exist_ok=True)
        with open(output_file, 'w') as f:
            f.write('\n'.join(lines))

    def _generate_message_defaults(self, msg: Message, lines: List[str], prefix: str = "") -> None:
        """Generate Default impl for a message"""
        # Generate nested messages first
        for nested in msg.nested_messages:
            self._generate_message_defaults(nested, lines, f"{msg.name}::")

        # Skip if no fields have defaults
        has_defaults = any(f.annotations.default is not None for f in msg.fields)
        if not has_defaults and not msg.nested_messages:
            return

        # Generate Default impl
        lines.append(f"impl Default for {prefix}{msg.name} {{")
        lines.append("    fn default() -> Self {")
        lines.append("        Self {")

        for f in msg.fields:
            default_val = self._get_default_value(f)
            if default_val:
                lines.append(f"            {f.name}: {default_val},")

        lines.append("        }")
        lines.append("    }")
        lines.append("}")
        lines.append("")

    def _get_default_value(self, field: Field) -> Optional[str]:
        """Get Rust default value for a field"""
        if not field.annotations.default:
            return None

        default = field.annotations.default

        # Handle different types
        if field.proto_type == "bool":
            return default.lower()
        elif field.proto_type in ["int32", "int64", "uint32", "uint64"]:
            return default
        elif field.proto_type in ["float", "double"]:
            return default
        elif field.proto_type == "string":
            # Remove quotes if present and re-quote
            val = default.strip('"').strip("'")
            return f'"{val}".to_string()'
        elif field.is_map:
            return "std::collections::HashMap::new()"
        elif field.is_repeated:
            return "vec![]"
        else:
            # Nested message
            return f"Some({field.proto_type}::default())"


class CppGenerator:
    """Generate C++ defaults.hpp"""

    def generate(self, messages: List[Message], output_dir: Path) -> None:
        """Generate C++ default factory functions"""
        lines = []
        lines.append("#pragma once")
        lines.append("// Auto-generated defaults for robotops.config.v1")
        lines.append("// DO NOT EDIT - generated by robotops-codegen")
        lines.append("")
        lines.append('#include "robotops/config/v1/config.pb.h"')
        lines.append('#include <string>')
        lines.append("")
        lines.append("namespace robotops {")
        lines.append("namespace config {")
        lines.append("namespace v1 {")
        lines.append("")

        for msg in messages:
            self._generate_message_factory(msg, lines)

        lines.append("} // namespace v1")
        lines.append("} // namespace config")
        lines.append("} // namespace robotops")

        output_file = output_dir / "cpp" / "defaults.hpp"
        output_file.parent.mkdir(parents=True, exist_ok=True)
        with open(output_file, 'w') as f:
            f.write('\n'.join(lines))

    def _generate_message_factory(self, msg: Message, lines: List[str]) -> None:
        """Generate factory function for a message"""
        # Generate nested messages first
        for nested in msg.nested_messages:
            self._generate_message_factory(nested, lines)

        # Skip if no fields have defaults
        has_defaults = any(f.annotations.default is not None for f in msg.fields)
        if not has_defaults and not msg.nested_messages:
            return

        # Generate factory function
        lines.append(f"inline {msg.name} CreateDefault{msg.name}() {{")
        lines.append(f"    {msg.name} config;")

        for f in msg.fields:
            setter = self._get_setter_call(f)
            if setter:
                lines.append(f"    {setter};")

        lines.append("    return config;")
        lines.append("}")
        lines.append("")

    def _get_setter_call(self, field: Field) -> Optional[str]:
        """Get C++ setter call for a field"""
        if not field.annotations.default:
            return None

        default = field.annotations.default

        # Handle different types
        if field.proto_type == "bool":
            val = default.lower()
            return f"config.set_{field.name}({val})"
        elif field.proto_type in ["int32", "int64", "uint32", "uint64"]:
            return f"config.set_{field.name}({default})"
        elif field.proto_type in ["float", "double"]:
            return f"config.set_{field.name}({default})"
        elif field.proto_type == "string":
            # Remove quotes if present and re-quote
            val = default.strip('"').strip("'")
            return f'config.set_{field.name}("{val}")'
        elif not field.is_repeated and not field.is_map:
            # Nested message
            return f"*config.mutable_{field.name}() = CreateDefault{field.proto_type}()"

        return None


class YamlGenerator:
    """Generate default.yaml"""

    def generate(self, messages: List[Message], output_dir: Path) -> None:
        """Generate YAML config with inline documentation"""
        lines = []
        lines.append("# =============================================================================")
        lines.append("# Robot Agent Configuration")
        lines.append("# Auto-generated from proto schema - DO NOT EDIT")
        lines.append("# =============================================================================")
        lines.append("")

        # Find the root Config message
        config_msg = next((m for m in messages if m.name == "Config"), None)
        if config_msg:
            self._generate_message_yaml(config_msg, messages, lines, indent=0)

        output_file = output_dir / "yaml" / "default.yaml"
        output_file.parent.mkdir(parents=True, exist_ok=True)
        with open(output_file, 'w') as f:
            f.write('\n'.join(lines))

    def _generate_message_yaml(self, msg: Message, all_messages: List[Message],
                                lines: List[str], indent: int) -> None:
        """Generate YAML for a message"""
        ind = "  " * indent

        for f in msg.fields:
            # Add section headers
            if f.annotations.section:
                if lines and lines[-1] != "":
                    lines.append("")
                lines.append(f"{ind}# " + "=" * 77)
                lines.append(f"{ind}# {f.annotations.section}")
                lines.append(f"{ind}# " + "=" * 77)
                lines.append("")

            # Add field comments
            for comment in f.comments:
                if not comment.startswith('@'):
                    lines.append(f"{ind}# {comment}")

            # Add env var info
            if f.annotations.env:
                lines.append(f"{ind}# Env: {f.annotations.env}")

            # Add default value
            if f.annotations.default:
                yaml_val = self._get_yaml_value(f)
                lines.append(f"{ind}{f.name}: {yaml_val}")
            elif self._is_message_type(f, all_messages):
                # Nested message
                lines.append(f"{ind}{f.name}:")
                nested_msg = self._find_message(f.proto_type, all_messages)
                if nested_msg:
                    self._generate_message_yaml(nested_msg, all_messages, lines, indent + 1)

            lines.append("")

    def _get_yaml_value(self, field: Field) -> str:
        """Get YAML representation of default value"""
        default = field.annotations.default

        if field.proto_type == "bool":
            return default.lower()
        elif field.proto_type in ["int32", "int64", "uint32", "uint64", "float", "double"]:
            return default
        elif field.proto_type == "string":
            # Remove quotes if present and re-quote for YAML
            val = default.strip('"').strip("'")
            return f'"{val}"'
        elif field.is_map:
            return "{}"
        elif field.is_repeated:
            return "[]"
        else:
            return ""

    def _is_message_type(self, field: Field, all_messages: List[Message]) -> bool:
        """Check if field type is a message"""
        return any(m.name == field.proto_type for m in all_messages)

    def _find_message(self, name: str, messages: List[Message]) -> Optional[Message]:
        """Find a message by name"""
        for msg in messages:
            if msg.name == name:
                return msg
            # Check nested messages
            for nested in msg.nested_messages:
                if nested.name == name:
                    return nested
        return None


def main():
    """Main entry point"""
    # Get input/output paths from environment or args
    # For buf plugin, input comes from stdin and output goes to --out dir

    # For now, we'll use a simple approach:
    # Find all .proto files in proto/ directory
    # Generate to generated/ directory

    script_dir = Path(__file__).parent
    repo_root = script_dir.parent.parent
    proto_dir = repo_root / "proto"
    output_dir = repo_root / "generated"

    # Find config.proto
    config_proto = proto_dir / "robotops" / "config" / "v1" / "config.proto"
    if not config_proto.exists():
        print(f"Error: {config_proto} not found", file=sys.stderr)
        sys.exit(1)

    # Parse proto file
    parser = ProtoParser()
    messages = parser.parse_file(config_proto)

    # Flatten nested messages for easier lookup
    all_messages = []
    def flatten(msgs):
        for msg in msgs:
            all_messages.append(msg)
            flatten(msg.nested_messages)
    flatten(messages)

    # Generate code
    rust_gen = RustGenerator()
    rust_gen.generate(all_messages, output_dir)

    cpp_gen = CppGenerator()
    cpp_gen.generate(all_messages, output_dir)

    yaml_gen = YamlGenerator()
    yaml_gen.generate(all_messages, output_dir)

    print(f"Generated code to {output_dir}")
    print(f"  - Rust: {output_dir}/rust/defaults.rs")
    print(f"  - C++: {output_dir}/cpp/defaults.hpp")
    print(f"  - YAML: {output_dir}/yaml/default.yaml")


if __name__ == "__main__":
    main()
