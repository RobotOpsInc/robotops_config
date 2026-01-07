# Build System Templates

This directory contains templates for SDK build system files (Cargo.toml, conanfile.py).

## Why Templates?

Instead of generating build system files from scratch in Python, we maintain them as real files that can be validated with standard tooling. This approach:

- **Prevents syntax errors**: Files are validated with `cargo check`, `conan create`, etc.
- **Enables dependency management**: Use `cargo update`, conan version management tools
- **Locks toolchain versions**: Specify `rust-version` in Cargo.toml
- **Catches issues early**: Syntax errors appear when editing templates, not during generation

## Maintaining Templates

### Cargo.toml (Rust)

Edit `rust/Cargo.toml` directly and validate with:

```bash
# Test generation
just clean && just generate

# Validate with cargo
cargo check --manifest-path generated/sdks/rust/Cargo.toml

# Update dependencies
cd tools/templates/rust
cargo update --dry-run  # See available updates
# Edit Cargo.toml manually to update versions
```

### conanfile.py (C++)

Edit `cpp/conanfile.py` directly and validate with:

```bash
# Test generation
just clean && just generate

# Validate with conan (if you have it installed)
cd generated/sdks/cpp
conan create . --build=missing
```

## Version Placeholder

Both templates use `{{VERSION}}` as a placeholder that gets replaced during code generation with the version from the `VERSION` file.

**Never** hardcode the version in templates - always use `{{VERSION}}`.

## Adding New Templates

To add a new template file:

1. Create the template file in the appropriate subdirectory
2. Use `{{VERSION}}` for version placeholders
3. Update `tools/robotops-codegen/main.py` to copy and process the template
4. Test generation with `just clean && just generate`
5. Validate the generated file with its respective toolchain
