# Version management and code generation for robotops_config

# Lint proto files for style and correctness
lint:
    @echo "Linting proto files..."
    docker build -t robotops-config:build .
    docker run --rm -v $(pwd):/ws/src/robotops-config robotops-config:build bash -c "cd /ws/src/robotops-config && buf lint proto"
    @echo "✅ Proto linting passed"

# Generate all code from proto schema (Rust, C++, YAML)
generate:
    @echo "Generating code from proto schema..."
    docker build -t robotops-config:build .
    docker run --rm -v $(pwd):/ws/src/robotops-config -u $(id -u):$(id -g) -e XDG_CACHE_HOME=/tmp/.cache robotops-config:build bash -c "cd /ws/src/robotops-config && buf generate"
    python3 tools/robotops-codegen/main.py
    @echo ""
    @echo "✅ Code generation complete"

# Clean all generated code
clean:
    @echo "Cleaning generated code..."
    rm -rf generated/
    @echo "✅ Cleaned generated/ directory"

# Bump version across all config files
bump-version LEVEL:
    #!/usr/bin/env bash
    set -euo pipefail

    CURRENT=$(cat VERSION)

    # Parse current version
    IFS='.' read -r MAJOR MINOR PATCH <<< "$CURRENT"

    # Bump based on level
    case "{{LEVEL}}" in
        major) MAJOR=$((MAJOR + 1)); MINOR=0; PATCH=0 ;;
        minor) MINOR=$((MINOR + 1)); PATCH=0 ;;
        patch) PATCH=$((PATCH + 1)) ;;
        *) echo "Usage: just bump-version [major|minor|patch]"; exit 1 ;;
    esac

    NEW_VERSION="${MAJOR}.${MINOR}.${PATCH}"
    echo "Bumping version: $CURRENT -> $NEW_VERSION"

    # Update VERSION file
    echo "$NEW_VERSION" > VERSION

    # Check if example files need manual updates
    EXAMPLE_FILES_WITH_VERSION=()
    for f in examples/*.yaml; do
        if [ -f "$f" ] && grep -q "schema_version:" "$f" 2>/dev/null; then
            EXAMPLE_FILES_WITH_VERSION+=("$f")
        fi
    done

    # Update proto @default annotation
    if [ -f "proto/robotops/config/v1/config.proto" ]; then
        sed -i '' "s/@default \"$CURRENT\"/@default \"$NEW_VERSION\"/g" proto/robotops/config/v1/config.proto
    fi

    # Regenerate code to update generated/yaml/default.yaml
    if command -v just &> /dev/null; then
        echo "Regenerating code..."
        just generate 2>/dev/null || echo "Warning: code generation failed"
    else
        echo "Warning: just not found - skipping code regeneration"
        echo "  Run manually: just generate"
    fi

    # Update CHANGELOG (add new section header if not exists)
    DATE=$(date +%Y-%m-%d)
    if ! grep -q "## \[$NEW_VERSION\]" CHANGELOG.md; then
        # Create new entry and insert after anchor comment
        awk -v version="$NEW_VERSION" -v date="$DATE" '
            /<!-- Versions below this line -->/ {
                print
                print ""
                print "## [" version "] - " date
                print ""
                print "### Changed"
                print ""
                print "- TODO: Describe changes"
                next
            }
            { print }
        ' CHANGELOG.md > CHANGELOG.md.tmp && mv CHANGELOG.md.tmp CHANGELOG.md
    fi

    echo "✅ Updated:"
    echo "  - VERSION"
    echo "  - proto/robotops/config/v1/config.proto (@default annotation)"
    echo "  - generated/yaml/default.yaml (regenerated)"
    echo "  - CHANGELOG.md (added section for $NEW_VERSION)"
    echo ""

    # Warn about example files that need manual updates
    if [ ${#EXAMPLE_FILES_WITH_VERSION[@]} -gt 0 ]; then
        echo "⚠️  Manual update required for example files:"
        for f in "${EXAMPLE_FILES_WITH_VERSION[@]}"; do
            echo "  - $f"
        done
        echo ""
        echo "  Update schema_version from \"$CURRENT\" to \"$NEW_VERSION\" in these files"
        echo ""
    fi

    echo "Next steps:"
    echo "  1. Update example YAML files (see warning above)"
    echo "  2. Verify: just validate-versions"
    echo "  3. Edit CHANGELOG.md to describe changes"
    echo "  4. Commit: git add -A && git commit -m 'chore: bump schema version to $NEW_VERSION'"
    echo "  5. Push: git push && git push --tags"

# Validate all schema_versions match VERSION file
validate-versions:
    #!/usr/bin/env bash
    set -euo pipefail

    EXPECTED=$(cat VERSION)
    EXIT_CODE=0

    echo "Expected version: $EXPECTED"
    echo ""

    # Check example YAML files (if they have schema_version)
    for f in examples/*.yaml; do
        if [ -f "$f" ] && grep -q '^schema_version:' "$f" 2>/dev/null; then
            ACTUAL=$(grep -E '^schema_version:' "$f" | sed 's/schema_version: *"\([^"]*\)"/\1/')
            if [ "$ACTUAL" != "$EXPECTED" ]; then
                echo "MISMATCH: $f has schema_version: \"$ACTUAL\""
                EXIT_CODE=1
            else
                echo "OK: $f"
            fi
        fi
    done

    # Check proto @default annotation for schema_version field
    if [ -f "proto/robotops/config/v1/config.proto" ]; then
        # Extract the @default value from the first occurrence (schema_version field in Config message)
        PROTO_VERSION=$(grep '@default "0' proto/robotops/config/v1/config.proto | head -1 | sed 's/.*@default "\([^"]*\)".*/\1/')
        if [ -z "$PROTO_VERSION" ]; then
            echo "ERROR: Could not extract proto schema_version @default"
            EXIT_CODE=1
        elif [ "$PROTO_VERSION" != "$EXPECTED" ]; then
            echo "MISMATCH: proto/robotops/config/v1/config.proto has @default \"$PROTO_VERSION\""
            EXIT_CODE=1
        else
            echo "OK: proto/robotops/config/v1/config.proto"
        fi
    fi

    # Check generated YAML
    if [ -f "generated/yaml/default.yaml" ]; then
        GEN_VERSION=$(grep -E '^schema_version:' generated/yaml/default.yaml | sed 's/schema_version: *"\([^"]*\)"/\1/')
        if [ "$GEN_VERSION" != "$EXPECTED" ]; then
            echo "MISMATCH: generated/yaml/default.yaml has schema_version: \"$GEN_VERSION\""
            echo "  (Run: just generate)"
            EXIT_CODE=1
        else
            echo "OK: generated/yaml/default.yaml"
        fi
    fi

    echo ""
    if [ $EXIT_CODE -eq 0 ]; then
        echo "All versions match: $EXPECTED"
    else
        echo "Version mismatches found! Expected: $EXPECTED"
        echo ""
        echo "To fix:"
        echo "  1. Update VERSION file: just bump-version patch|minor|major"
        echo "  2. Update proto: edit proto/robotops/config/v1/config.proto @default annotation"
        echo "  3. Regenerate: just generate"
    fi

    exit $EXIT_CODE

# Validate example YAML files conform to protobuf schema
validate-examples:
    #!/usr/bin/env bash
    set -euo pipefail

    # Check if generated Python protobuf exists
    if [ ! -f "generated/sdks/python/robotops/config/v1/config_pb2.py" ]; then
        echo "Error: Python protobuf files not found. Run 'just generate' first."
        exit 1
    fi

    # Check for required Python packages
    if ! python3 -c "import google.protobuf" 2>/dev/null; then
        echo "Error: protobuf Python package not found."
        echo "Install with: pip3 install protobuf pyyaml"
        exit 1
    fi

    if ! python3 -c "import yaml" 2>/dev/null; then
        echo "Error: pyyaml Python package not found."
        echo "Install with: pip3 install pyyaml"
        exit 1
    fi

    # Run validation script
    python3 tools/validate-examples.py

# Run all validations using Docker (recommended - ensures consistency)
validate: validate-versions
    docker build -f Dockerfile.validate -t robotops-config-validate .
    docker run --rm -v {{justfile_directory()}}:/workspace robotops-config-validate

# Run validations natively (requires yamllint installed locally)
validate-native: validate-versions validate-examples
    #!/usr/bin/env bash
    set -euo pipefail

    # Check for required tools
    if ! command -v yamllint &> /dev/null; then
        echo "Error: yamllint not found. Install with: pip3 install --user yamllint"
        exit 1
    fi

    # Run YAML linting
    yamllint generated/yaml/
    yamllint examples/
