# Version management for robotops_config

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

    # Update all config files
    find config -name "*.yaml" -exec sed -i '' "s/schema_version: \"$CURRENT\"/schema_version: \"$NEW_VERSION\"/" {} \;

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

    echo "Updated:"
    echo "  - VERSION"
    echo "  - config/*.yaml and config/examples/*.yaml"
    echo "  - CHANGELOG.md (added section for $NEW_VERSION)"
    echo ""
    echo "Next steps:"
    echo "  1. Edit CHANGELOG.md to describe changes"
    echo "  2. Commit: git add -A && git commit -m 'chore: bump schema version to $NEW_VERSION'"
    echo "  3. Tag: git tag v$NEW_VERSION"
    echo "  4. Push: git push && git push --tags"

# Validate all schema_versions match VERSION file
validate-versions:
    #!/usr/bin/env bash
    set -euo pipefail

    EXPECTED=$(cat VERSION)
    EXIT_CODE=0

    echo "Expected version: $EXPECTED"
    echo ""

    for f in config/*.yaml config/examples/*.yaml; do
        ACTUAL=$(grep -E '^schema_version:' "$f" | sed 's/schema_version: *"\([^"]*\)"/\1/')
        if [ "$ACTUAL" != "$EXPECTED" ]; then
            echo "MISMATCH: $f has schema_version: \"$ACTUAL\""
            EXIT_CODE=1
        else
            echo "OK: $f"
        fi
    done

    exit $EXIT_CODE

# Run all validations using Docker (recommended - ensures consistency)
validate: validate-versions
    docker build -f Dockerfile.validate -t robotops-config-validate .
    docker run --rm -v {{justfile_directory()}}:/workspace robotops-config-validate

# Run validations natively (requires ajv-cli and yamllint installed locally)
validate-native: validate-versions
    #!/usr/bin/env bash
    set -euo pipefail

    # Check for required tools
    if ! command -v ajv &> /dev/null; then
        echo "Error: ajv-cli not found. Install with: npm install -g ajv-cli"
        exit 1
    fi
    if ! command -v yamllint &> /dev/null; then
        echo "Error: yamllint not found. Install with: pip3 install --user yamllint"
        exit 1
    fi

    # Run validations
    ajv validate -s schema/config.schema.json -d config/default.yaml
    ajv validate -s schema/config.schema.json -d config/minimal.yaml
    for f in config/examples/*.yaml; do
        ajv validate -s schema/config.schema.json -d "$f"
    done
    yamllint config/
