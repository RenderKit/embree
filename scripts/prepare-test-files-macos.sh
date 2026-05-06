#!/bin/bash
# Prepare Embree test files from build artifacts (macOS)
set -e

# Default values - macOS uses ZIP files instead of TAR.GZ
DEST_DIR="./build/embree-release"
ARCHIVE_PATTERN="./build/*.zip"
TEST_TYPE="ctest"

# Parse command line arguments
while [[ $# -gt 0 ]]; do
  case $1 in
    --dest)
      DEST_DIR="$2"
      shift 2
      ;;
    --pattern)
      ARCHIVE_PATTERN="$2"
      shift 2
      ;;
    --type)
      TEST_TYPE="$2"
      shift 2
      ;;
    *)
      echo "Unknown argument: $1"
      exit 1
      ;;
  esac
done

echo "Preparing test files (macOS)..." >&2
echo "Destination: $DEST_DIR" >&2
echo "Archive pattern: $ARCHIVE_PATTERN" >&2
echo "Test type: $TEST_TYPE" >&2

# Create destination directory
mkdir -p "$DEST_DIR"

# Extract archives using unzip for macOS
if [[ "$TEST_TYPE" == "integration" ]]; then
  # For integration tests, extract to embree_install
  DEST_DIR="./build/embree_install"
  mkdir -p "$DEST_DIR"
  for archive in $ARCHIVE_PATTERN; do
    if [[ -f "$archive" ]]; then
      unzip -o "$archive" -d "$DEST_DIR" >&2
      echo "Extracted: $archive" >&2
    fi
  done
  
  # Find embree-config.cmake and export path
  EMBREE_CONFIG=$(find "$DEST_DIR" -name "embree-config.cmake" -print -quit)
  if [[ -n "$EMBREE_CONFIG" ]]; then
    EMBREE_DIR=$(dirname "$EMBREE_CONFIG")
    # Convert to absolute path for integration tests since they change directories
    EMBREE_DIR=$(cd "$EMBREE_DIR" && pwd)
    echo "EMBREE_DIR=$EMBREE_DIR" >> "$GITHUB_ENV"
    echo "Found embree-config.cmake at: $EMBREE_DIR" >&2
    echo "$EMBREE_DIR"  # Output for immediate use
  else
    echo "Error: embree-config.cmake not found" >&2
    exit 1
  fi
else
  # For CTest, extract to embree-release
  for archive in $ARCHIVE_PATTERN; do
    if [[ -f "$archive" ]]; then
      unzip -o "$archive" -d "$DEST_DIR" >&2
      echo "Extracted: $archive" >&2
    fi
  done
  echo "EMBREE_DIR=$DEST_DIR" >> "$GITHUB_ENV"
  echo "$DEST_DIR"  # Output for immediate use
fi

echo "Test files prepared successfully (macOS)" >&2