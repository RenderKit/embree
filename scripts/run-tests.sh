#!/bin/bash
# Run Embree tests (integration or CTest)
set -e

# Default values
TEST_TYPE="ctest"
BUILD_TYPE="Release"
C_COMPILER=""
CXX_COMPILER=""
EMBREE_DIR=""
CTEST_CONFIG="Release"

# Parse command line arguments
while [[ $# -gt 0 ]]; do
  case $1 in
    --type)
      TEST_TYPE="$2"
      shift 2
      ;;
    --build-type)
      BUILD_TYPE="$2"
      shift 2
      ;;
    --c-compiler)
      C_COMPILER="$2"
      shift 2
      ;;
    --cxx-compiler)
      CXX_COMPILER="$2"
      shift 2
      ;;
    --embree-dir)
      EMBREE_DIR="$2"
      shift 2
      ;;
    --ctest-config)
      CTEST_CONFIG="$2"
      shift 2
      ;;
    *)
      echo "Unknown argument: $1"
      exit 1
      ;;
  esac
done

echo "Running tests..."
echo "Test type: $TEST_TYPE"
echo "Build type: $BUILD_TYPE"
echo "Embree directory: $EMBREE_DIR"

if [[ "$TEST_TYPE" == "integration" ]]; then
  echo "Running integration tests..."
  
  if [[ -z "$EMBREE_DIR" ]]; then
    echo "Error: EMBREE_DIR not specified for integration tests"
    exit 1
  fi
  
  cd tests/integration/test_embree_release
  
  # Build cmake command with compiler options
  CMAKE_OPTS="-B build -DCMAKE_BUILD_TYPE=$BUILD_TYPE"
  
  if [[ -n "$C_COMPILER" ]]; then
    CMAKE_OPTS+=" -DCMAKE_C_COMPILER=$C_COMPILER"
  fi
  
  if [[ -n "$CXX_COMPILER" ]]; then
    CMAKE_OPTS+=" -DCMAKE_CXX_COMPILER=$CXX_COMPILER"
  fi
  
  # Use absolute path for embree_DIR
  if [[ "$EMBREE_DIR" = /* ]]; then
    # Already absolute path
    CMAKE_OPTS+=" -Dembree_DIR=$EMBREE_DIR"
  else
    # Relative path, make it absolute from original directory
    ROOT_DIR="$PWD/../../../"
    CMAKE_OPTS+=" -Dembree_DIR=$ROOT_DIR/$EMBREE_DIR"
  fi
  
  echo "CMake command: cmake $CMAKE_OPTS"
  cmake $CMAKE_OPTS
  cmake --build build --config "$BUILD_TYPE"
  ./build/test
  
elif [[ "$TEST_TYPE" == "ctest" ]]; then
  echo "Running CTest..."
  
  if [[ -z "$EMBREE_DIR" ]]; then
    echo "Error: EMBREE_DIR not specified for CTest"
    exit 1
  fi
  
  cd "$EMBREE_DIR"
  cmake -S testing -B build
  cd build
  ctest -C "$CTEST_CONFIG" --output-on-failure -VV
  
else
  echo "Error: Unknown test type '$TEST_TYPE'. Use 'integration' or 'ctest'"
  exit 1
fi

echo "Tests completed successfully"