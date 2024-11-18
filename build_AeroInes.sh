#!/bin/bash

# Navigate to ./FlairWs/flair-build
TARGET_DIR="./FlairWs/flair-build"
BUILD_DIR="$TARGET_DIR/build"
FLAIR_SRC_DIR="./FlairWs/flair-src"

# Just to see the absolute path of FlairWs/flair-src
ABSOLUTE_FLAIR_SRC_DIR=$(realpath "$FLAIR_SRC_DIR")
echo "Absolute path of flair-src: $ABSOLUTE_FLAIR_SRC_DIR"

# Check if the TARGET_DIR exists
if [ ! -d "$TARGET_DIR" ]; then
  echo "Error: $TARGET_DIR does not exist."
  exit 1
fi

# Navigate to the target directory
cd "$TARGET_DIR" || exit 1

# Check if the /build folder exists
if [ ! -d "$BUILD_DIR" ]; then
  echo "Build directory does not exist. Creating it now..."

  # Execute the cmake_codelite_outofsource.sh script using the absolute path
  if [ -z "$FLAIR_ROOT" ]; then
    echo "Error: FLAIR_ROOT environment variable is not set."
    exit 1
  fi

  $FLAIR_ROOT/flair-src/scripts/cmake_codelite_outofsource.sh "$ABSOLUTE_FLAIR_SRC_DIR"
  if [ $? -eq 0 ]; then
    echo "Build directory created successfully."
  else
    echo "Error: Failed to create build directory."
    exit 1
  fi
else
  echo "Build directory already exists. Exiting."
  exit 0
fi
