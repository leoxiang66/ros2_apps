#!/bin/bash

# Define the workspace directory
WORKSPACE_DIR=$(pwd)

# Define directories to clean
BUILD_DIR="$WORKSPACE_DIR/build"
INSTALL_DIR="$WORKSPACE_DIR/install"
LOG_DIR="$WORKSPACE_DIR/log"

# Function to remove a directory
clean_directory() {
    local DIR_PATH=$1
    if [ -d "$DIR_PATH" ]; then
        echo "Removing $DIR_PATH..."
        rm -rf "$DIR_PATH"
        echo "$DIR_PATH removed."
    else
        echo "$DIR_PATH does not exist."
    fi
}

echo "Starting clean process..."

# Clean build, install, and log directories
clean_directory "$BUILD_DIR"
clean_directory "$INSTALL_DIR"
clean_directory "$LOG_DIR"

echo "Clean process completed."