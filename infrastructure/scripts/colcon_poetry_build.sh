#!/bin/bash
# infrastructure/scripts/colcon_poetry_build.sh
# Purpose: Build shim that runs colcon inside Poetry virtual environment
# Requirement: ADR-002 (Poetry + Colcon Integration)

set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_ROOT="$(dirname "$(dirname "$SCRIPT_DIR")")"

# Check if Poetry is available
if ! command -v poetry &> /dev/null; then
    echo "Error: Poetry is not installed or not in PATH" >&2
    exit 1
fi

# Navigate to backend directory
cd "$PROJECT_ROOT/backend"

# Ensure poetry.lock and venv exist
echo "Setting up Poetry environment..."
poetry install --no-root

# Check if ROS 2 is available (either native or from Docker)
echo "Verifying ROS 2 environment..."
if ! poetry run ros2 --version &> /dev/null; then
    echo "Warning: ROS 2 not found in Poetry environment. Building without ROS 2." >&2
fi

# Run colcon build inside poetry shell
echo "Running colcon build..."
poetry run colcon build --symlink-install

echo "Build complete!"
exit 0
