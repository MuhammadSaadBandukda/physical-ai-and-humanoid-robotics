#!/bin/bash
# infrastructure/scripts/detect_ros_env.sh
# Purpose: Detect if Poetry virtual environment is active
# Returns: 0 if Poetry venv is active, 1 otherwise

# Check if VIRTUAL_ENV is set and contains "poetry"
if [[ -n "$VIRTUAL_ENV" && "$VIRTUAL_ENV" == *".venv"* ]]; then
    # Confirm we can run poetry
    if command -v poetry &> /dev/null; then
        exit 0
    fi
fi

# Alternative: Check if we're inside a Poetry shell
if poetry env info &> /dev/null; then
    exit 0
fi

exit 1
