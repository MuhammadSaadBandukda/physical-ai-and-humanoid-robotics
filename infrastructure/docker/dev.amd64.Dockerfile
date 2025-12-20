# infrastructure/docker/dev.amd64.Dockerfile
# Target: Simulated Environment (x86_64/amd64)
# Contains: ROS 2 Jazzy Desktop Full (Gazebo/Sim), Python 3.12 (via Ubuntu 24.04)

FROM osrf/ros:jazzy-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONUNBUFFERED=1

# 1. System Dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-venv \
    curl \
    build-essential \
    git \
    && rm -rf /var/lib/apt/lists/*

# 2. Install Poetry
ENV POETRY_HOME="/opt/poetry" \
    POETRY_VIRTUALENVS_IN_PROJECT=true \
    POETRY_NO_INTERACTION=1
ENV PATH="$POETRY_HOME/bin:$PATH"
RUN curl -sSL https://install.python-poetry.org | python3 -

# 3. Workspace Setup
WORKDIR /workspace/backend

# 4. Dependency Resolution
# CRITICAL: We copy ONLY pyproject.toml. We do NOT copy the host's poetry.lock
# because it was generated on a bleeding-edge Python 3.14 host and is likely invalid.
COPY backend/pyproject.toml .

# 5. Install Dependencies
# We run 'poetry lock' explicitly to generate a valid lockfile for THIS environment (Linux/Py3.12)
RUN poetry lock && poetry install --no-root

# 6. Verify ROS 2 is available
RUN ros2 --version
