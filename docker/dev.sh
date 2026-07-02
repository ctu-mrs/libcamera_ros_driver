#!/usr/bin/env bash
set -e

# This script builds and starts the development docker container.
# It relies on 'docker/.env' to set the user for the container.

# Absolute path to this script.
SCRIPT=$(readlink -f "$0")
# Absolute path this script is in.
SCRIPT_PATH=$(dirname "$SCRIPT")
# Path to project root.
ROOT_PATH=$(dirname "$SCRIPT_PATH")

cd "$ROOT_PATH"

echo "Building and starting the development container..."
if [ -f "docker/.env" ]; then
  echo "Using UID/GID from docker/.env. Please ensure it matches your user."
else
  echo "Warning: docker/.env not found. Using default UID/GID 1000. This may cause file permission issues."
  echo "Please create docker/.env with your user's UID and GID."
fi

# The docker-compose.dev.yaml is in the root of the project.
# It will automatically pick up the docker/.env file.
docker compose -f docker-compose.dev.yaml up -d --build

echo "Container started. Opening an interactive shell..."
echo "You can exit the shell with 'exit' and stop the container with 'docker/down.sh'."
echo "Build inside the container with: catkin build libcamera_ros_driver"

# Open an interactive shell in the running container
docker compose -f docker-compose.dev.yaml exec dev_shell bash
