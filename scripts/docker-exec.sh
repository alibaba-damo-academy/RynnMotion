#!/bin/bash
# RynnMotion Docker Exec Script - Execute commands in running container

CONTAINER_NAME="rynnmotion-dev"

# Check if container is running
if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "Error: Container '${CONTAINER_NAME}' is not running."
    echo ""
    echo "Start it with:"
    echo "  ./docker-run.sh"
    echo "  OR"
    echo "  docker-compose up -d rynnmotion-dev"
    exit 1
fi

# If no arguments, start interactive bash
if [ $# -eq 0 ]; then
    echo "Entering interactive shell in container..."
    docker exec -it "$CONTAINER_NAME" /bin/bash
else
    # Execute the provided command
    echo "Executing: $*"
    docker exec -it "$CONTAINER_NAME" "$@"
fi
