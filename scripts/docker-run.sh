#!/bin/bash
# RynnMotion Docker Run Script

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "=========================================="
echo "RynnMotion Docker Run"
echo "=========================================="
echo ""

# Check if container is already running
if docker ps -a --format '{{.Names}}' | grep -q '^rynnmotion-dev$'; then
    echo "Container 'rynnmotion-dev' already exists."
    echo ""
    read -p "Do you want to (r)estart, (a)ttach, or (d)elete and recreate? [r/a/d]: " -n 1 -r
    echo

    case $REPLY in
        r|R)
            echo "Restarting container..."
            docker restart rynnmotion-dev
            docker exec -it rynnmotion-dev /bin/bash
            ;;
        a|A)
            echo "Attaching to container..."
            docker exec -it rynnmotion-dev /bin/bash
            ;;
        d|D)
            echo "Removing and recreating container..."
            docker-compose down
            docker-compose up -d rynnmotion-dev
            docker exec -it rynnmotion-dev /bin/bash
            ;;
        *)
            echo "Invalid option. Exiting."
            exit 1
            ;;
    esac
else
    echo "Starting new container..."

    # Set up X11 forwarding for GUI applications (Linux/macOS)
    if [[ "$OSTYPE" == "darwin"* ]]; then
        echo "Detected macOS - Setting up XQuartz for GUI support..."
        echo "  Make sure XQuartz is installed and running:"
        echo "  brew install --cask xquartz"
        echo "  Open -a XQuartz"
        echo "  In XQuartz preferences, enable 'Allow connections from network clients'"
        echo ""

        # Get the IP address
        IP=$(ifconfig en0 | grep inet | awk '$1=="inet" {print $2}')
        if [ -z "$IP" ]; then
            IP="127.0.0.1"
        fi

        # Allow X11 connections
        xhost + "$IP" 2>/dev/null || echo "  Note: Run 'xhost + $IP' if GUI apps don't work"

        export DISPLAY="$IP:0"
    elif [[ "$OSTYPE" == "linux-gnu"* ]]; then
        echo "Detected Linux - Enabling X11 forwarding..."
        xhost +local:docker 2>/dev/null || echo "  Note: Run 'xhost +local:docker' if GUI apps don't work"
    fi

    echo ""
    echo "Starting RynnMotion development container..."
    docker-compose up -d rynnmotion-dev

    echo ""
    echo "Container started successfully!"
    echo "Entering interactive shell..."
    echo ""

    docker exec -it rynnmotion-dev /bin/bash
fi

echo ""
echo "=========================================="
echo "Container session ended"
echo "=========================================="
echo ""
echo "Useful commands:"
echo "  Re-enter container:  docker exec -it rynnmotion-dev /bin/bash"
echo "  Stop container:      docker-compose down"
echo "  View logs:           docker-compose logs -f rynnmotion-dev"
echo "  Execute command:     ./docker-exec.sh <command>"
echo ""
