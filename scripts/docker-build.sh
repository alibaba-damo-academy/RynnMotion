#!/bin/bash
# RynnMotion Docker Build Script

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "=========================================="
echo "RynnMotion Docker Build"
echo "=========================================="
echo ""

# Parse arguments
BUILD_TARGET="development"
NO_CACHE=""

while [[ $# -gt 0 ]]; do
    case $1 in
        --no-cache)
            NO_CACHE="--no-cache"
            shift
            ;;
        --target)
            BUILD_TARGET="$2"
            shift 2
            ;;
        *)
            echo "Unknown option: $1"
            echo "Usage: $0 [--no-cache] [--target development|base|builder]"
            exit 1
            ;;
    esac
done

echo "Building RynnMotion Docker image..."
echo "  Target: $BUILD_TARGET"
echo "  Cache: $([ -n "$NO_CACHE" ] && echo "disabled" || echo "enabled")"
echo ""

# Build using docker-compose
docker-compose build $NO_CACHE rynnmotion-dev

echo ""
echo "=========================================="
echo "Build completed successfully!"
echo "=========================================="
echo ""
echo "Next steps:"
echo "  1. Run container:    ./docker-run.sh"
echo "  2. Or use compose:   docker-compose up -d rynnmotion-dev"
echo "  3. Execute commands: ./docker-exec.sh <command>"
echo ""
