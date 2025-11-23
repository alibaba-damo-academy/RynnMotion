#!/bin/bash

set -e

DEFAULT_VERSION="3.3.7"
MUJOCO_VERSION="${1:-$DEFAULT_VERSION}"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo "=========================================================="
echo "MuJoCo Installation/Upgrade Script"
echo "Target version: ${MUJOCO_VERSION}"
echo "=========================================================="
echo ""

# Detect OS
if [[ "$OSTYPE" == "darwin"* ]]; then
    OS="macos"
    PLATFORM="macos-universal2"
    LIB_EXT="dylib"
    echo "Detected OS: macOS"
elif [[ "$OSTYPE" == "linux-gnu"* ]]; then
    OS="linux"
    PLATFORM="linux-x86_64"
    LIB_EXT="so"
    echo "Detected OS: Linux"
else
    echo -e "${RED}✗ Error: Unsupported OS: $OSTYPE${NC}"
    exit 1
fi

echo ""

MUJOCO_DIR="/usr/local/mujoco-${MUJOCO_VERSION}"
if [ -d "$MUJOCO_DIR" ]; then
    echo -e "${YELLOW}MuJoCo ${MUJOCO_VERSION} is already installed at: ${MUJOCO_DIR}${NC}"
    read -p "Do you want to reinstall it? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo "Installation cancelled."
        exit 0
    fi
    echo "Proceeding with reinstallation..."
    sudo rm -rf "$MUJOCO_DIR"
fi

if [ "$OS" == "macos" ]; then
    DOWNLOAD_URL="https://github.com/google-deepmind/mujoco/releases/download/${MUJOCO_VERSION}/mujoco-${MUJOCO_VERSION}-${PLATFORM}.dmg"
    DOWNLOAD_FILE="/tmp/mujoco-${MUJOCO_VERSION}-${PLATFORM}.dmg"
    DMG_MOUNT_PATH="/Volumes/MuJoCo"
else
    DOWNLOAD_URL="https://github.com/google-deepmind/mujoco/releases/download/${MUJOCO_VERSION}/mujoco-${MUJOCO_VERSION}-${PLATFORM}.tar.gz"
    DOWNLOAD_FILE="/tmp/mujoco-${MUJOCO_VERSION}-${PLATFORM}.tar.gz"
fi

echo ""
echo "Download URL: $DOWNLOAD_URL"
echo "Installing to: $MUJOCO_DIR"
echo ""

echo "Downloading MuJoCo ${MUJOCO_VERSION}..."
if curl -L -f -o "$DOWNLOAD_FILE" "$DOWNLOAD_URL"; then
    echo -e "${GREEN}✓ Download completed successfully${NC}"
else
    echo -e "${RED}✗ Error: Download failed${NC}"
    echo "Please check if version ${MUJOCO_VERSION} exists at:"
    echo "https://github.com/google-deepmind/mujoco/releases"
    exit 1
fi

echo ""

if [ "$OS" == "macos" ]; then
    echo "Mounting DMG..."
    hdiutil attach "$DOWNLOAD_FILE" -quiet

    if [ ! -d "$DMG_MOUNT_PATH" ]; then
        echo -e "${RED}✗ Error: Failed to mount DMG${NC}"
        exit 1
    fi

    echo -e "${GREEN}✓ DMG mounted successfully${NC}"

    if [ -d "$DMG_MOUNT_PATH/mujoco.framework" ]; then
        echo "Installing MuJoCo ${MUJOCO_VERSION} permanently to: $MUJOCO_DIR"

        sudo mkdir -p "$MUJOCO_DIR"
        sudo chown $(whoami):staff "$MUJOCO_DIR" 2>/dev/null || sudo chown $(whoami):$(id -gn) "$MUJOCO_DIR"

        echo "Copying MuJoCo framework..."
        mkdir -p "$MUJOCO_DIR/lib"
        mkdir -p "$MUJOCO_DIR/include"

        if ! cp "$DMG_MOUNT_PATH/mujoco.framework/Versions/A/"*.dylib "$MUJOCO_DIR/lib/" 2>/dev/null; then
            echo -e "${RED}✗ Error: Failed to copy library files${NC}"
            echo "Contents of framework:"
            ls -la "$DMG_MOUNT_PATH/mujoco.framework/Versions/A/"
            hdiutil detach "$DMG_MOUNT_PATH" -quiet 2>/dev/null || true
            exit 1
        fi

        if ! cp -R "$DMG_MOUNT_PATH/mujoco.framework/Versions/A/Headers/"* "$MUJOCO_DIR/include/" 2>/dev/null; then
            echo -e "${RED}✗ Error: Failed to copy header files${NC}"
            hdiutil detach "$DMG_MOUNT_PATH" -quiet 2>/dev/null || true
            exit 1
        fi

        INSTALLED_LIB=$(ls "$MUJOCO_DIR/lib/libmujoco"*.dylib 2>/dev/null | head -n 1)
        if [ -n "$INSTALLED_LIB" ] && [ -d "$MUJOCO_DIR/include" ] && [ -f "$MUJOCO_DIR/include/mujoco.h" ]; then
            echo -e "${GREEN}✓ MuJoCo ${MUJOCO_VERSION} installed successfully!${NC}"
            echo "  Library: $INSTALLED_LIB"
            echo "  Headers: $MUJOCO_DIR/include/"
        else
            echo -e "${RED}✗ Error: MuJoCo installation failed.${NC}"
            echo "Library files found:"
            ls -la "$MUJOCO_DIR/lib/" 2>/dev/null || echo "  No lib directory"
            echo "Header files found:"
            ls -la "$MUJOCO_DIR/include/" 2>/dev/null || echo "  No include directory"
            hdiutil detach "$DMG_MOUNT_PATH" -quiet 2>/dev/null || true
            exit 1
        fi

        echo ""
        echo "Cleaning up..."
        hdiutil detach "$DMG_MOUNT_PATH" -quiet 2>/dev/null || true
        rm -f "$DOWNLOAD_FILE"
        echo -e "${GREEN}✓ Cleanup completed${NC}"
    else
        echo -e "${RED}✗ Error: MuJoCo framework not found at $DMG_MOUNT_PATH/mujoco.framework${NC}"
        hdiutil detach "$DMG_MOUNT_PATH" -quiet 2>/dev/null || true
        exit 1
    fi
else
    echo "Extracting MuJoCo ${MUJOCO_VERSION}..."

    TEMP_DIR="/tmp/mujoco-${MUJOCO_VERSION}-extract"
    rm -rf "$TEMP_DIR"
    mkdir -p "$TEMP_DIR"

    tar -xzf "$DOWNLOAD_FILE" -C "$TEMP_DIR"

    EXTRACTED_DIR=$(find "$TEMP_DIR" -mindepth 1 -maxdepth 1 -type d -name "mujoco-*" | head -n 1)

    if [ -z "$EXTRACTED_DIR" ]; then
        echo -e "${RED}✗ Error: Failed to find extracted MuJoCo directory${NC}"
        echo "Contents of $TEMP_DIR:"
        ls -la "$TEMP_DIR"
        exit 1
    fi

    echo "Found extracted directory: $EXTRACTED_DIR"
    echo "Installing MuJoCo ${MUJOCO_VERSION} to: $MUJOCO_DIR"

    if [ ! -d "$EXTRACTED_DIR/lib" ]; then
        echo -e "${RED}✗ Error: Expected lib/ directory not found in extracted archive${NC}"
        echo "Contents of extracted directory:"
        ls -la "$EXTRACTED_DIR"
        exit 1
    fi

    if [ -d "$MUJOCO_DIR" ]; then
        sudo rm -rf "$MUJOCO_DIR"
    fi

    sudo mkdir -p "$(dirname "$MUJOCO_DIR")"
    sudo mv "$EXTRACTED_DIR" "$MUJOCO_DIR"

    sudo chown -R root:root "$MUJOCO_DIR"
    sudo chmod -R 755 "$MUJOCO_DIR"

    echo ""
    echo "Verifying installation..."
    echo "Checking for library files in: $MUJOCO_DIR/lib/"
    ls -la "$MUJOCO_DIR/lib/" | grep libmujoco || true

    if [ -f "$MUJOCO_DIR/lib/libmujoco.${LIB_EXT}" ] || [ -f "$MUJOCO_DIR/lib/libmujoco.${MUJOCO_VERSION}.${LIB_EXT}" ] || [ -f "$MUJOCO_DIR/lib/libmujoco.so.${MUJOCO_VERSION}" ]; then
        echo -e "${GREEN}✓ MuJoCo ${MUJOCO_VERSION} installed successfully!${NC}"
        echo "  Installation directory: $MUJOCO_DIR"
        echo "  Library: $MUJOCO_DIR/lib/"
        echo "  Headers: $MUJOCO_DIR/include/"
    else
        echo -e "${RED}✗ Error: MuJoCo installation failed.${NC}"
        echo "Library file not found in $MUJOCO_DIR/lib/"
        echo "Contents of lib directory:"
        ls -la "$MUJOCO_DIR/lib/" || echo "lib directory does not exist"
        exit 1
    fi

    echo ""
    echo "Cleaning up..."
    rm -rf "$TEMP_DIR"
    rm -f "$DOWNLOAD_FILE"
    echo -e "${GREEN}✓ Cleanup completed${NC}"
fi

echo ""
echo "=========================================================="
echo -e "${GREEN}MuJoCo ${MUJOCO_VERSION} installation completed!${NC}"
echo "=========================================================="
echo ""
echo "Next steps:"
echo "1. The CMake build system will automatically detect the new version"
echo "2. Clean your build directory: rm -rf build/*"
echo "3. Rebuild your project: cd build && cmake .. && make"
echo ""

echo "Checking for other MuJoCo installations..."
OTHER_VERSIONS=$(ls -d /usr/local/mujoco-* 2>/dev/null | grep -v "mujoco-${MUJOCO_VERSION}" || true)
if [ -n "$OTHER_VERSIONS" ]; then
    echo ""
    echo "Found other MuJoCo versions:"
    echo "$OTHER_VERSIONS"
    echo ""
    echo "You may want to remove old versions to save space:"
    echo "  sudo rm -rf /usr/local/mujoco-<old_version>"
fi

echo ""
echo "Installation complete!"
