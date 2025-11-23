# RynnMotion Docker Setup (Experimental)

> ⚠️ **Experimental Status:** This Docker setup has not been fully tested. For production use, please use the [native installation method](installation.md).
>
> We welcome testing feedback! Please report issues at [GitHub Issues](https://github.com/alibaba-damo-academy/RynnMotion/issues).

---

This guide provides a complete Docker-based development environment for RynnMotion, eliminating the need to manually install dependencies on your host system.

## Quick Start

### Prerequisites

- Docker (>= 20.10)
- Docker Compose (>= 2.0)
- (Optional) XQuartz on macOS or X11 on Linux for GUI applications

### Build and Run

```bash
# 1. Build the Docker image
./scripts/docker-build.sh

# 2. Run the container and enter interactive shell
./scripts/docker-run.sh

# 3. Inside the container, build RynnMotion
cd /workspace
mkdir -p build && cd build
cmake ..
make -j$(nproc)
```

## What's Included

The Docker image includes all dependencies specified in `setup_dependencies.sh`:

### System Libraries
- **Eigen** (>= 3.3.0) - Linear algebra
- **Boost** (>= 1.65) - C++ libraries
- **Pinocchio** (v3.7.0) - Rigid body dynamics
- **MuJoCo** (3.3.5) - Physics simulation
- **yaml-cpp** - YAML parsing
- **qpOASES** - Quadratic programming
- **LCM** - Lightweight communications
- **FCL** - Flexible collision library
- **nlohmann/json** - JSON parsing
- **Ruckig** (v0.15.3) - Motion generation
- **OpenCV** - Computer vision

### Development Tools
- GCC/G++ compiler toolchain
- CMake, Ninja build systems
- Git, vim, nano
- GDB, Valgrind for debugging
- Python 3 with pip
- Clang-format, clang-tidy for code quality

## Usage

### Helper Scripts

#### `scripts/docker-build.sh`
Builds the Docker image.

```bash
./scripts/docker-build.sh              # Build with cache
./scripts/docker-build.sh --no-cache   # Rebuild from scratch
```

#### `scripts/docker-run.sh`
Starts and enters the container with proper GUI support.

```bash
./scripts/docker-run.sh
```

Features:
- Auto-configures X11 forwarding for MuJoCo viewer
- Mounts project directory to `/workspace`
- Handles container lifecycle (create/restart/attach)

#### `scripts/docker-exec.sh`
Executes commands in a running container.

```bash
./scripts/docker-exec.sh               # Enter interactive shell
./scripts/docker-exec.sh ls -la        # Run specific command
./scripts/docker-exec.sh bash          # Start new bash session
```

### Using Docker Compose Directly

```bash
# Start container in background
docker-compose up -d rynnmotion-dev

# Enter container
docker exec -it rynnmotion-dev /bin/bash

# View logs
docker-compose logs -f rynnmotion-dev

# Stop container
docker-compose down

# Rebuild image
docker-compose build --no-cache
```

### Optional: Jupyter Notebook

For data analysis and visualization:

```bash
# Start Jupyter Lab
docker-compose --profile jupyter up -d rynnmotion-jupyter

# Access at http://localhost:8888
```

## GUI Applications (MuJoCo Viewer)

### Linux

```bash
# Allow Docker to connect to X server
xhost +local:docker

# Run container
./scripts/docker-run.sh
```

### macOS

1. Install XQuartz:
   ```bash
   brew install --cask xquartz
   ```

2. Configure XQuartz:
   - Open XQuartz
   - Go to Preferences → Security
   - Enable "Allow connections from network clients"
   - Restart XQuartz

3. Allow connections:
   ```bash
   xhost + $(ifconfig en0 | grep inet | awk '$1=="inet" {print $2}')
   ```

4. Run container:
   ```bash
   ./scripts/docker-run.sh
   ```

### Windows (WSL2)

1. Install VcXsrv or X410
2. Configure X server to allow connections
3. Set DISPLAY environment variable in WSL2:
   ```bash
   export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0
   ```

## Hardware Access

### USB Devices (Robots, Sensors)

Edit `docker-compose.yml` to add devices:

```yaml
devices:
  - /dev/ttyUSB0:/dev/ttyUSB0  # Serial device
  - /dev/video0:/dev/video0     # Camera
```

### GPU Support (NVIDIA)

Install NVIDIA Container Toolkit, then:

```yaml
services:
  rynnmotion-dev:
    runtime: nvidia  # Add this line
```

Or use Docker command:
```bash
docker run --gpus all ...
```

## Development Workflow

### 1. Build RynnMotion C++ Libraries

```bash
# Inside container
cd /workspace
mkdir -p build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j$(nproc)
sudo make install
```

### 2. Install Python Package

```bash
cd /workspace/python
python3 -m venv venv
source venv/bin/activate
pip install -e ".[dev]"
```

### 3. Run Examples

```bash
# Test MuJoCo viewer
python scripts/pinkine_viewer.py --robot fr3

# Run dataset recording
cd /workspace/python/src/RynnMotion/RynnDatasets
python run_dataset.py
```

### 4. Test Hardware (in Satellites)

```bash
cd /workspace/robots/RynnLeRobot
python3 -m venv venv
source venv/bin/activate
pip install -e .

# Make sure hardware is connected and mounted
python scripts/run_lerobot.py --robot so101 --mode record
```

## File Structure

```
RynnMotion/
├── Dockerfile              # Multi-stage Docker image
├── docker-compose.yml      # Container orchestration
├── .dockerignore           # Build context optimization
├── Makefile.docker         # Make-based workflow
├── scripts/
│   ├── docker-build.sh     # Build helper script
│   ├── docker-run.sh       # Run helper script
│   ├── docker-exec.sh      # Exec helper script
│   ├── setup_dependencies.sh  # Native dependency installation
│   └── upgrade_mujoco.sh   # MuJoCo upgrade utility
└── docs/
    └── DOCKER-SETUP.md     # This file
```

## Volumes

The setup uses Docker volumes for persistence:

- **rynnmotion-build**: Persists build artifacts across restarts
- **rynnmotion-venv**: Persists Python virtual environment
- **Bind mount (.)**: Live code editing from host

## Troubleshooting

### GUI doesn't work

**Linux:**
```bash
xhost +local:docker
export DISPLAY=:0
```

**macOS:**
```bash
# Get your IP
IP=$(ifconfig en0 | grep inet | awk '$1=="inet" {print $2}')
xhost + $IP
export DISPLAY=$IP:0
```

### Container won't start

```bash
# Check logs
docker-compose logs rynnmotion-dev

# Remove and recreate
docker-compose down
docker-compose up -d rynnmotion-dev
```

### Build fails

```bash
# Clean rebuild
docker-compose down
docker volume rm rynnmotion-build rynnmotion-venv
./docker-build.sh --no-cache
```

### Permission issues

The container runs as user `developer` (UID 1000). If your host UID is different:

```bash
# Rebuild with your UID
docker-compose build --build-arg USER_UID=$(id -u) --build-arg USER_GID=$(id -g)
```

### MuJoCo not found

```bash
# Verify installation inside container
ls -la /usr/local/mujoco-3.3.5
echo $LD_LIBRARY_PATH
```

## Advanced Usage

### Multi-stage Builds

The Dockerfile has multiple stages:

- **base**: System dependencies
- **builder**: Compiles all dependencies
- **development**: Final dev environment

Build specific stages:
```bash
docker build --target base -t rynnmotion:base .
docker build --target builder -t rynnmotion:builder .
```

### Custom CMake Configuration

```bash
# Inside container
cd /workspace/build
cmake -DCMAKE_BUILD_TYPE=Debug \
      -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
      -DBUILD_TESTING=ON \
      ..
make -j$(nproc)
```

### Debugging with GDB

```bash
# Build with debug symbols
cmake -DCMAKE_BUILD_TYPE=Debug ..
make -j$(nproc)

# Run with GDB
gdb ./your_executable
```

### Code Formatting

```bash
# Format all C++ files
find /workspace -name "*.cpp" -o -name "*.hpp" | xargs clang-format -i

# Check formatting
clang-format --dry-run --Werror src/**/*.cpp
```

## Performance Considerations

### Build Speed

- **Parallel builds**: `make -j$(nproc)` uses all CPU cores
- **Ninja**: Faster than Make: `cmake -GNinja ..`
- **ccache**: Add to Dockerfile for incremental builds

### Container Resources

Set in Docker Desktop or daemon.json:
```json
{
  "cpus": 8,
  "memory": "16GB",
  "swap": "4GB"
}
```

## CI/CD Integration

### GitHub Actions

```yaml
jobs:
  build:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      - name: Build Docker image
        run: docker-compose build rynnmotion-dev
      - name: Run tests
        run: docker-compose run rynnmotion-dev make test
```

### GitLab CI

```yaml
build:
  image: docker:latest
  services:
    - docker:dind
  script:
    - docker-compose build
    - docker-compose run rynnmotion-dev make test
```

## Comparison: Docker vs Native

| Aspect | Docker | Native (`scripts/setup_dependencies.sh`) |
|--------|--------|-------------------------------------------|
| **Setup time** | 15-30 min (one-time) | 30-60 min (per machine) |
| **Reproducibility** | ✅ Identical across machines | ⚠️ Depends on host OS |
| **Isolation** | ✅ No conflicts | ⚠️ Can conflict with system libs |
| **Portability** | ✅ Linux/macOS/Windows (WSL2) | ⚠️ Linux/macOS only |
| **Performance** | ~95% native (Linux), ~85% (macOS) | ✅ 100% native |
| **Disk space** | ~5-8 GB image | ~2-3 GB dependencies |
| **GUI apps** | ⚠️ Requires X11 setup | ✅ Native display |
| **Hardware access** | ⚠️ Requires device mounting | ✅ Direct access |

## When to Use Docker

**Use Docker if:**
- You want quick, reproducible setup
- You work on multiple machines
- You want isolated development environments
- You're on macOS and want Linux toolchain
- You're contributing to RynnMotion and need consistent builds

**Use Native if:**
- You need absolute maximum performance
- You frequently use GUI applications
- You have complex hardware setups
- You're deploying to real robots (use Docker for dev, native for deployment)

## Support

For issues related to:
- **Docker setup**: Check this guide or open an issue
- **RynnMotion**: See main [README.md](../README.md) or [CLAUDE.md](../CLAUDE.md)
- **Dependencies**: Refer to [scripts/setup_dependencies.sh](../scripts/setup_dependencies.sh)
- **Native Installation**: See [installation.md](installation.md) for tested production setup

## License

Same as RynnMotion - Apache License 2.0

---

## Related Documentation

- **[Installation Guide](installation.md)** - Comprehensive native installation (recommended for production)
- **[Quick Start](../README.md#-quick-start)** - Get running quickly
- **[Architecture](ARCHITECTURE.md)** - Understand how RynnMotion works
