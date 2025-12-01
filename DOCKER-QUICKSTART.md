# RynnMotion Docker Quick Start Guide

> ⚠️ **Note:** Docker setup is experimental and untested. For production use, see [native installation](README.md#-installation).

> **Full Docker Documentation:** See [docs/DOCKER-SETUP.md](docs/DOCKER-SETUP.md)

---

Get started with RynnMotion in Docker in under 5 minutes!

## Prerequisites

Install Docker Desktop:
- **macOS**: `brew install --cask docker` or download from [docker.com](https://www.docker.com/products/docker-desktop)
- **Linux**: `sudo apt install docker.io docker-compose` (Ubuntu/Debian)
- **Windows**: Download Docker Desktop from [docker.com](https://www.docker.com/products/docker-desktop)

## 3-Step Setup

### 1. Build the Environment

```bash
./scripts/docker-build.sh
```

This will take 15-30 minutes on first run (downloads and compiles all dependencies).

### 2. Start the Container

```bash
./scripts/docker-run.sh
```

You're now inside the container with all dependencies installed!

### 3. Build RynnMotion

Inside the container:

```bash
# Build C++ libraries
mkdir -p build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j$(nproc)

# Install Python package
cd /workspace/python
python3 -m venv venv
source venv/bin/activate
pip install -e ".[dev]"
```

## Test It Works

```bash
# Test MuJoCo viewer (requires GUI setup - see below)
python scripts/pinkine_viewer.py --robot fr3

# Or test without GUI
python -c "from RynnMotion.manager.robot_manager import RobotManager; print('Success!')"
```

## GUI Support (Optional)

### macOS

```bash
# Install XQuartz
brew install --cask xquartz

# Open XQuartz and enable network connections in Preferences → Security
open -a XQuartz

# Allow connections
xhost + $(ifconfig en0 | grep inet | awk '$1=="inet" {print $2}')
```

### Linux

```bash
# Allow Docker to connect
xhost +local:docker
```

## Alternative: One-Command Setup

Using the Makefile:

```bash
# Complete setup (build image + build project + install Python)
make -f Makefile.docker setup

# Enter container
make -f Makefile.docker shell
```

Create an alias for convenience:

```bash
echo "alias dmake='make -f Makefile.docker'" >> ~/.bashrc
source ~/.bashrc

# Now you can use:
dmake build
dmake run
dmake shell
```

## Common Commands

```bash
# Enter running container
./scripts/docker-exec.sh

# Execute a command
./scripts/docker-exec.sh python script.py

# Stop container
docker-compose down

# View logs
docker-compose logs -f

# Rebuild from scratch
./scripts/docker-build.sh --no-cache
```

## What's Next?

- **Full Docker documentation**: See [docs/DOCKER-SETUP.md](docs/DOCKER-SETUP.md)
- **Quick Start Tutorial**: See [tutorials/quickstart.md](tutorials/quickstart.md)
- **RynnMotion usage**: See [CLAUDE.md](CLAUDE.md)
- **Troubleshooting**: See [docs/DOCKER-SETUP.md](docs/DOCKER-SETUP.md#troubleshooting)

## Quick Reference

| Task | Command |
|------|---------|
| Build image | `./scripts/docker-build.sh` |
| Start container | `./scripts/docker-run.sh` |
| Enter container | `./scripts/docker-exec.sh` |
| Stop container | `docker-compose down` |
| Build project | Inside container: `mkdir build && cd build && cmake .. && make` |
| Python install | Inside container: `cd python && pip install -e .` |

## Disk Space

The Docker setup uses approximately:
- **Image size**: 5-8 GB
- **Build cache**: 2-3 GB
- **Container**: Minimal (shares files with host)

**Total**: ~8-12 GB

## Performance

Docker container performance is approximately:
- **Linux**: 95-100% of native
- **macOS**: 80-90% of native (due to virtualization)
- **Windows (WSL2)**: 85-95% of native

For development, this is perfectly acceptable!

## Need Help?

- Docker issues: Check [docs/DOCKER-SETUP.md](docs/DOCKER-SETUP.md#troubleshooting)
- RynnMotion questions: Check [CLAUDE.md](CLAUDE.md)
- Can't solve it: Open an issue on GitHub

Happy coding! 🚀
