# RynnMotion Documentation

Welcome to the comprehensive RynnMotion documentation! This page serves as the central hub for all technical documentation.

## 📋 Documentation Index

### 🚀 Getting Started

Start here if you're new to RynnMotion:

- **[Quick Start Guide](../tutorials/quickstart.md)** - Get up and running in 5 minutes
- **[Installation Guide](installation.md)** - Comprehensive installation instructions for all platforms
- **[Tutorial Index](../tutorials/README.md)** - Step-by-step learning guides

### 🏗️ Core Documentation

Deep dive into RynnMotion's architecture and design:

- **[Architecture Overview](ARCHITECTURE.md)** - System design, module architecture, and data flow
  - Shared RuntimeData pattern
  - Module lifecycle and initialization
  - Control pipeline architecture
  - Auto-discovery system

- **[Core-Satellite Architecture](core-satellite-architecture.md)** - Project structure philosophy
  - Core library (C++ and Python)
  - Satellite robot-specific modules
  - Dependency management

### 🛠️ Development

Resources for contributors and developers:

- **[C++ Coding Style](c++_coding_style.md)** - Code conventions (v2.2, Nov 2025)
  - Naming conventions
  - File organization
  - Documentation standards
  - CMake guidelines

- **[Open Source Roadmap](OPEN_SOURCE_ROADMAP.md)** - Development plans and milestones
  - Release schedule
  - Feature priorities
  - Community goals

- **[Adding Robots](../models/how_to_add_new_robot_scene.md)** - How to integrate new robot models
  - Auto-discovery system
  - MJCF file requirements
  - Scene configuration

### 🐳 Environment Setup

Alternative development environments:

- **[Docker Setup](DOCKER-SETUP.md)** - Container development environment (experimental)
  - Quick start with Docker
  - GUI support (X11 forwarding)
  - Hardware access
  - Troubleshooting

### 📖 User Guides

Practical guides for common tasks:

- **Robot Control** (coming soon)
  - Writing custom controllers
  - OSC configuration
  - Multi-robot coordination

- **Data Collection** (coming soon)
  - LeRobot integration
  - Teleoperation setup
  - Dataset formats

- **Deployment** (coming soon)
  - Sim-to-real transfer
  - Hardware interfaces
  - Safety considerations

## 📚 External Resources

### API Documentation

- **[Python API Reference](https://rynnmotion.readthedocs.io)** - Full Python bindings documentation
- **[C++ API Reference](https://rynnmotion.readthedocs.io/cpp)** - C++ core library reference

### Community

- **[GitHub Repository](https://github.com/alibaba-damo-academy/RynnMotion)** - Source code and issue tracker
- **[GitHub Discussions](https://github.com/alibaba-damo-academy/RynnMotion/discussions)** - Ask questions and share ideas
- **[Contributing Guide](../CONTRIBUTING.md)** - How to contribute to RynnMotion

## 🗺️ Documentation Roadmap

### Current Status (v0.9.0-pre)

- ✅ Architecture documentation
- ✅ Coding style guide
- ✅ Quick start tutorial
- ✅ Docker setup guide
- 🚧 Installation guide (in progress)
- 📅 User guides (planned)

### Coming Soon

- **User Guides** - Task-specific tutorials (robot control, data collection, deployment)
- **API Documentation** - Complete Python and C++ API reference
- **Video Tutorials** - YouTube playlist with walkthroughs
- **Example Gallery** - Showcase of community projects

## 📖 Documentation Conventions

### Symbols Used

- ✅ **Stable** - Production-ready, fully tested
- ⚠️ **Experimental** - Working but not fully tested
- 🚧 **In Progress** - Actively being developed
- 📅 **Planned** - On the roadmap
- ❌ **Not Available** - Not currently supported

### Difficulty Levels

- ⭐ **Beginner** - No prior robotics knowledge required
- ⭐⭐ **Intermediate** - Basic robotics concepts helpful
- ⭐⭐⭐ **Advanced** - Requires solid understanding of robotics

### Document Types

- **Guides** - Step-by-step instructions for specific tasks
- **References** - Comprehensive documentation of APIs and features
- **Overviews** - High-level architectural and design documentation
- **Tutorials** - Learning-focused walkthroughs

## 🤝 Contributing to Documentation

Found a typo, broken link, or missing information? We welcome documentation contributions!

### How to Help

1. **Report issues** - Open an issue on GitHub for documentation bugs
2. **Suggest improvements** - Share ideas in GitHub Discussions
3. **Submit PRs** - Fix typos, add examples, or write new guides
4. **Share examples** - Contribute to the example gallery

### Documentation Style Guide

- Use clear, concise language
- Include code examples where applicable
- Add visual aids (diagrams, screenshots) when helpful
- Test all commands and code snippets
- Follow existing formatting conventions

See [Contributing Guide](../CONTRIBUTING.md) for detailed instructions.

## 📄 License

All documentation is released under Apache License 2.0, same as the RynnMotion codebase.

See [LICENSE](../LICENSE) for details.

---

## Quick Links

| Category | Links |
|----------|-------|
| **Getting Started** | [Quick Start](../tutorials/quickstart.md) • [Installation](installation.md) • [Examples](../examples/) |
| **Architecture** | [Overview](ARCHITECTURE.md) • [Core-Satellite](core-satellite-architecture.md) • [Coding Style](c++_coding_style.md) |
| **Development** | [Roadmap](OPEN_SOURCE_ROADMAP.md) • [Docker](DOCKER-SETUP.md) • [Adding Robots](../models/how_to_add_new_robot_scene.md) |
| **Community** | [GitHub](https://github.com/alibaba-damo-academy/RynnMotion) • [Contributing](../CONTRIBUTING.md) • [CLAUDE.md](../CLAUDE.md) |

---

<p align="center">
  <sub>Built with ❤️ by the RynnMotion Team</sub><br>
  <sub>Making robot manipulation research accessible to everyone</sub>
</p>
