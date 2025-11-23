# Core-Satellite Python Architecture Pattern for RynnMotion

## Overview

RynnMotion uses a **Core-Satellite Pattern** (Hub-and-Spoke) for managing a shared algorithmic core with multiple robot-specific implementations. This document outlines the current architecture, best practices, and refactoring recommendations.

## Current Architecture

### Structure Overview

```
RynnMotion/
├── python/                    # CORE: Main RynnMotion package
│   └── src/RynnMotion/       # Algorithms, simulation, datasets
│       ├── base/             # Base interfaces
│       ├── teleop/           # Teleoperation framework
│       ├── RynnDatasets/     # Dataset management
│       └── utils/            # Shared utilities
│
└── robots/                   # SATELLITES: Robot-specific submodules
    ├── RynnLeRobot/         # LeRobot implementation (own venv)
    ├── franka/              # Franka robots (own venv)
    ├── piper/               # Piper robots (own venv)
    └── realman/             # RealMan robots (own venv)
```

### Key Characteristics

1. **Core Package** (`python/`):
   - Contains shared algorithms, simulation, and dataset tools
   - Minimal dependencies
   - Hardware-agnostic

2. **Satellite Submodules** (`robots/*`):
   - Git submodules for specific robot implementations
   - Own virtual environments with hardware SDKs
   - Install core package via editable install: `pip install -e ../../python/`

3. **Dependency Flow**:
   - Satellites → Core (unidirectional)
   - Core never depends on satellites
   - Each satellite manages its own hardware SDKs

## Technical Implementation Details

### Current Setup Method

From `robots/RynnLeRobot/setup_lerobot.sh`:

```bash
# Install core package as editable without dependencies
uv pip install -e ../../python/ --no-deps

# Set PYTHONPATH for common modules
export PYTHONPATH="$(pwd)/../../../:$PYTHONPATH"

# Install remaining dependencies
uv pip install numpy scipy PyYAML matplotlib opencv-python ...
```

### Dependency Isolation Strategy

- Each robot submodule has isolated venv
- Core installed with `--no-deps` to avoid conflicts
- Hardware SDKs only in respective submodules
- PyTorch version flexibility (CPU vs CUDA)

## Refactoring Plan

### Phase 1: Improve Package Structure

#### 1.1 Convert to Namespace Packages

**Current:**
```
python/src/RynnMotion/
```

**Proposed:**
```python
python/src/
├── rynn_motion/           # Core namespace
│   ├── core/             # Core algorithms
│   ├── sim/              # Simulation
│   ├── datasets/         # Dataset management
│   └── __init__.py
└── rynn_robots/          # Robot implementations namespace
    └── __init__.py       # Namespace package marker
```

#### 1.2 Define Clear Interface Contracts

Create `python/src/rynn_motion/core/interfaces.py`:

```python
from abc import ABC, abstractmethod
from typing import Protocol, Dict, Any, Optional

class RobotInterface(Protocol):
    """Contract that all robot implementations must follow"""

    @abstractmethod
    def initialize(self, config: Dict[str, Any]) -> None:
        """Initialize robot with configuration"""
        ...

    @abstractmethod
    def get_state(self) -> Dict[str, Any]:
        """Get current robot state"""
        ...

    @abstractmethod
    def execute_command(self, command: Dict[str, Any]) -> bool:
        """Execute control command"""
        ...

    @abstractmethod
    def shutdown(self) -> None:
        """Clean shutdown of robot"""
        ...

class DatasetInterface(Protocol):
    """Contract for dataset implementations"""

    @abstractmethod
    def load(self, path: str) -> None:
        ...

    @abstractmethod
    def save(self, path: str) -> None:
        ...
```

### Phase 2: Dependency Management

#### 2.1 Core Package Dependencies

Update `python/pyproject.toml`:

```toml
[project]
name = "rynn-motion"
version = "1.0.0"
dependencies = [
    # Minimal core dependencies
    "numpy>=1.20.0,<2.0.0",
    "scipy>=1.7.0",
    "pyyaml>=5.4",
]

[project.optional-dependencies]
simulation = [
    "mujoco>=3.0.0",
    "opencv-python>=4.5.0",
]
datasets = [
    "datasets>=2.0.0",
    "huggingface-hub>=0.20.0",
]
visualization = [
    "matplotlib>=3.5.0",
]
```

#### 2.2 Satellite Package Dependencies

Template for `robots/*/pyproject.toml`:

```toml
[project]
name = "rynn-robot-{name}"
version = "0.1.0"
dependencies = [
    # Core package with version constraint
    "rynn-motion>=1.0.0,<2.0.0",
    # Robot-specific SDKs
    "{robot_sdk}>=x.y.z",
]

[tool.rynn]
compatible_core = ">=1.0.0,<2.0.0"
hardware_requirements = ["usb", "serial", "camera"]
```

### Phase 3: Build and Test Infrastructure

#### 3.1 Root Makefile

Create `Makefile` at project root:

```makefile
.PHONY: install-core install-all test-core test-all clean

ROBOTS := $(wildcard robots/*)
PYTHON := python3.13

install-core:
	@echo "Installing core RynnMotion package..."
	cd python && pip install -e .[all]

install-all: install-core
	@echo "Installing all robot submodules..."
	@for robot in $(ROBOTS); do \
		if [ -f "$$robot/setup.sh" ]; then \
			echo "Setting up $$robot..."; \
			cd "$$robot" && ./setup.sh; \
		fi; \
	done

test-core:
	@echo "Testing core package..."
	cd python && pytest tests/ -v

test-all: test-core
	@echo "Testing all robot implementations..."
	@for robot in $(ROBOTS); do \
		if [ -d "$$robot/tests" ]; then \
			echo "Testing $$robot..."; \
			cd "$$robot" && pytest tests/ -v; \
		fi; \
	done

test-integration:
	@echo "Running integration tests..."
	pytest integration_tests/ -v

clean:
	find . -type d -name "__pycache__" -exec rm -rf {} +
	find . -type d -name "*.egg-info" -exec rm -rf {} +
	find . -type d -name ".pytest_cache" -exec rm -rf {} +
```

#### 3.2 CI/CD Configuration

Create `.github/workflows/test.yml`:

```yaml
name: Test Core and Satellites

on: [push, pull_request]

jobs:
  test-core:
    runs-on: ubuntu-latest
    strategy:
      matrix:
        python-version: ["3.10", "3.11", "3.13"]

    steps:
    - uses: actions/checkout@v3
    - name: Set up Python
      uses: actions/setup-python@v4
      with:
        python-version: ${{ matrix.python-version }}

    - name: Install core dependencies
      run: |
        cd python
        pip install -e .[all,dev]

    - name: Run core tests
      run: |
        cd python
        pytest tests/ --cov=rynn_motion

  test-satellites:
    runs-on: ubuntu-latest
    needs: test-core
    strategy:
      matrix:
        robot: [RynnLeRobot, franka, piper]

    steps:
    - uses: actions/checkout@v3
      with:
        submodules: recursive

    - name: Test ${{ matrix.robot }}
      run: |
        cd robots/${{ matrix.robot }}
        if [ -f "tests/run_tests.sh" ]; then
          ./tests/run_tests.sh
        fi
```

### Phase 4: Version Management

#### 4.1 Compatibility Matrix

Create `config/compatibility.yaml`:

```yaml
# Version compatibility matrix
compatibility_matrix:
  core_version: "1.0.0"

  satellites:
    RynnLeRobot:
      min_version: "0.5.0"
      max_version: "1.0.0"
      tested_versions: ["0.5.0", "0.6.0", "0.7.0"]

    franka:
      min_version: "1.2.0"
      max_version: "2.0.0"
      tested_versions: ["1.2.0", "1.3.0"]

    piper:
      min_version: "0.3.0"
      max_version: "1.0.0"
      tested_versions: ["0.3.0", "0.4.0"]

# Breaking changes log
breaking_changes:
  "1.0.0":
    - "Changed RobotInterface.get_state() return type"
    - "Removed deprecated teleop.legacy module"

  "0.9.0":
    - "Restructured dataset formats"
```

#### 4.2 Version Check Script

Create `scripts/check_compatibility.py`:

```python
#!/usr/bin/env python3
"""Check version compatibility between core and satellites"""

import yaml
import sys
from pathlib import Path
from packaging import version

def check_compatibility():
    # Load compatibility matrix
    with open('config/compatibility.yaml', 'r') as f:
        compat = yaml.safe_load(f)

    core_version = compat['compatibility_matrix']['core_version']

    # Check each satellite
    robots_dir = Path('robots')
    for robot_dir in robots_dir.iterdir():
        if robot_dir.is_dir():
            # Check if version is compatible
            robot_name = robot_dir.name
            if robot_name in compat['compatibility_matrix']['satellites']:
                sat_info = compat['compatibility_matrix']['satellites'][robot_name]
                print(f"✓ {robot_name}: Compatible versions {sat_info['min_version']} - {sat_info['max_version']}")
            else:
                print(f"⚠ {robot_name}: No compatibility info found")

    return 0

if __name__ == "__main__":
    sys.exit(check_compatibility())
```

### Phase 5: Documentation

#### 5.1 Architecture Documentation

Create `docs/architecture/README.md`:

```markdown
# RynnMotion Architecture

## Package Structure
- **Core Package**: Shared algorithms and simulation (`python/`)
- **Satellite Packages**: Robot-specific implementations (`robots/*/`)
- **Dependency Flow**: Satellites depend on Core (unidirectional)

## Development Guidelines

### Adding a New Robot
1. Create submodule in `robots/`
2. Implement `RobotInterface` from core
3. Create setup script with core installation
4. Add to compatibility matrix
5. Write integration tests

### Modifying Core Package
1. Check impact on all satellites
2. Update compatibility matrix
3. Use deprecation warnings for breaking changes
4. Run full integration test suite

## API Stability
- Core APIs follow semantic versioning
- Breaking changes require major version bump
- Deprecation period of 2 minor versions
```

#### 5.2 Migration Guide Template

Create `docs/migration/template.md`:

```markdown
# Migration Guide: vX.Y.Z to vA.B.C

## Breaking Changes
- List all breaking changes
- Provide before/after examples

## Deprecations
- List deprecated features
- Show migration path

## New Features
- List new features
- Provide usage examples

## Update Instructions
1. Update core package
2. Check compatibility matrix
3. Update satellite packages
4. Run integration tests
```

## Best Practices

### Do's ✅
1. **Keep core dependencies minimal** - Only essential packages
2. **Use editable installs for development** - `-e` flag for real-time updates
3. **Version everything** - Core, satellites, and compatibility matrix
4. **Test integration regularly** - Automated CI/CD for all combinations
5. **Document interfaces clearly** - Type hints and docstrings
6. **Use semantic versioning** - Major.Minor.Patch
7. **Isolate hardware SDKs** - Keep in satellite venvs only

### Don'ts ❌
1. **Don't create circular dependencies** - Core should never import satellites
2. **Don't hardcode paths** - Use relative imports and config files
3. **Don't skip compatibility testing** - Test before merging
4. **Don't break interfaces without deprecation** - Use warnings first
5. **Don't mix hardware SDKs** - Keep them isolated
6. **Don't use global Python packages** - Always use venvs
7. **Don't ignore version constraints** - Pin compatible versions

## Monitoring and Maintenance

### Health Checks
- Weekly compatibility tests
- Monthly dependency updates
- Quarterly architecture review

### Metrics to Track
- Import time of core package
- Test coverage percentage
- Number of cross-dependencies
- Time to set up new robot

### Warning Signs
- Increasing setup complexity
- Circular import attempts
- Version conflict reports
- Slow import times

## Future Considerations

### Potential Improvements
1. **Plugin System**: Dynamic robot loading
2. **Docker Containers**: Isolated environments
3. **Microservices**: Separate services for each robot
4. **gRPC/REST APIs**: Network-based interfaces
5. **Conda Environments**: Better binary dependency management

### Scalability Path
- Current: 5-10 robot types
- Medium-term: 20-30 robot types
- Long-term: 50+ robot types → Consider microservices

## Conclusion

The Core-Satellite pattern provides good separation of concerns for RynnMotion's multi-robot architecture. Following these guidelines will ensure maintainable and scalable growth of the codebase.

Last Updated: 2024-10-18