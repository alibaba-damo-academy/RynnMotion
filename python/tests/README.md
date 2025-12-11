# RynnMotion Test Suite

This directory contains the test suite for RynnMotion, organized to mirror the source code structure.

## Directory Structure

```
tests/
├── test_algorithms/               # Tests for RynnMotion.algorithms
│   ├── test_pin_kine.py           # PinKine kinematics tests (integrated with PRobotManager)
│   └── ...                        # Future: pose_mapper, osc, etc.
├── test_base/                     # Tests for RynnMotion.base
│   └── ...                        # Future: MuJoCo interface tests
├── test_controller/               # Tests for RynnMotion.controller
│   └── ...                        # Future: controller tests
├── test_manager/                  # Tests for RynnMotion.manager
│   ├── test_manager.py            # PRobotManager and PSceneManager tests
│   └── test_mjcf_parser.py        # MjcfParser CLI test script
├── test_common/                   # Tests for RynnMotion.common
│   └── ...                        # Future: common utilities tests
├── test_teleop/                   # Tests for RynnMotion.teleop
│   └── ...                        # Future: teleoperation tests
└── standalone/                    # Standalone tests (no RynnMotion dependencies)
    ├── test_pin_kine_standalone.py  # FK tests using local XML fixtures
    └── fixtures/                  # Robot MJCF files for standalone tests
        ├── fr3_pinocchio.xml
        ├── ur5e_pinocchio.xml
        ├── piper_pinocchio.xml
        ├── rm75_pinocchio.xml
        └── so101_pinocchio.xml
```

## Test Categories

### Integrated Tests
Tests that use RynnMotion infrastructure (PRobotManager, PSceneManager):
- `test_algorithms/test_pin_kine.py` - Tests PinKine with all robots via PRobotManager
- `test_manager/test_manager.py` - Tests robot and scene managers

These tests validate the full integration with the project structure.

### CLI Test Scripts
Command-line test scripts for interactive validation:
- `test_manager/test_mjcf_parser.py` - MjcfParser robot configuration extraction

### Standalone Tests
Tests that work independently with local fixtures:
- `standalone/test_pin_kine_standalone.py` - FK validation using local XML files

These tests can be copied to other projects or used for isolated validation.

## Running Tests

### Run all tests:
```bash
pytest tests/
```

### Run specific test module:
```bash
pytest tests/test_algorithms/test_pin_kine.py -v
pytest tests/test_manager/test_manager.py -v
pytest tests/standalone/test_pin_kine_standalone.py -v
```

### Run tests for specific robot:
```bash
pytest tests/test_algorithms/test_pin_kine.py -k "fr3" -v
```

### Run with coverage:
```bash
pytest tests/ --cov=RynnMotion --cov-report=html
```

## Test Summary

| Module | Tests | Coverage |
|--------|-------|----------|
| `test_algorithms/` | 30 tests | PinKine FK, multi-site, DOF validation |
| `test_manager/` | 47 tests | PRobotManager, PSceneManager, integration |
| `standalone/` | 10 tests | Independent FK validation |
| **Total** | **87 tests** | |

## Adding New Tests

1. **Mirror source structure**: Create tests in `test_<module>/` matching `src/RynnMotion/<module>/`
2. **Use fixtures**: Leverage `conftest.py` for shared fixtures
3. **Follow naming**: Test files should be `test_*.py`, test functions `test_*()`
4. **Use classes**: Group related tests in classes (e.g., `TestSingleSiteFK`)
5. **Add docstrings**: Document what each test validates

Example:
```python
# tests/test_algorithms/test_my_algorithm.py
import pytest
from RynnMotion.algorithms.my_algorithm import MyAlgorithm

class TestMyAlgorithm:
    """Test MyAlgorithm functionality"""

    def test_initialization(self):
        """Test that MyAlgorithm initializes correctly"""
        algo = MyAlgorithm()
        assert algo is not None
```

## Notes

- All tests should pass with only the standard hppfcl deprecation warning
- Standalone tests are portable and can be run outside the RynnMotion environment
- Integrated tests validate the full project infrastructure
