# Contributing to RynnMotion

Thank you for your interest in contributing to RynnMotion! We welcome contributions from the community.

---

## Code of Conduct

By participating in this project, you agree to maintain a respectful and professional environment. We are committed to making participation in this project a harassment-free experience for everyone.

---

## How to Contribute

### Reporting Bugs

**Before creating an issue:**
1. Check if the issue already exists in [GitHub Issues](https://github.com/alibaba-damo-academy/RynnMotion/issues)
2. Search [Discussions](https://github.com/alibaba-damo-academy/RynnMotion/discussions) for similar questions

**When creating a bug report, include:**
- **Environment:** OS, compiler version, RynnMotion version
- **Minimal reproducible example:** Code snippet or steps to reproduce
- **Expected behavior:** What you expected to happen
- **Actual behavior:** What actually happened
- **Logs/errors:** Full error messages, stack traces

**Template:**
```markdown
**Environment:**
- OS: Ubuntu 22.04
- Compiler: GCC 11.4
- RynnMotion version: v0.9.0

**To Reproduce:**
1. Run `./mujocoExe fr3 ui`
2. Observe segfault in Estimator module

**Expected:** Simulation runs normally
**Actual:** Segfault at line 42 in estimator.cpp

**Error log:**
```
Segmentation fault (core dumped)
```
```

### Suggesting Features

**Feature requests are welcome!**

Please include:
- **Use case:** What problem does this solve?
- **Proposed solution:** How would you implement it?
- **Alternatives considered:** What other approaches did you think about?

### Pull Requests

**We love PRs!** Here's how to submit:

#### 1. Fork and Branch

```bash
# Fork on GitHub, then clone
git clone https://github.com/your-username/RynnMotion.git
cd RynnMotion

# Create feature branch
git checkout -b feature/my-awesome-feature
```

#### 2. Code

**CRITICAL: Follow coding standards**

- Read `docs/c++_coding_style.md` (v2.2, Nov 19, 2025)
- Namespace: `rynn::`
- Naming: `member_` (trailing underscore for class members)
- Comments: Minimal in `.cpp`, API docs in `.hpp`
- CMake: NO COMMENTS

**Example:**

```cpp
// Good ✅
namespace rynn {

class MyModule : public CModuleBase {
public:
  void update() override;

private:
  Eigen::VectorXd _qCmd;  // Trailing underscore
  void _computeControl();  // Leading underscore for private
};

} // namespace rynn
```

```cpp
// Bad ❌
namespace my_namespace {  // Wrong namespace

class MyModule : public CModuleBase {
public:
  void update() override;

private:
  Eigen::VectorXd qCmd;  // Missing trailing underscore
  void computeControl();  // Missing leading underscore for private
};

}
```

#### 3. Test

**Before submitting PR:**

```bash
# Build
cmake --build build -j$(nproc)

# Run C++ tests
cd build
ctest

# Run Python tests
cd ../python
pytest tests/

# Test your specific example
./mujocoExe fr3 ui
```

**Add tests for new features:**
- C++ tests: `unittest/`
- Python tests: `python/tests/`

#### 4. Commit

**Commit message format:**

```
<type>: <subject>

<body>

<footer>
```

**Types:**
- `feat`: New feature
- `fix`: Bug fix
- `docs`: Documentation only
- `refactor`: Code refactoring
- `test`: Adding tests
- `chore`: Maintenance

**Example:**

```
feat: add QP-based trajectory optimizer

Implements trajectory optimization using OSQP for
smooth joint-space trajectories with constraints.

Closes #123
```

**Rules:**
- Subject: 50 chars or less, imperative ("add" not "adds")
- Body: Wrap at 72 chars, explain "why" not "what"
- Footer: Reference issues (`Closes #123`, `Fixes #456`)

#### 5. Push and Create PR

```bash
git push origin feature/my-awesome-feature
```

**On GitHub:**
1. Create Pull Request
2. Fill in template:
   - **What:** What does this PR do?
   - **Why:** Why is this change needed?
   - **How:** How was this implemented?
   - **Testing:** How was this tested?

**PR Template:**

```markdown
## What
Adds QP-based trajectory optimization module

## Why
Current trajectory generation doesn't handle constraints.
Users requested smoother motion with acc/jerk limits.

## How
- Implemented `TrajectoryOptimizer` class using OSQP
- Added constraints: pos/vel/acc/jerk limits
- Integrated with existing Planner module

## Testing
- [x] Unit tests (100% coverage)
- [x] Integration test with FR3 robot
- [x] Tested on dual-arm scenario
- [x] Documentation updated

## Checklist
- [x] Code follows style guide
- [x] Tests added
- [x] Documentation updated
- [x] CHANGELOG.md updated
```

#### 6. Review Process

**What to expect:**
- Maintainers review within 48 hours
- CI/CD runs automated tests
- May request changes
- Once approved, we'll merge!

**Iterate based on feedback:**
```bash
# Make requested changes
git add .
git commit -m "fix: address review comments"
git push origin feature/my-awesome-feature
```

---

## Development Setup

### C++ Development

```bash
# Install dependencies (Ubuntu)
sudo apt-get install \
  build-essential \
  cmake \
  libeigen3-dev \
  libyaml-cpp-dev \
  git

# Clone
git clone https://github.com/alibaba-damo-academy/RynnMotion.git
cd RynnMotion

# Build
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Debug ..
make -j$(nproc)

# Run tests
ctest --output-on-failure
```

### Python Development

```bash
# Create virtual environment
python3 -m venv venv
source venv/bin/activate

# Install in editable mode
pip install -e python/

# Run tests
pytest python/tests/
```

### Pre-commit Hooks (Recommended)

```bash
# Install pre-commit
pip install pre-commit

# Install hooks
pre-commit install

# Now hooks run automatically on `git commit`
```

**Hooks:**
- Format C++ with clang-format
- Format Python with black
- Check for large files
- Check YAML syntax

---

## Code Review Guidelines

### For Contributors

**When submitting code:**
- ✅ Self-review first (read your own diff)
- ✅ Keep PRs small (<500 lines if possible)
- ✅ One feature per PR
- ✅ Add tests for new code
- ✅ Update documentation

### For Reviewers

**When reviewing code:**
- ✅ Be respectful and constructive
- ✅ Explain "why" when requesting changes
- ✅ Approve quickly if minor issues
- ✅ Test the code locally if complex

**Review checklist:**
- [ ] Code follows style guide
- [ ] Tests added and passing
- [ ] Documentation updated
- [ ] No breaking changes (or properly documented)
- [ ] Performance considered
- [ ] Security considered

---

## Project Structure for Contributors

```
RynnMotion/
├── motion/              # C++ core (main work here)
│   ├── manager/         # Robot, scene, FSM managers
│   ├── runtime/         # Module orchestration
│   ├── module/          # Control modules (add new modules here)
│   ├── utils/           # Utilities (kinematics, IK, etc.)
│   └── interface/       # Sim/hardware interfaces
├── mujoco/              # MuJoCo interface
├── python/              # Python bindings (pybind11)
│   ├── src/RynnMotion/  # Python package source
│   └── tests/           # Python tests
├── models/              # MJCF robot models
├── robots/              # Hardware drivers
├── examples/            # Examples (add tutorials here)
├── unittest/            # C++ tests
├── docs/                # Documentation (update as needed)
└── cmake/               # Build system
```

---

## What Makes a Good Contribution?

### Great First Contributions:
- 🌟 Fix typos in documentation
- 🌟 Improve error messages
- 🌟 Add examples
- 🌟 Improve test coverage
- 🌟 Update outdated docs

### Good Issues for Beginners:
Look for `good-first-issue` label on GitHub.

**Current needs:**
- More tutorial examples
- Documentation improvements
- Python API coverage
- Test coverage
- Hardware driver support

### Advanced Contributions:
- New control modules
- Optimization algorithms
- Hardware interfaces
- ROS2 integration
- Performance improvements

---

## Community

### Communication Channels

- **GitHub Discussions:** General questions, ideas
- **GitHub Issues:** Bug reports, feature requests
- **Discord:** Real-time chat (https://discord.gg/rynnmotion)
- **Mailing List:** Announcements (rynnmotion@googlegroups.com)

### Getting Help

**Before asking:**
1. Check documentation (`docs/`)
2. Search existing issues/discussions
3. Try examples (`examples/`)

**When asking:**
- Be specific
- Provide code snippets
- Include error messages
- Describe what you've tried

### Community Calls

**Monthly community call:**
- **When:** First Wednesday of each month, 10 AM PT
- **Where:** Zoom (link in Discord)
- **Agenda:** Feature demos, Q&A, roadmap discussion

---

## Recognition

**Contributors are valued!**

- Listed in [CONTRIBUTORS.md](CONTRIBUTORS.md)
- Mentioned in release notes
- Co-authorship on methodology papers (if significant contribution)
- Swag for major contributors (stickers, t-shirts)

---

## License

By contributing, you agree that your contributions will be licensed under the MIT License.

---

## Questions?

**Still have questions?**
- Open a [Discussion](https://github.com/alibaba-damo-academy/RynnMotion/discussions)
- Ask on [Discord](https://discord.gg/rynnmotion)
- Email: maintainers@rynnmotion.dev

**Thank you for contributing to RynnMotion!** 🚀
