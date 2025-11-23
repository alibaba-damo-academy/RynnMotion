# C++ Coding Style Guide

**RynnMotion Open Source Project**

---

## Core Principles

1. **Code is self-documenting** - Clear names, clean structure
2. **Headers document API** - Brief, professional, complete
3. **Implementation is clean** - Minimal comments, clear logic
4. **No redundancy** - Every comment adds value or is removed

---

## Header Files (.hpp)

### Class Documentation

```cpp
/**
 * @class MjcfParser
 * @brief Parses MuJoCo MJCF files to extract robot configuration
 *
 * Responsibilities:
 * - Load both robotMJCF (full model) and pinoMJCF (kinematic tree)
 * - Extract robot parameters (actuator types, joint limits, gains, keyframes)
 * - Populate RobotManager with extracted data
 */
class MjcfParser {
```

### Method Documentation

**Public methods:**
```cpp
/**
 * @brief Construct MjcfParser and load MJCF models
 * @param rm Reference to RobotManager to populate
 *
 * Automatically:
 * - Loads robotMJCF (*_robot.xml) into robotModel_
 * - Loads pinoMJCF (*_pinocchio.xml) into pinoModel_
 * - Calls parseAndPopulate() to extract all configuration
 */
explicit MjcfParser(RobotManager &rm);
```

**Private methods** (minimal or none):
```cpp
/**
 * @brief Detect end-effectors (grippers) vs motion joints
 */
void detectEndEffectors();

/**
 * @brief Extract joint position/velocity/torque limits
 */
void extractJointLimits();
```

### What NOT to Document in Headers

❌ **Don't explain obvious things:**
```cpp
// ❌ BAD
/**
 * @brief Get robot number
 * @return The robot number
 */
int getRobotNumber() const;

// ✅ GOOD
int getRobotNumber() const;
```

❌ **Don't use section dividers:**
```cpp
// ❌ BAD
// ======================================================================
// Public Getters
// ======================================================================

// ✅ GOOD (no dividers, clear organization)
```

❌ **Don't over-document:**
```cpp
// ❌ BAD - Too verbose
/**
 * @brief Extract actuator PD gains (kp, kd) from MJCF
 *
 * Source: pinoModel_ (*_pinocchio.xml)
 * Reason: Kinematic tree has actuator gain parameters
 *
 * Extraction logic depends on actuator mode:
 * - Position: kp = gainprm[0], kd = -biasprm[2] (or kp*0.05 if missing)
 * - Velocity: kp = gainprm[0], kd = 0
 * - Torque: kp = 1.0, kd = 0
 *
 * Populates: rm_._simKp, rm_._simKd
 */

// ✅ GOOD - Concise
/**
 * @brief Extract actuator PD gains (kp, kd) from MJCF
 */
```

---

## Implementation Files (.cpp)

### NO Section Dividers

**Keep implementation files clean - no section markers:**

```cpp
// ❌ BAD
// ======================================================================
// Constructor & Destructor
// ======================================================================

MjcfParser::MjcfParser(RobotManager &rm) : rm_(rm), robotModel_(nullptr), pinoModel_(nullptr) {
  parseAndPopulate();
}

// ✅ GOOD
MjcfParser::MjcfParser(RobotManager &rm) : rm_(rm), robotModel_(nullptr), pinoModel_(nullptr) {
  parseAndPopulate();
}
```

### Inline Comments (Very Rare)

**Only for critical clarifications (data formats, special mappings):**
```cpp
// ✅ Acceptable inline comments
obj.quat = Eigen::Quaterniond(
    data[0],  // w
    data[1],  // x
    data[2],  // y
    data[3]   // z
);

int actuatorIdx = jointIndices_[i];  // ← CORRECT: Use jointIndices mapping
```

**NO standalone explanatory comments:**
```cpp
// ❌ BAD
// Load robotMJCF (*_robot.xml)
robotModel_ = mj_loadXML(rm_.getRobotMJCF().c_str(), 0, nullptr, 0);
if (!robotModel_) return;

// Load pinoMJCF (*_pinocchio.xml) or fallback to robotModel_
pinoModel_ = mj_loadXML(rm_.getPinoMJCF().c_str(), 0, nullptr, 0);
if (!pinoModel_) pinoModel_ = robotModel_;

// ✅ GOOD (no comments)
robotModel_ = mj_loadXML(rm_.getRobotMJCF().c_str(), 0, nullptr, 0);
if (!robotModel_) return;

pinoModel_ = mj_loadXML(rm_.getPinoMJCF().c_str(), 0, nullptr, 0);
if (!pinoModel_) pinoModel_ = robotModel_;
```

### What NOT to Write in Implementation

❌ **No method documentation** (already in .hpp):
```cpp
// ❌ BAD
/**
 * @brief Detect end-effectors (grippers) vs motion joints
 */
void MjcfParser::detectEndEffectors() {

// ✅ GOOD
void MjcfParser::detectEndEffectors() {
```

❌ **No obvious comments:**
```cpp
// ❌ BAD
// Loop through all actuators
for (int i = 0; i < robotModel_->nu; i++) {

// ✅ GOOD (no comment)
for (int i = 0; i < robotModel_->nu; i++) {
```

❌ **No std::cerr spam:**
```cpp
// ❌ BAD
if (!robotModel_) {
  std::cerr << "Error: Robot MJCF path is empty" << std::endl;
  return;
}

// ✅ GOOD (fail silently or use DEBUG_LOG)
if (!robotModel_) return;
```

❌ **No section dividers - EVER:**
```cpp
// ❌ BAD
// ======================================================================
// Constructor & Destructor
// ======================================================================

// ❌ BAD
// === Phase 1: Load robotMJCF ===

// ❌ BAD
// Constructor implementation

// ✅ GOOD (no dividers, no section comments)
MjcfParser::MjcfParser(RobotManager &rm) : rm_(rm) {
  robotModel_ = mj_loadXML(rm_.getRobotMJCF().c_str(), 0, nullptr, 0);
  if (!robotModel_) return;

  pinoModel_ = mj_loadXML(rm_.getPinoMJCF().c_str(), 0, nullptr, 0);
  if (!pinoModel_) pinoModel_ = robotModel_;
}
```

---

## Naming Conventions

### Variables

```cpp
// Class members
RobotManager &rm_;           // Reference
mjModel *robotModel_;        // Pointer
std::vector<int> eeIndices_; // Container

// Local variables
int mdof = jointIndices.size();
std::string robotMjcfPath = rm_.getRobotMJCF();
```

### Methods

```cpp
// Descriptive action verbs
void detectEndEffectors();
void extractJointLimits();
void parseAndPopulate();

// Getters/setters (no get/set prefix for members)
int getMotionDOF() const;
void setMotionDOF(int mdof);
```

---

## Organization Patterns

### Header File Structure

```cpp
#pragma once

#include "dependencies.hpp"

namespace rynn {

/**
 * @class ClassName
 * @brief Brief description
 *
 * Responsibilities listed
 */
class ClassName {
public:
  // Constructor & Destructor
  explicit ClassName(Args args);
  ~ClassName();

  // Disable copy/move if needed
  ClassName(const ClassName &) = delete;
  ClassName &operator=(const ClassName &) = delete;

  // Static utility methods
  static ReturnType utilityMethod();

private:
  // Private methods
  void privateMethod();

  // Member variables
  Type member_;
};

} // namespace rynn
```

### Implementation File Structure

```cpp
#include "header.hpp"

#include <system_headers>
#include "project_headers.hpp"

namespace rynn {

ClassName::ClassName(Args args) : member_(value) {
  initialize();
}

ClassName::~ClassName() {
  cleanup();
}

void ClassName::publicMethod() {
  // Implementation
}

void ClassName::privateMethod() {
  // Implementation
}

} // namespace rynn
```

---

## Debug Logging

### Use DEBUG_LOG for Development

```cpp
// Development/debugging
DEBUG_LOG("Loaded robot MJCF: " << robotMjcfPath);
DEBUG_LOG("Detected end-effector: actuator[" << i << "] = \"" << name << "\"");

// Production (minimal)
if (!robotModel_) return;  // Fail silently
```

### Remove Before Release

```cpp
// ❌ Remove debug output
std::cout << "Debug: " << value << std::endl;

// ❌ Remove verbose warnings
std::cerr << "Warning: This might happen sometimes" << std::endl;
```

---

## CMake Files

**NO COMMENTS** - CMake code should be self-documenting with clear variable names and structure.

```cmake
cmake_minimum_required(VERSION 3.16)
project(manager)

set(LIBRARY_NAME manager)

find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

if(APPLE)
  set(YAML_LIB yaml-cpp::yaml-cpp)
  include_directories(/opt/homebrew/opt/yaml-cpp/include)
else()
  find_library(YAML_CPP_LIBRARY NAMES yaml-cpp)
  set(YAML_LIB ${YAML_CPP_LIBRARY})
endif()

set(SOURCE_FILES
  robot_manager.cpp
  mjcf_parser.cpp
)

add_library(${LIBRARY_NAME} SHARED ${SOURCE_FILES})

target_include_directories(${LIBRARY_NAME} PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${MUJOCO_INCLUDE_DIRS}
)

target_link_libraries(${LIBRARY_NAME} PUBLIC
  Eigen3::Eigen
  ${YAML_LIB}
)
```

---

## Pre-Commit Checklist

**Headers (.hpp)**:
- [ ] Class has brief `@brief` documentation
- [ ] Public methods documented (brief + params)
- [ ] Private methods have one-line brief or none
- [ ] No section dividers
- [ ] No redundant comments

**Implementation (.cpp)**:
- [ ] No section dividers
- [ ] No method documentation (already in .hpp)
- [ ] Inline comments only for non-obvious logic
- [ ] No std::cerr spam (use DEBUG_LOG or fail silently)
- [ ] No commented-out code

**CMake Files**:
- [ ] NO COMMENTS - remove all comments
- [ ] Self-documenting with clear variable/target names

**General**:
- [ ] Code is self-documenting (clear names)
- [ ] Comments explain WHY, not WHAT
- [ ] Professional and concise
- [ ] Ready for open source release

---

## Quick Reference

### When to Comment

| Location | What | Example |
|----------|------|---------|
| .hpp class | Brief + responsibilities | `@brief Parses MJCF files` |
| .hpp public method | Brief + params | `@param rm RobotManager to populate` |
| .hpp private method | One-line brief or none | `@brief Extract joint limits` |
| .cpp inline | Non-obvious logic only | `// Fallback to robotModel_` |
| CMakeLists.txt | NEVER | No comments allowed |

### When NOT to Comment

- ❌ Obvious getters/setters
- ❌ Repeated method documentation in .cpp
- ❌ Section dividers in .hpp or .cpp
- ❌ Explaining WHAT code does
- ❌ Debug output (use DEBUG_LOG)
- ❌ Commented-out code
- ❌ Any comments in CMake files

---

**Version**: 2.2
**Last Updated**: 2025-11-19
**Maintainer**: RynnMotion Team
