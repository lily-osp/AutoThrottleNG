# Contributing to AutoThrottleNG

We welcome contributions to AutoThrottleNG! This document provides guidelines and information for contributors.

## Table of Contents

- [Code of Conduct](#code-of-conduct)
- [Getting Started](#getting-started)
- [Types of Contributions](#types-of-contributions)
- [Development Process](#development-process)
- [Code Standards](#code-standards)
- [Testing Requirements](#testing-requirements)
- [Documentation](#documentation)
- [Pull Request Process](#pull-request-process)
- [Issue Reporting](#issue-reporting)
- [Community](#community)

## Code of Conduct

This project follows a code of conduct to ensure a welcoming environment for all contributors. By participating, you agree to:

- Be respectful and inclusive
- Focus on constructive feedback
- Accept responsibility for mistakes
- Show empathy towards other contributors
- Help create a positive community

## Getting Started

### Prerequisites

Before contributing, ensure you have:

- **Arduino IDE** 1.8.0+ or **PlatformIO**
- **Arduino PID Library** (automatically installed via `make install-deps`)
- **Git** for version control
- **Arduino board** for testing (Uno, Mega, ESP32, etc.)

### Setting Up Development Environment

1. **Clone the repository:**
   ```bash
   git clone https://github.com/yourusername/AutoThrottleNG.git
   cd AutoThrottleNG
   ```

2. **Install dependencies:**
   ```bash
   make install-deps
   ```

3. **Verify setup:**
   ```bash
   make test-all
   ```

4. **Run examples:**
   ```bash
   make compile-examples
   ```

## Types of Contributions

### Code Contributions

- **Bug fixes** - Critical for library stability
- **New features** - Enhancements and new capabilities
- **Performance improvements** - Optimization and efficiency
- **Compatibility fixes** - Support for new boards/platforms

### Documentation

- **README updates** - Keep installation and usage current
- **API documentation** - Document new methods and features
- **Examples** - Create practical usage examples
- **Troubleshooting guides** - Help users resolve issues

### Testing

- **Unit tests** - Test individual components
- **Integration tests** - Test combined functionality
- **Compatibility testing** - Verify across different platforms
- **Example validation** - Ensure examples compile and work

### Other Contributions

- **Issue triage** - Help categorize and prioritize issues
- **Community support** - Answer questions in issues/discussions
- **Code review** - Review pull requests from other contributors

## Development Process

### 1. Choose an Issue

- Check [GitHub Issues](https://github.com/yourusername/AutoThrottleNG/issues) for open tasks
- Look for issues labeled `good first issue` or `help wanted`
- Comment on the issue to indicate you're working on it

### 2. Create a Branch

```bash
# Create and switch to a feature branch
git checkout -b feature/your-feature-name

# Or for bug fixes
git checkout -b fix/issue-number-description
```

### 3. Make Changes

- Follow the [code standards](#code-standards) below
- Test your changes thoroughly
- Update documentation as needed
- Add examples for new features

### 4. Test Your Changes

```bash
# Run all tests
make test-all

# Test specific examples
make compile-basic
make compile-motor
# ... etc

# Upload to Arduino for real testing
make upload-basic PORT=/dev/ttyACM0
```

### 5. Commit Your Changes

```bash
# Stage your changes
git add .

# Commit with descriptive message
git commit -m "Add feature: brief description of changes

- Detailed explanation of what was changed
- Why the change was needed
- Any breaking changes or special considerations"
```

### 6. Create Pull Request

- Push your branch to GitHub
- Create a pull request with a clear description
- Reference any related issues
- Request review from maintainers

## Code Standards

### C++ Code Style

- **Naming Conventions:**
  - Classes: `PascalCase` (e.g., `AutoThrottleNG`)
  - Methods: `camelCase` (e.g., `setTarget()`)
  - Variables: `camelCase` (e.g., `pidInput`)
  - Constants: `UPPER_CASE` (e.g., `MAX_OUTPUT`)
  - Private members: `_underscorePrefix` (e.g., `_pidInput`)

- **Formatting:**
  - Use 4 spaces for indentation (or match existing code)
  - Line length: 100 characters maximum
  - Consistent brace placement
  - Space around operators

- **Documentation:**
  - Use Doxygen-style comments for public methods
  - Document parameters and return values
  - Explain complex algorithms
  - Include usage examples where helpful

### Example Code Style

```cpp
/**
 * @brief Sets the target setpoint for the controller
 * @param target The desired setpoint value
 */
void setTarget(double target) {
    if (isnan(target) || isinf(target)) {
        return;  // Reject invalid inputs
    }

    _pidSetpoint = target;
}
```

### Arduino-Specific Guidelines

- **Memory Management:** Be mindful of limited RAM on AVR boards
- **PROGMEM:** Use `F()` macro for string literals in Serial prints
- **Volatile Variables:** Use `volatile` for interrupt-shared variables
- **Timing:** Use `millis()` instead of `delay()` for non-blocking code

## Testing Requirements

### Compilation Testing

All contributions must pass compilation tests:

```bash
# Test all examples compile
make compile-examples

# Test specific functionality
make compile-basic     # Basic PID
make compile-advanced  # Failsafe features
make compile-operational  # Operational modes
```

### Functional Testing

- **Manual Testing:** Upload examples to Arduino board and verify operation
- **Edge Cases:** Test with extreme values, error conditions
- **Mode Testing:** Verify all operational modes work correctly
- **Recovery Testing:** Test auto-recovery functionality

### Compatibility Testing

Test on multiple platforms when possible:

- **AVR:** Arduino Uno, Mega, Leonardo
- **ARM:** Arduino Due, Zero
- **ESP32:** ESP32-based boards
- **SAMD:** MKR series boards

### Performance Testing

- **Memory Usage:** Monitor RAM and flash usage
- **Execution Time:** Verify real-time performance
- **Resource Limits:** Ensure operation within Arduino constraints

## Documentation

### Documentation Requirements

- **README Updates:** Update main README for new features
- **API Documentation:** Document all public methods
- **Examples:** Create examples for new features
- **Inline Comments:** Explain complex logic

### Documentation Standards

- **Clear Language:** Use simple, technical English
- **Complete Information:** Include all necessary details
- **Practical Examples:** Provide working code examples
- **Cross-References:** Link related documentation sections

## Pull Request Process

### Before Submitting

1. **Self-Review:** Check your code meets all standards
2. **Testing:** Ensure all tests pass
3. **Documentation:** Update docs for any changes
4. **Compatibility:** Test on target platforms

### Pull Request Template

Use this template for pull requests:

```markdown
## Description
Brief description of the changes made

## Type of Change
- [ ] Bug fix
- [ ] New feature
- [ ] Documentation update
- [ ] Performance improvement
- [ ] Code refactoring

## Testing
- [ ] All examples compile
- [ ] Manual testing performed
- [ ] Edge cases tested
- [ ] Documentation updated

## Breaking Changes
- [ ] No breaking changes
- [ ] Breaking changes detailed below:
  - Change 1
  - Change 2

## Additional Notes
Any additional information or context
```

### Review Process

1. **Automated Checks:** CI/CD runs compilation tests
2. **Code Review:** Maintainers review code quality and standards
3. **Testing:** Contributors may be asked to test specific scenarios
4. **Approval:** PR approved and merged, or feedback provided for revisions

## Issue Reporting

### Bug Reports

When reporting bugs, include:

- **Clear Title:** Descriptive but concise
- **Environment:** Arduino board, IDE version, library version
- **Steps to Reproduce:** Detailed reproduction steps
- **Expected Behavior:** What should happen
- **Actual Behavior:** What actually happens
- **Code Sample:** Minimal code to reproduce the issue
- **Serial Output:** Any relevant debug output

### Feature Requests

For new features, include:

- **Use Case:** Why is this feature needed?
- **Implementation Idea:** How should it work?
- **Alternatives:** Other solutions considered
- **Impact:** How it affects existing functionality

### Enhancement Suggestions

For improvements, include:

- **Current Limitation:** What's the current problem?
- **Proposed Solution:** How to improve it
- **Benefits:** What advantages does it provide?
- **Compatibility:** Impact on existing code

## Community

### Communication Channels

- **GitHub Issues:** Bug reports and feature requests
- **GitHub Discussions:** General questions and community support
- **Pull Request Comments:** Code review discussions

### Getting Help

- **Documentation First:** Check docs/ folder for detailed guides
- **Examples:** Run provided examples to understand usage
- **Search Issues:** Look for similar reported issues
- **Create Issue:** If no existing solution found

### Recognition

Contributors are recognized through:

- **GitHub Contributors List:** Automatic recognition
- **Changelog Entries:** Major contributions documented
- **Issue/PR References:** Links in release notes

## Recognition and Credits

We appreciate all contributions, from bug fixes to documentation improvements. Contributors may be acknowledged in:

- **README Contributors Section**
- **Changelog Entries**
- **GitHub Release Notes**
- **Project Documentation**

---

Thank you for contributing to AutoThrottleNG! Your efforts help make this library better for the entire Arduino community.
