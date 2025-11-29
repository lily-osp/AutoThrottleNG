# AutoThrottleNG Documentation

## Table of Contents

- [Overview](#overview)
- [Documentation Structure](#documentation-structure)
- [Quick Start Guide](#quick-start-guide)
- [Key Features Overview](#key-features-overview)
- [Example Applications](#example-applications)
- [Support and Resources](#support-and-resources)
- [Version Information](#version-information)

## Overview

This documentation provides information about the AutoThrottleNG Arduino PID control library. The documentation is organized into focused guides covering different aspects of the library, from basic usage to advanced technical details.

## Documentation Structure

### [System Architecture](system_architecture.md)
Detailed explanation of AutoThrottleNG's internal architecture, data flow, memory management, and integration patterns.

**Topics Covered:**
- Core architecture components
- Signal processing pipeline
- Safety and failsafe systems
- Memory and timing architecture
- Integration approaches

### [Troubleshooting Guide](troubleshooting.md)
Comprehensive guide for diagnosing and resolving common issues with AutoThrottleNG implementations.

**Topics Covered:**
- Compilation errors and solutions
- Runtime issues and fixes
- Failsafe troubleshooting
- Sensor and hardware problems
- PID tuning difficulties
- Memory and performance issues
- Debugging tools and techniques

### [Detailed Explanation](explanation.md)
In-depth explanation of how AutoThrottleNG works internally, including algorithms and design decisions.

**Topics Covered:**
- PID control fundamentals
- Signal processing algorithms
- Failsafe system architecture
- Timing and synchronization
- Memory management
- Controller modes and configurations
- Reset and initialization processes

### [Examples Guide](examples.md)
Complete guide to all example sketches, with detailed explanations and practical applications.

**Topics Covered:**
- Example categories and purposes
- Hardware requirements and setup
- Real-world application examples
- Running and modifying examples
- Extending examples for custom applications

### [Usage Guide](usage.md)
Practical guide for implementing AutoThrottleNG in your projects, from basic setup to advanced configurations.

**Topics Covered:**
- Installation and setup
- Basic usage patterns
- Configuration options
- PID tuning guidelines
- Signal processing setup
- Failsafe configuration
- Monitoring and debugging
- Common usage patterns
- Performance optimization

### [Internal Mechanisms](mechanism.md)
Technical deep-dive into AutoThrottleNG's internal algorithms, data structures, and implementation details.

**Topics Covered:**
- PID algorithm implementation
- Signal processing mechanisms
- Failsafe system internals
- Timing and synchronization
- Memory management
- Controller modes
- Saturation and anti-windup
- Performance characteristics

## Quick Start Guide

### For New Users
1. Start with **[Usage Guide](usage.md)** for basic setup and installation
2. Run examples from **[Examples Guide](examples.md)** to understand applications
3. Refer to **[Troubleshooting Guide](troubleshooting.md)** when issues arise

### For Intermediate Users
1. Review **[Detailed Explanation](explanation.md)** for algorithm understanding
2. Study **[Examples Guide](examples.md)** for implementation patterns
3. Use **[Usage Guide](usage.md)** for configuration options

### For Advanced Users
1. Review **[System Architecture](system_architecture.md)** for design understanding
2. Study **[Internal Mechanisms](mechanism.md)** for technical implementation
3. Use **[Troubleshooting Guide](troubleshooting.md)** for debugging complex issues

### For Library Developers
1. **[System Architecture](system_architecture.md)** - overall design and patterns
2. **[Internal Mechanisms](mechanism.md)** - implementation details and algorithms
3. **[Detailed Explanation](explanation.md)** - design rationale and decisions

## Key Features Overview

### Core PID Control
- Standard PID algorithm with Arduino PID library integration
- Configurable proportional, integral, and derivative gains
- Configurable output limits and sample time
- Direct and reverse acting controller modes
- Proportional on Error (P_ON_E) and Proportional on Measurement (P_ON_M) modes

### Signal Processing
- Exponential Moving Average (EMA) input filtering for noise reduction
- Configurable output smoothing with rate limiting
- Sensor noise reduction and signal conditioning
- Mechanical stress prevention through gradual output changes

### Safety & Reliability
- Comprehensive failsafe mechanisms with multiple error states
- Input validation and timeout protection
- Stability monitoring with configurable tolerance and duration
- Automatic error detection with manual recovery confirmation
- Configurable failsafe output values for safe system states

### Monitoring & Debugging
- Extensive status monitoring and getter functions
- Real-time error state reporting and error type identification
- PID parameter access for tuning verification
- Saturation detection and stability status monitoring
- Last update time tracking for timeout diagnostics

### Performance & Efficiency
- Optimized for Arduino resource constraints
- Minimal memory footprint (450-1000 bytes RAM depending on features)
- Fast execution times (< 130μs per compute cycle)
- No dynamic memory allocation for predictable performance
- Efficient algorithms suitable for real-time control

## Example Applications

### Robotics & Automation
- **Motor Speed Control**: Precise DC motor velocity control with encoder feedback
- **Position Control**: Servo and actuator positioning systems with feedback
- **Motion Control**: Smooth acceleration and deceleration for robotic joints
- **Conveyor Systems**: Belt speed regulation with load compensation

### Environmental Control
- **Temperature Regulation**: Heating and cooling system control with thermal inertia
- **Humidity Control**: Climate system management for environmental chambers
- **Pressure Control**: Process pressure regulation in industrial applications
- **Flow Control**: Fluid flow rate maintenance in piping systems

### User Interface & Feedback
- **LED Brightness Control**: Adaptive display backlighting with ambient light compensation
- **Haptic Feedback**: Vibration motor intensity control for user feedback
- **Audio Level Control**: Speaker volume regulation with noise gating
- **Display Contrast**: Automatic adjustment based on ambient lighting

### Industrial & Process Control
- **Level Control**: Tank level maintenance in process systems
- **pH Control**: Chemical process pH regulation with electrode feedback
- **Mixing Control**: Agitator speed control for consistent mixing
- **Valve Control**: Proportional valve positioning for flow control

## Documentation Cross-References

### Learning Path Dependencies
```
Basic Usage
├── Usage Guide (Primary)
├── Examples Guide (Practical Application)
└── Troubleshooting Guide (Problem Resolution)

Advanced Understanding
├── Detailed Explanation (Algorithm Theory)
├── System Architecture (Design Patterns)
└── Internal Mechanisms (Implementation Details)
```

### Reference Relationships
- **Usage Guide** → Examples Guide (for practical implementation)
- **Examples Guide** → Troubleshooting Guide (for common issues)
- **Troubleshooting Guide** → Detailed Explanation (for root cause understanding)
- **Detailed Explanation** → Internal Mechanisms (for technical depth)
- **Internal Mechanisms** → System Architecture (for design rationale)

## Support and Resources

### Getting Help
- Start with the **[Troubleshooting Guide](troubleshooting.md)** for common issues
- Review relevant examples in **[Examples Guide](examples.md)** for similar applications
- Use the monitoring functions described in **[Usage Guide](usage.md)** for debugging
- Check **[Detailed Explanation](explanation.md)** for algorithm understanding

### Best Practices
- Always start with conservative PID values and tune gradually
- Enable appropriate filtering based on sensor noise characteristics
- Configure failsafes based on system safety requirements and failure modes
- Monitor system status regularly using built-in status functions
- Test failsafe behavior under controlled conditions

### Performance Optimization
- Select appropriate sample times based on system dynamics
- Use input filtering to reduce sensor noise and improve stability
- Configure output smoothing to prevent mechanical stress and oscillations
- Monitor memory usage on resource-constrained boards
- Profile execution times for real-time performance verification

### Development Workflow
1. **Planning**: Review requirements and select appropriate example as starting point
2. **Implementation**: Use Usage Guide for basic setup, Examples Guide for patterns
3. **Testing**: Verify with monitoring functions, test edge cases and failsafes
4. **Tuning**: Adjust PID parameters gradually, use Detailed Explanation for theory
5. **Optimization**: Apply performance tips from Usage Guide and Troubleshooting Guide
6. **Documentation**: Document custom configurations and tuning values

## Version Information

**Current Version:** AutoThrottleNG v1.2.0
**Compatibility:** Arduino IDE 1.8.0+, PlatformIO, VS Code with Arduino extension
**Dependencies:** Arduino PID library v1.2.0+ (automatically installed)
**Supported Architectures:** AVR (Uno, Mega, etc.), ESP32, ARM (Arduino framework)
**License:** MIT License
**Repository:** https://github.com/lily-osp/AutoThrottleNG

## Contributing to Documentation

### Documentation Standards
- Use clear, technical language without marketing buzzwords
- Include practical examples and code snippets
- Provide cross-references between related topics
- Include table of contents in all documentation files
- Use consistent formatting and terminology
- Test all code examples for compilation and functionality

### Content Organization
- **Conceptual**: How and why things work (Detailed Explanation, Internal Mechanisms)
- **Practical**: How to use and implement (Usage Guide, Examples Guide)
- **Reference**: Architecture and design (System Architecture, troubleshooting)
- **Support**: Problem resolution and debugging (Troubleshooting Guide)

This documentation is designed to support users at all levels, from beginners implementing their first PID controller to advanced users optimizing complex control systems. Each guide focuses on specific aspects while maintaining cross-references for understanding and practical application.
