# Mecanum Wheel Robot — Kinematic Modeling & Control

Kinematic modeling and simulation of a **4-wheel Mecanum robot** (WHEELTEC platform, STM32-based), developed for the *Modeling and Simulation* course at Southern University of Science and Technology (SUSTech).

## Project Overview

This project focuses on the kinematic analysis and motion control of a Mecanum-wheeled mobile robot. The robot uses four omnidirectional Mecanum wheels oriented at ±45°, enabling holonomic motion (forward/backward, left/right, rotation) with a single chassis.

## Specifications

| Parameter | Value |
|-----------|-------|
| Platform | WHEELTEC Mecanum Wheel Robot |
| MCU | STM32 |
| Wheel Outer Diameter | 74.78 mm |
| Front-Back Wheelbase | 173 mm |
| Left-Right Track | 194.2 mm |
| Encoder Type | GMR encoder |
| Motion | Omnidirectional (3 DOF) |

## Kinematic Model

The Mecanum wheel forward kinematics matrix:

```
[v_x]   [ 1   1   1   1 ]   [ω₁]
[v_y] = [ 1  -1  -1   1 ] × [ω₂] × (r/4)
[ω  ]   [ 1  -1   1  -1 ]   [ω₃]
            [ω₄]
```

Where:
- `v_x, v_y`: Linear velocities in body frame
- `ω`: Angular velocity of robot
- `ω₁~ω₄`: Wheel angular velocities
- `r`: Wheel radius

## Inverse Kinematics (Wheel Speed Allocation)

```python
# Forward = all wheels forward
# Left    = FL+BL forward, FR+BR reverse
# Right   = FL+FR forward, BL+BR reverse  
# Rotate  = FL+BR forward, FR+BL reverse
```

## Project Structure

```
├── stm32_code/              # STM32 source code (C, modifies oblique region for figure-8 motion)
├── kinematics_model.ipynb   # Kinematic modeling (if available)
└── README.md
```

## STM32 Code

The STM32 firmware implements:
- GMR encoder reading (quadrature decoding)
- Wheel speed closed-loop control (PI)
- Figure-8 trajectory generation (modified in the "oblique region")
- Serial debug output

**Note**: The STM32 code has two versions:
- `2024年旧Mec小车（1辆）`: Legacy version
- `2026年新Mec小车（3辆）`: Updated version with GMR encoders

## Course

Modeling and Simulation, Southern University of Science and Technology, Spring 2026.

---
**Author**: Lixian Xue (薛力衔) — [LixianXue3](https://github.com/LixianXue3)  
**Student ID**: 12411409
