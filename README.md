# ALYX Embedded Control Stack

Low-level embedded control software for **ALYX**, Project MANAS' autonomous ground vehicle developed for the **Intelligent Ground Vehicle Competition (IGVC) 2026**.

The stack runs on an **STM32 Nucleo-G491RE** and provides deterministic real-time control of:

- 4 independent drive motors
- 4 independent steering modules
- Encoder feedback acquisition
- Closed-loop PID velocity control
- Homing and steering calibration
- E-Stop handling
- ROS2 communication interface

The controller operates independently of the onboard computer and executes the drive control loop at **100 Hz**, ensuring deterministic motor control for autonomous navigation.

---

## Competition Results

ALYX competed at **IGVC 2026**, achieving:

- 🥈 2nd Place Top Performer Award
- 🏅 4th Place Self-Drive Grand Award
- 🏅 7th Place AutoNav Challenge

---

## Robot Architecture

```text
ROS2 Navigation Stack
        │
        │ UART5
        ▼
┌───────────────────────┐
│ STM32G491RE Controller│
└───────────────────────┘
      │          │
      │          │
      ▼          ▼
 Drive Motors   Steering Modules
 (BDC + PID)    (Stepper Control)

      ▲
      │
 Encoders + Limit Switches
```

---

## Hardware Overview

### Drive System

Each wheel module contains:

- 1 BDC drive motor
- Quadrature encoder feedback
- Dedicated PID controller
- PWM motor driver interface

The embedded controller maintains target wheel RPM using encoder feedback and a closed-loop PID velocity controller.

### Steering System

Each wheel module contains:

- 1 stepper motor
- Mechanical limit switch
- Homing routine
- Absolute angle tracking

Steering modules support:

- Startup homing
- Angle-based commands
- Constraint-aware shortest-path steering
- Optional RPM smoothing
- Queue-based command execution
- Runtime preemption handling

---

## Software Architecture

```text
main.c
│
├── motor_driver
│   ├── command parsing
│   └── PWM motor control
│
├── encoder
│   ├── RPM calculation
│   └── moving average filtering
│
├── pid_controller
│   ├── velocity control
│   ├── anti-windup
│   ├── slew limiting
│   └── deadband handling
│
└── Stepperv2
    ├── homing
    ├── angle control
    ├── queue management
    └── one-pulse mode control
```

---

## Features

### Closed-Loop Velocity Control

Each drive motor runs an independent PID controller featuring:

- Proportional, Integral and Derivative control
- Anti-windup protection
- Output saturation
- Slew-rate limiting
- Measurement deadbands
- Integral decay near zero setpoints

### Encoder Processing

Encoder subsystem provides:

- Quadrature encoder acquisition
- RPM estimation
- Exponential moving-average filtering
- Noise reduction for stable velocity feedback

### Steering Control

Steering subsystem supports:

- Startup homing
- Absolute angle tracking
- Steering constraint enforcement
- Dead-zone avoidance around limit switches
- Runtime command preemption
- Queue and non-queue operating modes

### Safety Features

Implemented safety systems include:

- Wired E-Stop
- Wireless E-Stop
- Heartbeat watchdog
- Motor shutdown logic
- Mode ownership and arbitration
- Safe startup homing sequence

---

## Operating Modes

| Mode | Description |
|--------|-------------|
| HOMING | Initial steering calibration |
| TELEOP | Manual control |
| AUTONAV | Autonomous navigation |
| SELFDRIVE | Structured road environment mode |
| IDLE | Standby mode |
| ESTOP | Immediate safe shutdown |

The embedded controller synchronizes operating modes with the robot's higher-level ROS2 software stack.

---

## Communication Protocol

Commands are transmitted over UART from the onboard computer.

Example command packet:

```text
B1 120
B2 120
B3 120
B4 120

S1 45
S2 45
S3 45
S4 45
```

Where:

| Command | Description |
|----------|-------------|
| Bx RPM | Drive motor RPM command |
| Sx Angle | Steering angle command |

Examples:

```text
B1 150
```

Set Motor 1 target velocity to 150 RPM.

```text
S3 -90
```

Rotate Steering Module 3 to -90°.

The parser supports simultaneous drive and steering updates for all wheel modules.

---

## Control Loop

| Parameter | Value |
|------------|---------|
| Control Frequency | 100 Hz |
| Encoder CPR | 1993 |
| Steering Resolution | 800 Steps/Rev × 5 Gear Ratio |
| MCU | STM32G491RE |
| Communication | UART + DMA |
| PWM Control | Hardware Timers |
| Encoder Interface | Hardware Encoder Mode |

---

## Repository Structure

```text
Alyx_Stack/
│
├── Core/
│   │
│   ├── Inc/
│   │   ├── encoder.h
│   │   ├── globals.h
│   │   ├── main.h
│   │   ├── motor_driver.h
│   │   ├── pid_controller.h
│   │   ├── Stepperv2.h
│   │   ├── stm32g4xx_hal_conf.h
│   │   └── stm32g4xx_it.h
│   │
│   ├── Src/
│   │   ├── encoder.c
│   │   ├── main.c
│   │   ├── motor_driver.c
│   │   ├── pid_controller.c
│   │   ├── Stepperv2.c
│   │   ├── stm32g4xx_hal_msp.c
│   │   ├── stm32g4xx_it.c
│   │   ├── syscalls.c
│   │   ├── sysmem.c
│   │   └── system_stm32g4xx.c
│   │
│   └── Startup/
│       └── startup_stm32g491retx.s
│
├── Drivers/
│   ├── CMSIS/
│   └── STM32G4xx_HAL_Driver/
│
├── Alyx_Stack.ioc
├── STM32G491RETX_FLASH.ld
├── STM32G491RETX_RAM.ld
├── .project
├── .cproject
└── README.md
```

### Core Modules

| Module | Responsibility |
|----------|---------------|
| `motor_driver` | PWM generation, motor direction control and command parsing |
| `encoder` | Quadrature encoder processing, RPM estimation and filtering |
| `pid_controller` | Closed-loop velocity control with anti-windup and slew limiting |
| `Stepperv2` | Steering control, homing, constraint handling and queue management |
| `main` | System initialization, scheduling, mode management and communication |
| `globals` | Shared system state across modules |

### Toolchain

- STM32CubeIDE
- STM32 HAL
- CMSIS
- STM32G491RE (ARM Cortex-M4F)
```

---

```
## Repository Branches

| Branch | Purpose |
|----------|----------|
| `main` | Stable working branch |
| `pid-autotuner` | PID autotuning experiments |
| `without_Onepulsemode` | Legacy steering implementation without One Pulse Mode |
| `working-shit-(avoid-main)` | Active development branch |

---

## Future Work

- ROS2 CAN/UART abstraction layer
- Automatic wheel calibration storage in flash
- Dynamic PID autotuning
- Improved steering trajectory generation
- Odometry feedback integration
- CAN-based motor controller interface
- Hardware fault diagnostics

---

## Team

Developed as part of **Project MANAS** for the **Intelligent Ground Vehicle Competition (IGVC) 2026**.

**Sensing & Automation**

- Akshat Kakade
- Aryan Pagaria
- Shubh Kesarwani
- Soham Saxena
- Taman Raja Kochi

---

## License

This repository is intended for educational and research purposes.

```
*"If it compiles first try, something is probably unplugged."*
```
