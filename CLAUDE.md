# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Smart follow-me suitcase (智能跟随行李箱) firmware for ESP32-S3. Uses UWB for target tracking, IMU for heading stabilization, and differential-drive chassis with ESC motors and encoder-based PID speed control.

## Build & Flash

```bash
# Set target and configure
idf.py set-target esp32s3
idf.py menuconfig    # Configure pins, PID gains, follow parameters under "Follow-only suitcase"

# Build and flash
idf.py build
idf.py flash monitor
```

## Running Tests (PC-side)

Tests run on the host with gcc, no hardware required:

```bash
# Algorithm tests (kinematics, follow/avoid logic)
./tests/algorithm/run_tests.sh

# Protocol tests (sensor parsers: UWB, ultrasonic, FSR)
./tests/protocol/run_tests.sh
```

Test binaries compile to `work/test-build/`.

## Architecture

### Component Organization

- **components/sensors/** — Hardware drivers, each with init/read/deinit API:
  - `bu_uwb/` — BU03/BU04 UWB (UART, TWR JSON parsing)
  - `imu_i2c/` — 9-axis IMU (I2C, quaternion/euler output)
  - `a02yyuw/` — Ultrasonic distance sensors (software UART)
  - `rplidar_c1/` — RPLIDAR C1 laser scanner
  - `fsr_adc/` — Force-sensitive resistors
  - `vl53l1x_tof/` — Time-of-flight sensor

- **components/control/** — Motion control:
  - `chassis/` — Closed-loop differential drive: RC PWM ESC output + AB quadrature encoder PID + feed-forward
  - `follow_avoid/` — Follow + VFH obstacle avoidance algorithm

### Main Examples

- **examples/follow_only/** — Pure following (UWB + IMU + chassis only, no obstacle avoidance). 3-state machine: IDLE/SEARCH/FOLLOW. This is the primary reference for the follow algorithm.
- **examples/follow_robot/** — Full version with lidar, ultrasonics, and VFH avoidance. 5-state machine adds AVOID/ESTOP states.

### Concurrency Model

FreeRTOS tasks communicate via mutex-protected `shared_t` snapshot structs. Each sensor runs in its own task (or is read synchronously in the control task). The control task runs at fixed rate (default 50Hz) and:
1. Reads sensor snapshot
2. Runs state machine
3. Applies IMU heading closed-loop correction
4. Commands chassis velocity → encoder PID → ESC pulses

### Chassis API Flow

```
chassis_set_velocity(&ch, v, omega)  →  per-wheel speed targets
chassis_update(&ch, dt)              →  read encoders → PID → write ESC pulses
```

Sign convention: `v > 0` = forward, `omega > 0` = turn LEFT (CCW).

### Coordinate/Sign Conventions

- `FR_UWB_LEFT_SIGN` — flip if UWB bearing direction is inverted
- `FR_IMU_YAW_SIGN` — flip if IMU yaw rotation is inverted
- `FR_LEFT_INVERT` / `FR_RIGHT_INVERT` — flip ESC forward/reverse per wheel
- `FR_LEFT_ENC_INVERT` / `FR_RIGHT_ENC_INVERT` — flip encoder count direction

## Key Calibration

`TICKS_PER_METER` must be physically calibrated: push robot exactly 1m, read log tick delta. Default 2000 (4x quadrature).

## Working Rules

- Temporary analysis goes in `work/`
- User-facing generated files go in `outputs/`
- Durable documentation goes in `docs/`
- Do not modify files outside this project unless explicitly requested
