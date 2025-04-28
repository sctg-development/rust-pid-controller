# PID Temperature Controller in Rust

A simulation-based implementation of a PID controller for temperature regulation with automatic PID parameter tuning.

## Overview

This project demonstrates a complete temperature control system using a PID (Proportional-Integral-Derivative) controller implemented in Rust. It includes:

- A PID controller implementation with configurable parameters
- A thermal system simulation modeling a heating element and thermal dynamics
- An autotuning module that automatically determines optimal PID parameters
- Comprehensive testing to validate controller behavior

The system simulates controlling the temperature of a thermal mass using a 10W heating element (resistor), with temperature readings every 1 second and a PWM-style duty cycle output to control the heater power.

## Features

- **PID Controller**: Classic PID implementation with anti-windup protection
- **Thermal System Simulation**: Physics-based model with configurable parameters:
  - Thermal capacity (J/°C)
  - Heater power (W)
  - Heat loss coefficient (W/°C)
  - Ambient temperature
- **Automatic PID Tuning**: Relay-feedback method (Åström-Hägglund) to determine optimal control parameters
- **Visualization**: Console-based output of temperature response and control signals

## How It Works

### PID Controller

The PID controller calculates control output based on three terms:
- **Proportional**: Immediate response to current error
- **Integral**: Response to accumulated error over time
- **Derivative**: Response to rate of change of error

The controller automatically limits output to a valid range (0.0-1.0) suitable for PWM control.

### Autotuning

The system uses relay-feedback autotuning to determine optimal PID parameters:

1. A relay controller creates controlled oscillations around the setpoint
2. The system measures temperature peaks, valleys, and oscillation periods
3. Using the Ziegler-Nichols method, it calculates optimal Kp, Ki, and Kd values
4. The resulting parameters are applied to the PID controller

## Running the Simulation

```bash
cargo run
```

The output shows:
1. The autotuning process with time, temperature, and relay output
2. Calculated PID parameters
3. A simulation run with the tuned PID controller

## Example Output

```
Starting autotuning process at setpoint 48.0°C
Time(s) | Temperature(°C) | Output
--------|-----------------|-------
    0.0 |           25.00 | 0.900
    1.0 |           25.08 | 0.900
...
Peak detected at time 145.0s, temp: 49.23°C
Valley detected at time 204.0s, temp: 46.85°C
...
Autotuning results:
System amplitude: 2.38°C
Oscillation period: 118.0s
Ultimate gain (Ku): 0.675
Ultimate period (Tu): 118.0s

Autotuning complete!
Recommended PID parameters:
Kp = 0.405
Ki = 0.007
Kd = 5.962

Starting control with autotuned parameters:
Time(s) | Temperature(°C) | Duty Cycle
--------|-----------------|----------
    0.0 |           48.04 |     0.000
    1.0 |           47.97 |     0.012
...
```

## Testing

The project includes several tests:

```bash
cargo test
```

Tests validate:
- PID controller convergence to setpoint
- Thermal system physics
- Autotuning parameter calculation

## System Requirements

- Rust 1.54.0 or higher
- No external dependencies

## Use Cases

- Educational tool for understanding PID control
- Testing different PID tuning methods
- Simulating thermal systems with various characteristics
- Starting point for embedded temperature control projects

## License

MIT