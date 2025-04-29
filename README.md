# PID Temperature Controller in Rust

A reusable PID controller implementation in Rust with simulation capabilities and automatic PID parameter tuning.

## Overview

This project provides a complete temperature control system using a PID (Proportional-Integral-Derivative) controller implemented in Rust. It includes:

- A reusable PID controller library with configurable parameters
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

## Project Structure

- **Library**: Reusable components for PID control and thermal simulation
  - `PidController`: The core PID controller logic
  - `ThermalSystem`: Thermal model for simulating temperature dynamics
  - `PidAutoTuner`: Automatic parameter tuning using relay feedback

- **Binary**: Simulation tool to demonstrate the PID controller in action
  - `pid-simulation`: Runs a complete thermal control simulation with autotuning

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

## Using the Library

Add this to your `Cargo.toml`:

```toml
[dependencies]
pid_controller = { git = "https://github.com/sctg-development/rust-pid-controller.git" }
```

Example usage:

```rust
use pid_controller::{PidController, ThermalSystem};

// Create a PID controller with parameters and setpoint
let mut pid = PidController::new(0.5, 0.01, 0.2, 50.0);

// Get the current temperature from your sensor
let current_temp = 25.0;

// Calculate control output (dt = time since last calculation in seconds)
let dt = 1.0;
let duty_cycle = pid.compute(current_temp, dt);

// Use duty_cycle to control your heating element (0.0-1.0)
```

## Running the Simulation

```bash
cargo run --bin pid-simulation
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
- Integration into hardware control systems

## Raspberry Pi Implementation

This project includes a specialized binary (`rpi-48C-pid-controller`) that implements the PID controller on Raspberry Pi hardware for precise temperature control at 48°C.

### Hardware Setup

- **Raspberry Pi** (any model with I2C and PWM)
- **Texas Instruments ADS1115** ADC for temperature measurement
- **NTC thermistor** (10kΩ @ 25°C, β=3950)
- **Voltage divider** with 10kΩ series resistor
- **4.096V precision voltage reference**
- **10W heater element**
- **MOSFET/transistor** for PWM control of the heater

### Wiring Instructions

1. **NTC Temperature Sensor:**
   - Connect the 10kΩ series resistor from 4.096V reference to ADS1115 A0 input
   - Connect the NTC thermistor from ADS1115 A0 input to GND
   - Connect ADS1115 to Raspberry Pi I2C pins (SDA to GPIO2, SCL to GPIO3)

2. **Heater Control:**
   - Connect Raspberry Pi PWM0 (GPIO18) to MOSFET gate/transistor base
   - Connect MOSFET drain/collector to heater and appropriate power supply

### Building for Raspberry Pi

```bash
cargo build --target=arm-unknown-linux-gnueabihf --bin rpi-48C-pid-controller --features="rpi"
```

Transfer the compiled binary to your Raspberry Pi and run:

```bash
./rpi-48C-pid-controller
```

### Operation

1. The controller first runs an autotuning process to determine optimal PID parameters
2. After autotuning, it applies the tuned parameters to maintain a precise 48°C temperature
3. Real-time temperature and control output are displayed continuously

### Customization

You can modify these parameters in `src/bin/rpi_controller.rs`:
- `TARGET_TEMP`: Target temperature (default: 48.0°C)
- `NTC_R25`: NTC resistance at 25°C (default: 10,000Ω)
- `NTC_BETA`: NTC beta coefficient (default: 3950)
- `SAMPLING_INTERVAL`: Temperature reading frequency (default: 1000ms)
- `PWM_FREQUENCY`: PWM frequency for heater control (default: 1Hz)

## License

MIT