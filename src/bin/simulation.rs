// MIT License
//
// Copyright (c) 2025 Ronan LE MEILLAT for SCTG Development
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

use pid_controller::{PidController, ThermalSystem, PidAutoTuner};



fn main() {
    // Create thermal system simulation
    // Starting at 25°C, ambient 22°C, 300J/°C heat capacity,
    // 10W resistor, heat loss coefficient 0.1 W/°C
    let mut thermal_system = ThermalSystem::new(25.0, 22.0, 300.0, 10.0, 0.1);
    
    // Set temperature target
    let setpoint = 48.0;
    
    // First, run autotuning process
    println!("Starting autotuning process at setpoint {}°C", setpoint);
    println!("Time(s) | Temperature(°C) | Output");
    println!("--------|-----------------|-------");
    
    let mut auto_tuner = PidAutoTuner::new(setpoint, 0.4, 0.1);
    
    // Simulation parameters
    let dt = 0.1; // time step in seconds
    let sampling_interval = 1.0; // Read temperature every 1s
    let mut elapsed_time = 0.0;
    let mut next_sample_time = 0.0;
    let mut duty_cycle = 0.0;
    
    // Run autotuning
    while !auto_tuner.is_autotuning_finished() && elapsed_time < 3600.0 {
        // Read temperature and update relay at sampling intervals
        if elapsed_time >= next_sample_time {
            let temperature = thermal_system.get_temperature();
            duty_cycle = auto_tuner.update(temperature, sampling_interval);
            
            println!("{:6.1} | {:15.2} | {:6.3}", 
                     elapsed_time, temperature, duty_cycle);
            
            next_sample_time += sampling_interval;
        }
        
        // Update thermal system simulation
        thermal_system.update(duty_cycle, dt);
        elapsed_time += dt;
    }
    
    // Get tuned PID parameters
    let parameters = auto_tuner.get_pid_parameters();
    
    let (kp, ki, kd) = match parameters {
        Some((kp, ki, kd)) => {
            println!("\nAutotuning complete!");
            println!("Recommended PID parameters:");
            println!("Kp = {:.3}", kp);
            println!("Ki = {:.3}", ki);
            println!("Kd = {:.3}", kd);
            (kp, ki, kd)
        },
        None => {
            println!("\nAutotuning failed, using default parameters");
            (0.8, 0.05, 0.2) // Default parameters
        }
    };
    
    // Create PID controller with autotuned parameters
    let mut pid = PidController::new(kp, ki, kd, setpoint);
    
    // Reset time and run normal simulation with tuned parameters
    println!("\nStarting control with autotuned parameters:");
    println!("Time(s) | Temperature(°C) | Duty Cycle");
    println!("--------|-----------------|----------");
    
    elapsed_time = 0.0;
    next_sample_time = 0.0;
    let simulation_duration = 300.0;
    
    while elapsed_time < simulation_duration {
        // Read temperature and update PID at sampling intervals
        if elapsed_time >= next_sample_time {
            let temperature = thermal_system.get_temperature();
            duty_cycle = pid.compute(temperature, sampling_interval);
            
            println!("{:6.1} | {:15.2} | {:10.3}", 
                     elapsed_time, temperature, duty_cycle);
            
            next_sample_time += sampling_interval;
        }
        
        // Update thermal system simulation
        thermal_system.update(duty_cycle, dt);
        elapsed_time += dt;
    }
    
    println!("\nSimulation complete!");
}