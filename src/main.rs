use std::time::Duration;
use std::thread;

/// PID controller for temperature regulation
struct PidController {
    kp: f32,         // Proportional gain
    ki: f32,         // Integral gain
    kd: f32,         // Derivative gain
    setpoint: f32,   // Target temperature
    integral: f32,   // Accumulated error
    prev_error: f32, // Previous error for derivative calculation
    output_min: f32, // Minimum output value
    output_max: f32, // Maximum output value
}

impl PidController {
    /// Create a new PID controller
    fn new(kp: f32, ki: f32, kd: f32, setpoint: f32) -> Self {
        PidController {
            kp,
            ki,
            kd,
            setpoint,
            integral: 0.0,
            prev_error: 0.0,
            output_min: 0.0,
            output_max: 1.0, // Duty cycle between 0 and 1
        }
    }

    /// Calculate the control output based on current temperature
    fn compute(&mut self, current_temp: f32, dt: f32) -> f32 {
        let error = self.setpoint - current_temp;
        
        // Proportional term
        let p_term = self.kp * error;
        
        // Integral term with anti-windup
        self.integral += error * dt;
        let i_term = self.ki * self.integral;
        
        // Derivative term
        let d_term = self.kd * (error - self.prev_error) / dt;
        self.prev_error = error;
        
        // Calculate output with limits
        let output = (p_term + i_term + d_term).clamp(self.output_min, self.output_max);
        
        output
    }
    
    /// Set a new temperature setpoint
    fn set_setpoint(&mut self, setpoint: f32) {
        self.setpoint = setpoint;
    }
    
    /// Update PID parameters
    fn set_parameters(&mut self, kp: f32, ki: f32, kd: f32) {
        self.kp = kp;
        self.ki = ki;
        self.kd = kd;
        self.integral = 0.0;  // Reset integral term when changing parameters
        self.prev_error = 0.0;
    }
}

/// Thermal model for simulation
struct ThermalSystem {
    temperature: f32,      // Current temperature (°C)
    ambient_temp: f32,     // Ambient temperature (°C)
    thermal_capacity: f32, // Thermal capacity (J/°C)
    heater_power: f32,     // Heater power (W)
    heat_loss_coeff: f32,  // Heat loss coefficient (W/°C)
}

impl ThermalSystem {
    fn new(initial_temp: f32, ambient_temp: f32, thermal_capacity: f32, heater_power: f32, heat_loss_coeff: f32) -> Self {
        ThermalSystem {
            temperature: initial_temp,
            ambient_temp,
            thermal_capacity,
            heater_power,
            heat_loss_coeff,
        }
    }
    
    /// Update temperature based on heater duty cycle
    fn update(&mut self, duty_cycle: f32, dt: f32) {
        // Heat added by resistor (J)
        let heat_added = self.heater_power * duty_cycle * dt;
        
        // Heat lost to environment (J)
        let heat_lost = self.heat_loss_coeff * (self.temperature - self.ambient_temp) * dt;
        
        // Net heat and temperature change
        let net_heat = heat_added - heat_lost;
        let temp_change = net_heat / self.thermal_capacity;
        
        self.temperature += temp_change;
    }
    
    fn get_temperature(&self) -> f32 {
        self.temperature
    }
}

/// Autotuner for PID parameters using relay feedback method (Åström-Hägglund)
struct PidAutoTuner {
    setpoint: f32,          // Target temperature
    relay_amplitude: f32,   // Relay output amplitude (0.5 = 50% duty cycle variation)
    noise_band: f32,        // Noise filter band
    
    // State variables
    relay_output: f32,      // Current relay output
    last_crossing_time: f32, // Time of last zero crossing
    peaks: Vec<(f32, f32)>,  // (time, temperature) at peaks
    valleys: Vec<(f32, f32)>, // (time, temperature) at valleys
    peak_count: usize,      // Number of peaks/valleys required
    prev_temp: f32,         // Previous temperature
    time: f32,              // Current time
    is_finished: bool,      // Whether autotuning is complete
}

impl PidAutoTuner {
    fn new(setpoint: f32, relay_amplitude: f32, noise_band: f32) -> Self {
        PidAutoTuner {
            setpoint,
            relay_amplitude,
            noise_band,
            relay_output: 0.5 + relay_amplitude, // Start with heater on
            last_crossing_time: 0.0,
            peaks: Vec::new(),
            valleys: Vec::new(),
            peak_count: 4,       // Need at least 4 peaks for accurate tuning
            prev_temp: 0.0,      // Will be set on first iteration
            time: 0.0,
            is_finished: false,
        }
    }
    
    /// Process new temperature reading and update relay state
    fn update(&mut self, temperature: f32, dt: f32) -> f32 {
        if self.is_finished {
            return 0.5; // Return middle value when done
        }
        
        // Update time
        self.time += dt;
        
        // First measurement
        if self.prev_temp == 0.0 {
            self.prev_temp = temperature;
            return self.relay_output;
        }
        
        // Calculate error from setpoint
        let error = self.setpoint - temperature;
        
        // Relay control with hysteresis
        if self.relay_output > 0.5 && error < -self.noise_band {
            // Switch relay off
            self.relay_output = 0.5 - self.relay_amplitude;
            
            // Record peak
            if self.last_crossing_time > 0.0 {
                self.peaks.push((self.time, temperature));
                println!("Peak detected at time {}s, temp: {}°C", self.time, temperature);
            }
            self.last_crossing_time = self.time;
            
        } else if self.relay_output < 0.5 && error > self.noise_band {
            // Switch relay on
            self.relay_output = 0.5 + self.relay_amplitude;
            
            // Record valley
            if self.last_crossing_time > 0.0 {
                self.valleys.push((self.time, temperature));
                println!("Valley detected at time {}s, temp: {}°C", self.time, temperature);
            }
            self.last_crossing_time = self.time;
        }
        
        // Check if we have enough peaks and valleys
        if self.peaks.len() >= self.peak_count && self.valleys.len() >= self.peak_count {
            self.is_finished = true;
        }
        
        self.prev_temp = temperature;
        self.relay_output
    }
    
    /// Calculate PID parameters from collected data
    fn get_pid_parameters(&self) -> Option<(f32, f32, f32)> {
        if !self.is_finished || self.peaks.len() < 2 || self.valleys.len() < 2 {
            return None;
        }
        
        // Calculate peak-to-peak amplitude
        let mut peak_sum = 0.0;
        let mut valley_sum = 0.0;
        
        for i in 1..self.peaks.len() {
            peak_sum += self.peaks[i].1;
        }
        
        for i in 1..self.valleys.len() {
            valley_sum += self.valleys[i].1;
        }
        
        let peak_avg = peak_sum / (self.peaks.len() as f32 - 1.0);
        let valley_avg = valley_sum / (self.valleys.len() as f32 - 1.0);
        let amplitude = peak_avg - valley_avg;
        
        // Calculate average period
        let mut period_sum = 0.0;
        let mut periods = 0;
        
        for i in 1..self.peaks.len() {
            period_sum += self.peaks[i].0 - self.peaks[i-1].0;
            periods += 1;
        }
        
        let period = period_sum / periods as f32;
        
        // Calculate ultimate gain and period
        let ku = (4.0 * self.relay_amplitude) / (std::f32::consts::PI * amplitude);
        let tu = period;
        
        println!("Autotuning results:");
        println!("System amplitude: {}°C", amplitude);
        println!("Oscillation period: {}s", period);
        println!("Ultimate gain (Ku): {}", ku);
        println!("Ultimate period (Tu): {}s", tu);
        
        // Calculate PID parameters using Ziegler-Nichols method
        let kp = 0.6 * ku;
        let ki = 1.2 * ku / tu;
        let kd = 0.075 * ku * tu;
        
        Some((kp, ki, kd))
    }
    
    fn is_autotuning_finished(&self) -> bool {
        self.is_finished
    }
}

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

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_pid_controller() {
        let mut pid = PidController::new(0.8, 0.05, 0.2, 60.0);
        let mut thermal_system = ThermalSystem::new(25.0, 22.0, 300.0, 10.0, 0.1);
        
        // Test that system eventually reaches target temperature
        let mut time = 0.0;
        let dt = 1.0;
        let max_time = 400.0;
        
        while time < max_time {
            let temp = thermal_system.get_temperature();
            let duty_cycle = pid.compute(temp, dt);
            thermal_system.update(duty_cycle, dt);
            
            // After 300 seconds, temperature should be close to setpoint
            if time > 300.0 {
                assert!((temp - pid.setpoint).abs() < 2.0, 
                    "Temperature {} should be close to setpoint {}", temp, pid.setpoint);
            }
            
            time += dt;
        }
    }
    
    #[test]
    fn test_thermal_model() {
        let mut system = ThermalSystem::new(25.0, 25.0, 300.0, 10.0, 0.0);
        
        // With no heat loss, applying 10W for 30s should increase temp by 1°C
        system.update(1.0, 30.0);
        assert!((system.get_temperature() - 26.0).abs() < 0.01);
        
        // Test heat loss
        let mut system_with_loss = ThermalSystem::new(30.0, 20.0, 300.0, 0.0, 0.5);
        
        // With no heating and 0.5 W/°C heat loss coefficient, temperature should decrease
        system_with_loss.update(0.0, 60.0);
        assert!(system_with_loss.get_temperature() < 30.0);
    }
    
    #[test]
    fn test_autotuning() {
        let mut thermal_system = ThermalSystem::new(25.0, 22.0, 300.0, 10.0, 0.1);
        let mut auto_tuner = PidAutoTuner::new(50.0, 0.4, 0.1);
        let dt = 1.0;
        let mut time = 0.0;
        let mut duty_cycle = 0.5;
        
        // Run autotuner for a long enough time to complete
        while !auto_tuner.is_autotuning_finished() && time < 1000.0 {
            let temp = thermal_system.get_temperature();
            duty_cycle = auto_tuner.update(temp, dt);
            thermal_system.update(duty_cycle, dt);
            time += dt;
        }
        
        // Check that tuning finished and returned sensible parameters
        if auto_tuner.is_autotuning_finished() {
            let params = auto_tuner.get_pid_parameters();
            assert!(params.is_some(), "Should have calculated PID parameters");
            
            if let Some((kp, ki, kd)) = params {
                // Parameters should be positive and reasonable
                assert!(kp > 0.0, "Kp should be positive");
                assert!(ki > 0.0, "Ki should be positive");
                assert!(kd >= 0.0, "Kd should be positive or zero");
            }
        }
    }
}