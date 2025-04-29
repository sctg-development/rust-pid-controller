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

use pid_controller::{PidController, PidAutoTuner};
use ads1x1x::{channel, Ads1x1x, FullScaleRange, TargetAddr};
use linux_embedded_hal::I2cdev;
use nb::block;
use rppal::pwm::{Channel, Polarity, Pwm};
use std::error::Error;
use std::thread;
use std::time::{Duration, Instant};

// NTC thermistor parameters
const NTC_R25: f32 = 10_000.0;  // NTC resistance at 25°C (10k)
const NTC_BETA: f32 = 3950.0;   // NTC beta value (typical for 10k NTC)
const NTC_SERIES_RESISTOR: f32 = 10_000.0; // Series resistor value (10k)
const REF_VOLTAGE: f32 = 4.096; // Reference voltage (V)
const ABS_ZERO: f32 = 273.15;   // Absolute zero in Celsius
const T25_KELVIN: f32 = 25.0 + ABS_ZERO; // 25°C in Kelvin

// Control parameters
const TARGET_TEMP: f32 = 48.0;  // Target temperature (°C)
const PWM_FREQUENCY: f64 = 1.0; // PWM frequency in Hz
const SAMPLING_INTERVAL: u64 = 1000; // Temperature sampling interval (ms)

// Autotuning parameters
const RELAY_AMPLITUDE: f32 = 0.4; // Relay amplitude (0.4 = 40% variation around midpoint)
const NOISE_BAND: f32 = 0.1;      // Noise band for autotuning (°C)

fn main() -> Result<(), Box<dyn Error>> {
    println!("Raspberry Pi PID Temperature Controller");
    println!("Target temperature: {}°C", TARGET_TEMP);
    
    // Initialize I2C and ADC
    let i2c_dev = I2cdev::new("/dev/i2c-1")?;
    let mut adc = Ads1x1x::new_ads1115(i2c_dev, TargetAddr::default());
    adc.set_full_scale_range(FullScaleRange::Within4_096V).unwrap();
    
    // Initialize PWM for heater control
    let mut pwm = Pwm::with_frequency(
        Channel::Pwm0,
        PWM_FREQUENCY,
        0.0, // Start with heater off
        Polarity::Normal,
        true
    )?;
    
    // Perform autotuning to determine optimal PID parameters
    println!("\nStarting autotuning process at setpoint {}°C", TARGET_TEMP);
    println!("Time(s) | Temperature(°C) | Output");
    println!("--------|-----------------|-------");
    
    let mut auto_tuner = PidAutoTuner::new(TARGET_TEMP, RELAY_AMPLITUDE, NOISE_BAND);
    let start_time = Instant::now();
    let mut next_sample_time = Instant::now();
    
    // Run autotuning process
    while !auto_tuner.is_autotuning_finished() {
        let now = Instant::now();
        
        // Check if it's time to take a temperature measurement
        if now >= next_sample_time {
            // Read temperature from NTC
            let temperature = read_temperature(&mut adc)?;
            let elapsed = now.duration_since(start_time).as_secs_f32();
            
            // Update autotuner with new temperature
            let duty_cycle = auto_tuner.update(temperature, SAMPLING_INTERVAL as f32 / 1000.0);
            pwm.set_duty_cycle(duty_cycle as f64)?;
            
            // Print status
            println!("{:6.1} | {:15.2} | {:6.3}", 
                     elapsed, temperature, duty_cycle);
            
            // Schedule next sample
            next_sample_time = now + Duration::from_millis(SAMPLING_INTERVAL);
        }
        
        // Small sleep to prevent CPU hogging
        thread::sleep(Duration::from_millis(10));
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
    
    // Create PID controller with tuned parameters
    let mut pid = PidController::new(kp, ki, kd, TARGET_TEMP);
    
    // Start normal control with tuned parameters
    println!("\nStarting control with autotuned parameters:");
    println!("Time(s) | Temperature(°C) | Duty Cycle");
    println!("--------|-----------------|----------");
    
    let control_start_time = Instant::now();
    next_sample_time = Instant::now();
    
    // Main control loop
    loop {
        let now = Instant::now();
        
        // Check if it's time to take a temperature measurement
        if now >= next_sample_time {
            // Read temperature from NTC
            let temperature = read_temperature(&mut adc)?;
            let elapsed = now.duration_since(control_start_time).as_secs_f32();
            
            // Calculate PID output
            let duty_cycle = pid.compute(temperature, SAMPLING_INTERVAL as f32 / 1000.0);
            pwm.set_duty_cycle(duty_cycle as f64)?;
            
            // Print status
            println!("{:6.1} | {:15.2} | {:10.3}", 
                     elapsed, temperature, duty_cycle);
            
            // Schedule next sample
            next_sample_time = now + Duration::from_millis(SAMPLING_INTERVAL);
        }
        
        // Small sleep to prevent CPU hogging
        thread::sleep(Duration::from_millis(10));
    }
}

/// Read temperature from NTC thermistor via ADS1115 ADC
fn read_temperature<I2C, E>(adc: &mut Ads1x1x<I2C, ads1x1x::ic::Ads1115, ads1x1x::ic::Resolution16Bit, ads1x1x::mode::OneShot>) 
    -> Result<f32, Box<dyn Error>>
where
    I2C: embedded_hal::i2c::I2c<Error = E>,
    E: std::error::Error + 'static,
{
    // Read raw ADC value
    let raw_value = block!(adc.read(channel::SingleA0)).unwrap();
    
    // Convert raw value to voltage (ADS1115 is 16-bit signed, -32768 to +32767)
    let voltage = (raw_value as f32 / 32768.0) * REF_VOLTAGE;
    
    // Calculate NTC resistance from voltage (voltage divider)
    // NTC is connected between ADC input and GND
    // Series resistor is connected between ADC input and VREF
    let ntc_resistance = if voltage > 0.0 && voltage < REF_VOLTAGE {
        NTC_SERIES_RESISTOR * voltage / (REF_VOLTAGE - voltage)
    } else {
        return Err("Invalid voltage reading".into());
    };
    
    // Convert resistance to temperature using simplified Steinhart-Hart equation
    // 1/T = 1/T0 + (1/B) * ln(R/R0)
    let temp_kelvin = 1.0 / (1.0 / T25_KELVIN + (1.0 / NTC_BETA) * (ntc_resistance / NTC_R25).ln());
    let temp_celsius = temp_kelvin - ABS_ZERO;
    
    Ok(temp_celsius)
}