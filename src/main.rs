// Rust port of fancontrol.py.

mod fancontrol
{
use serde::{Deserialize, Serialize};
use serde_yaml::{self};
use std::time::{SystemTime};
use std::num;

#[derive(Debug, Serialize, Deserialize)]
pub struct Sensor
{
    hwmon_name:String,
    nice_name:String,
    sensor_name:String,
    n_sample_avg:i32
}

// Settings for Various RPM Interpolation Strategies:
#[derive(Serialize, Deserialize, Debug)]
struct QuadraticSettings
{
    A:f32,
    B:f32,
    C:f32,
    T_threshold:f32,
}

#[derive(Serialize, Deserialize, Debug)]
struct FeedforwardQuadraticSettings
{
    A_quad:f32,
    B_quad:f32,
    C_quad:f32,
    A_ff:f32,
    B_ff:f32
}

#[derive(Serialize, Deserialize, Debug)]
struct PIDSettings
{
    T_setpoint:f32,
    K_p:f32,
    K_i:f32,
    K_d:f32,
    windup_limit:f32,
}

#[derive(Serialize, Deserialize, Debug)]
#[serde(tag = "type")]
pub enum InterpolationSettings
{
    Quadratic(QuadraticSettings),
    FFQuad(FeedforwardQuadraticSettings),
    PID(PIDSettings)
}

#[derive(Debug, Serialize, Deserialize)]
pub struct Device
{
    hwmon_name:String,
    nice_name:String,
    max_temp:f32,
    temp_deadzone:i32,
    sensor_name:String,
    poll_mult:i32,
    interpolator:InterpolationSettings,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct Config
{
    loop_interval:f32,
    control_loop_ratio:i32,
    base_hwmon_path:String,
    charge_state_path:String,
    
    fan_hwmon_name:String,
    fan_hwmon_name_alt:String,
    fan_min_speed:i32,
    fan_threshold_speed:i32,
    fan_max_speed:i32,
    fan_gain:i32,
    ec_ramp_rate:i32,

    sensors:Vec<Sensor>,
    devices:Vec<Device>,
}

trait Interpolator
{
    fn evaluate(&mut self, x:f32, power:f32) -> f32;
}

struct Quadratic
{
    coeffs:QuadraticSettings,
    value:f32
}

struct PID
{
    settings:PIDSettings,
    // current_time:f64,
    previous_time:SystemTime,
    i_term:f32,
    last_error:f32,

    int_error:f32,
    windup_guard:f32,
    winddown_guard:f32
}

struct QuadraticFeedforward
{
    coeffs:FeedforwardQuadraticSettings,
    ff_deadzone:f32,
    ff_last_setpoint:f32,
    quad:Quadratic,
    value:f32
}



impl Interpolator for Quadratic
{
    fn evaluate(&mut self, mut x:f32, power:f32) -> f32
    {
        self.value = (self.coeffs.A * x * x) + self.coeffs.B * x + self.coeffs.C;
        return self.value;
    }
}

impl Interpolator for PID
{
    fn evaluate(&mut self, mut x:f32, power:f32) -> f32
    {
        let error = self.settings.T_setpoint - x;
        let current_time = SystemTime::now();

        let time_delta = current_time.duration_since(self.previous_time).expect("Time went backwards since last call.").as_secs_f32();
        self.previous_time = current_time;
        
        let error_delta = error - self.last_error;
        self.last_error = error;

        let p_term = self.settings.K_p * error;
        self.i_term += error * time_delta;

        self.i_term.clamp(-self.windup_guard, self.windup_guard);

        let mut d_term = 0.0;
        if(time_delta > 0.0)
        {
            d_term = error_delta / time_delta;
        }

        return -1.0 * (p_term + (self.settings.K_i * self.i_term) + (self.settings.K_d * d_term));
    }
}

impl QuadraticFeedforward
{
    fn get_ff_setpoint(&mut self, power:f32) -> f32
    {
        let rpm_setpoint = self.coeffs.A_ff * power * self.coeffs.B_ff;
        if((rpm_setpoint - self.ff_last_setpoint).abs() > self.ff_deadzone)
        {
            self.ff_last_setpoint = rpm_setpoint;
        }

        return self.ff_last_setpoint;
    }
}

impl Interpolator for QuadraticFeedforward
{
    fn evaluate(&mut self, mut x:f32, power:f32) -> f32
    {
        x = self.quad.evaluate(x, power);
        let ff_output = self.get_ff_setpoint(power);
        println!("FeedForward Controller - Quad:{:?}    FF:{:?}", x, ff_output);
        return x + ff_output;
    }
}

pub fn read_config(configpath:&str) -> Config
{
    let f = std::fs::File::open(configpath).expect("Could not open config file");
    let scrape_config:Config = serde_yaml::from_reader(f).expect("Could not deserialize config file");
    return scrape_config;
}
}

fn main() {
    println!("Config file: {:?}", fancontrol::read_config("/usr/share/jupiter-fan-control/jupiter-fan-control-config.yaml"));
}
