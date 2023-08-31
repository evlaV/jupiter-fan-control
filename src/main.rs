// Rust port of fancontrol.py.

mod fancontrol
{
use serde::{Deserialize, Serialize};
use serde_yaml::{self};

#[derive(Debug, Serialize, Deserialize)]
pub struct Sensor
{
    hwmon_name:String,
    nice_name:String,
    sensor_name:String,
    n_sample_avg:i32
}

// Various RPM Interpolation Strategies:
#[derive(Serialize, Deserialize, PartialEq, Debug)]
#[serde(tag = "type")]
pub enum Interpolator
{
    Quadratic 
    {
        A:f32,
        B:f32,
        C:f32,
        T_threshold:f32,
    },
    FFQuad
    {
        A_quad:f32,
        B_quad:f32,
        C_quad:f32,
        A_ff:f32,
        B_ff:f32
    },
    PID
    {
        T_setpoint:f32,
        K_p:f32,
        K_i:f32,
        K_d:f32,
        windup_limit:f32,
    },
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
    interpolator:Interpolator,
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
