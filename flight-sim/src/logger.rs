use std::{fs::File, io::BufWriter};

use nalgebra as na;

use crate::input::InputDef;
use flight_control::controller::Input;

#[derive(Clone, Copy, serde::Serialize, serde::Deserialize)]
pub struct SimLogRow {
    time: f32,

    #[serde(with = "InputDef")]
    input: Input,

    motors: na::Vector4<f32>,
    torque: na::Vector3<f32>,
    angular_velocty: na::Vector3<f32>,
    rotation: na::Vector3<f32>,

    target_motors: na::Vector4<f32>,
    target_torque: na::Vector3<f32>,
    target_angular_velocty: na::Vector3<f32>,
    target_rotation: na::Vector3<f32>,
}

pub struct MsgPackSimLogger {
    file: File,
}

impl SimLogger for MsgPackSimLogger {
    fn write_sim_data(
        &self,
        simulation_data: &Vec<SimLogRow>,
    ) -> Result<(), Box<dyn std::error::Error>> {
        rmp_serde::encode::write(&mut BufWriter::new(&self.file), simulation_data)?;
        Ok(())
    }
}

pub trait SimLogger {
    fn write_sim_data(
        &self,
        simulation_data: &Vec<SimLogRow>,
    ) -> Result<(), Box<dyn std::error::Error>>;
}
