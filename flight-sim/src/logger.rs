use std::{fs::File, io::BufWriter};

use nalgebra as na;

use crate::input::InputDef;
use flight_control::controller::ControllerLogRow;
use flight_control::controller::Input;

#[derive(Clone, Copy, serde::Serialize, serde::Deserialize)]
#[serde(remote = "ControllerLogRow")]
pub struct ControllerLogRowDef {
    time: f32,

    #[serde(with = "InputDef")]
    input: Input,

    sens_angular_velocty: na::Vector3<f32>,
    sens_rotation: na::Vector3<f32>,

    target_motors: na::Vector4<f32>,
    target_torque: na::Vector3<f32>,
    target_angular_velocty: na::Vector3<f32>,
    target_rotation: na::Vector3<f32>,
}

#[derive(Clone, Copy, serde::Serialize, serde::Deserialize)]
pub struct SimLogRow {
    time: f32,

    #[serde(with = "ControllerLogRowDef")]
    controller_log: ControllerLogRow,

    real_motors: na::Vector4<f32>,
    real_torque: na::Vector3<f32>,
    real_angular_velocty: na::Vector3<f32>,
    real_rotation: na::Vector3<f32>,
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
