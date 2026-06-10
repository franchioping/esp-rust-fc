use core::any::Any;

use nalgebra as na;

#[derive(Clone, Copy, PartialEq, Eq, Default)]
pub enum InputMode {
    #[default]
    ACRO,
    LEVEL,
}

#[derive(Clone, Copy, PartialEq, Default)]
pub struct Input {
    pub mode: InputMode,
    pub inp: na::Vector4<f32>,
}

#[derive(Clone, Copy)]
pub struct DroneState {
    pub rotation: na::Unit<na::Quaternion<f32>>,
    pub angular_vel: na::Vector3<f32>,
    pub time: f32,
}

pub struct MotorCharacteristics {
    pub relative_motor_positions: [na::OPoint<f32, na::Const<3>>; 4],
    pub max_thrust: f32,
    pub max_torque: f32,
    pub time_constant: f32,
}

#[derive(Clone, Copy, Default)]
pub struct ControllerLogRow {
    pub time: f32,

    pub input: Input,

    pub sens_angular_velocty: na::Vector3<f32>,
    pub sens_rotation: na::Vector3<f32>,

    pub target_motors: [f32; 4],
    pub target_torque: na::Vector3<f32>,
    pub target_angular_velocty: na::Vector3<f32>,
    pub target_rotation: na::Vector3<f32>,
}

pub trait DroneController {
    // Allow downcast of trait -> class.
    //
    // This gives us the ability to have a Box<dyn DroneController>
    // And transform it into a pointer to its class.
    fn as_any(&self) -> &dyn Any;
    fn as_mut_any(&mut self) -> &mut dyn Any;

    fn set_motor_characteristics(&mut self, _motor_characteristics: &MotorCharacteristics) {}

    fn set_input(&mut self, _inp: &Input) {}
    fn update(&mut self, state: &DroneState) -> [f32; 4];

    fn get_last_log_row(&self) -> Option<ControllerLogRow>;
}

pub struct SampleController {
    log_row: ControllerLogRow,
}

impl SampleController {
    pub fn new() -> Self {
        Self {
            log_row: Default::default(),
        }
    }
}

impl DroneController for SampleController {
    fn as_any(&self) -> &dyn Any {
        self
    }
    fn as_mut_any(&mut self) -> &mut dyn Any {
        self
    }
    fn update(&mut self, state: &DroneState) -> [f32; 4] {
        let target: [f32; 4] = [
            rand::random_range(-1.0..1.0),
            rand::random_range(-1.0..1.0),
            rand::random_range(-1.0..1.0),
            rand::random_range(-1.0..1.0),
        ];
        self.log_row = ControllerLogRow {
            time: state.time,
            target_motors: target,
            ..Default::default()
        };

        return target;
    }
    fn get_last_log_row(&self) -> Option<ControllerLogRow> {
        Some(self.log_row)
    }
}
