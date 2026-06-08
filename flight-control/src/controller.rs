use core::any::Any;

use nalgebra as na;

pub enum InputMode {
    ACRO,
    LEVEL,
}

pub struct Input {
    pub mode: InputMode,
    pub inp: na::Vector4<f32>,
}

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

pub trait DroneController {
    // Allow downcast of trait -> class.
    //
    // This gives us the ability to have a Box<dyn DroneController>
    // And transform it into a pointer to its class.
    fn as_any(&self) -> &dyn Any;
    fn as_mut_any(&mut self) -> &mut dyn Any;

    fn set_motor_characteristics(&mut self, _motor_characteristics: &MotorCharacteristics) {}

    fn set_input(&mut self, inp: &Input) {}
    fn update(&mut self, state: &DroneState) -> [f32; 4];
}
