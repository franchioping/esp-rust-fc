use core::any::Any;

use nalgebra as na;

pub struct DroneState {
    rotation: na::Unit<na::Quaternion<f32>>,
    angular_vel: na::Vector3<f32>,
    linear_vel: na::Vector3<f32>,
    position: na::Vector3<f32>,
    time: f32,
}

pub trait DroneController {
    // Allow downcast of trait -> class.
    //
    // This gives us the ability to have a Box<dyn DroneController>
    // And transform it into a pointer to its class.
    fn as_any(&self) -> &dyn Any;
    fn as_mut_any(&mut self) -> &mut dyn Any;

    fn set_motor_characteristics(&mut self, _motor_characteristics: &MotorCharacteristics) {}
    fn update(&mut self, state: &DroneState) -> [f32; 4];
}

pub struct MotorCharacteristics {
    pub relative_motor_positions: [na::OPoint<f32, na::Const<3>>; 4],
    pub max_thrust: f32,
    pub max_torque: f32,
    pub time_constant: f32,
}

impl Default for MotorCharacteristics {
    fn default() -> Self {
        Self {
            /*
             * Motor position indices
             *    ^ - Front
             *    |
             *    |
             * 1 --- 0
             * |     |
             * |     |
             * 2 --- 3
             */
            relative_motor_positions: [
                na::point![5.0, 5.0, 0.0],
                na::point![5.0, -5.0, 0.0],
                na::point![-5.0, -5.0, 0.0],
                na::point![-5.0, 5.0, 0.0],
            ],
            max_thrust: 2.6,
            max_torque: 0.5,
            time_constant: 0.01,
        }
    }
}
