use nalgebra as na;

use crate::controller::DroneState;

#[derive(Clone, Copy, Default)]
pub struct SensorState {
    pub linear_acceleration_unfiltered: na::Vector3<f32>,
    pub angular_vel_unfiltered: na::Vector3<f32>,
    pub time: f32,
}


pub trait SensFusion {
    fn fuse(&mut self, sens_state: &SensorState) -> DroneState;
}

pub struct AngvelNoFiltering {

}

impl SensFusion for  AngvelNoFiltering{
    fn fuse(&mut self, sens_state: &SensorState) -> DroneState {
        return DroneState { angular_vel: sens_state.angular_vel_unfiltered, time: sens_state.time };
    }
}
