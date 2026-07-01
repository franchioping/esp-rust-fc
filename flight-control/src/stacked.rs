use crate::controller::*;
use crate::mixer::MotorMixer;
use crate::pid::PidProcessor;

use nalgebra as na;

#[derive(Default, Clone, Copy)]
pub struct StackedControllerState {
    input: Input,
    last_time: f32,
    last_log_row: ControllerLogRow,
}

pub struct StackedController {
    motor_characteristics: MotorCharacteristics,

    rate_controller: PidProcessor,
    mixer: MotorMixer,

    state: StackedControllerState,
}

impl StackedController {
    pub fn new(rate_controller: PidProcessor, motor_mixer: MotorMixer) -> Self {
        return Self {
            motor_characteristics: Default::default(),
            rate_controller,
            mixer: motor_mixer,
            state: Default::default(),
        };
    }
}

impl DroneController for StackedController {
    fn as_any(&self) -> &dyn std::any::Any {
        self
    }
    fn as_mut_any(&mut self) -> &mut dyn std::any::Any {
        self
    }

    fn get_last_log_row(&self) -> Option<ControllerLogRow> {
        Some(self.state.last_log_row)
    }

    fn set_motor_characteristics(&mut self, motor_characteristics: &MotorCharacteristics) {
        self.motor_characteristics = *motor_characteristics;
    }

    fn set_input(&mut self, inp: &Input) {
        self.state.input = *inp;
    }

    fn update(&mut self, drone_state: &DroneState) -> [f32; 4] {
        let dt = drone_state.time - self.state.last_time;

        let setpoint = self.state.input.inp.xyz();

        let angular_accel = self
            .rate_controller
            .update(setpoint, drone_state.angular_vel, dt);

        let target_torque = angular_accel * self.motor_characteristics.mass;

        let motors = self.mixer.mix(
            0.5,
            target_torque.component_div(&self.motor_characteristics.max_torque()),
        );

        for val in motors {
            if val > 1.0 || val < -1.0 {
                dbg!(motors);
                dbg!(target_torque);
            }
        }

        self.state.last_log_row = ControllerLogRow {
            time: drone_state.time,
            input: self.state.input,
            target_motors: motors,
            target_torque: target_torque,
            target_angular_velocty: setpoint,
            target_angular_accel: angular_accel,
            ..Default::default()
        };

        self.state.last_time = drone_state.time;
        return motors;
    }
}
