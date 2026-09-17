use std::{error::Error, fs::File};

use flight_control::fusion::*;
use flight_control::{mixer::MotorMixer, pid::PidProcessor, stacked::StackedController};

use nalgebra as na;
use rand_distr::Normal;

use crate::drone::SensorCharacteristics;
use crate::sens::SensorErrorParams;
use crate::{input::InputRecording, logger::MsgPackSimLogger};

pub mod drone;
pub mod input;
pub mod logger;
pub mod sens;
pub mod sim;
pub mod world;

fn main() -> Result<(), Box<dyn Error>> {
    let mut world = world::World::new(1.0 / 10000.0);

    // let mut inp_recording: InputRecording = Default::default(); for n in 0..10 { inp_recording.add_input(
    //         n as f32 * 0.2,
    //         flight_control::controller::Input {
    //             mode: flight_control::controller::InputMode::ACRO,
    //             inp: na::vector![n as f32, n as f32 * 2.0, n as f32 / 2.0, 1.0],
    //         },
    //     );
    // }
    // inp_recording.save_to_file("./run-data/rec.json")?;

    let drone = drone::Drone::new(
        &mut world,
        Box::new(StackedController::new(
            PidProcessor {
                kp: na::vector![2.0, 2.0, 2.0],
                ki: na::vector![0.1, 0.1, 0.1],
                kd: na::vector![0.0, 0.0, 0.0],
                kff: na::vector![0.0, 0.0, 0.0],
                last_error: na::vector![0.0, 0.0, 0.0],
                integral: na::vector![0.0, 0.0, 0.0],
            },
            MotorMixer {
                motor_map: [
                    [1.0, -1.0, 1.0],
                    [-1.0, -1.0, -1.0],
                    [-1.0, 1.0, 1.0],
                    [1.0, 1.0, -1.0],
                ],
                min_throttle: 0.0,
                max_throttle: 1.0,
                mixing_mode: Default::default(),
            },
        )),
        Box::new(AngvelNoFiltering {}),
        flight_control::controller::MotorCharacteristics {
            relative_motor_positions: [
                na::point![5.0, 5.0, 0.0],
                na::point![5.0, -5.0, 0.0],
                na::point![-5.0, -5.0, 0.0],
                na::point![-5.0, 5.0, 0.0],
            ],
            max_thrust: 2.6,
            max_torque: 0.1,
            time_constant: 0.0,
            mass: 0.5,
            resonance_throttle_coeff: 20000.0 / 60.0 * 3.0, // approximately RPM / 60.0 * prop_count
        },
        SensorCharacteristics {
            gyro_error_params: SensorErrorParams {
                bias: na::Vector3::<f32>::zeros(),
                bias_drift_rate: na::Vector3::<f32>::zeros(),
                scale_factors: na::vector![1.0, 1.0, 1.0],
                cross_talk: na::Matrix3::zeros(),
                random_noise_distrib: Normal::new(0.0, 0.0).unwrap(),
                resolution: 0.0005,
                ..Default::default()
            },

            accel_error_params: Default::default(),
        },
    );

    let mut sim = sim::Simulation::new(
        drone,
        world,
        10000,
        Box::new(MsgPackSimLogger {
            file: File::create("./run-data/test.idfk")?,
        }),
        InputRecording::load_from_file("./run-data/rec.json")?,
    );

    sim.run()?;

    Ok(())
}
