use std::{error::Error, fs::File};

use flight_control::controller::SampleController;
use nalgebra as na;

use crate::{input::InputRecording, logger::MsgPackSimLogger};

pub mod drone;
pub mod input;
pub mod logger;
mod renderer;
pub mod sim;
pub mod world;

use blue_engine::{prelude::Engine, primitive_shapes::cube, ObjectSettings};

fn main() -> Result<(), Box<dyn Error>> {
    let mut world = world::World::new(1.0 / 6000.0);

    // let mut inp_recording: InputRecording = Default::default();
    // for n in 0..10 {
    //     inp_recording.add_input(
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
        Box::new(SampleController::new()),
        flight_control::controller::MotorCharacteristics {
            relative_motor_positions: [
                na::point![1.0, -1.0, 1.0],
                na::point![-1.0, -1.0, -1.0],
                na::point![-1.0, 1.0, 1.0],
                na::point![1.0, 1.0, -1.0],
            ],
            max_thrust: 2.6,
            max_torque: 0.1,
            time_constant: 0.0,
        },
        0.5,
    );

    let mut sim = sim::Simulation::new(
        drone,
        world,
        100,
        Box::new(MsgPackSimLogger {
            file: File::create("./run-data/test.idfk")?,
        }),
        InputRecording::load_from_file("./run-data/rec.json")?,
    );

    let mut engine = Engine::new()?;
    let mut physics_renderer = renderer::PhysicsRenderer::new();

    physics_renderer.build_scene(&sim.world, &mut engine)?;

    cube(
        "ground",
        ObjectSettings {
            shader_settings: blue_engine::ShaderSettings {
                cull_mode: None,
                ..Default::default()
            },

            ..Default::default()
        },
        &mut engine.renderer,
        &mut engine.objects,
    )?;

    engine
        .objects
        .get_mut("ground")
        .unwrap()
        .set_scale((20.0, 20.0, 1.0))
        .set_color(0.5, 0.5, 1.0, 1.0)
        .set_texture(
            "background",
            blue_engine::TextureData::Path("assets/texture_08.png".to_string()),
            blue_engine::TextureMode::Repeat,
            &mut engine.renderer,
        )?;

    engine.update_loop(move |engine| {
        match sim.step().unwrap() {
            sim::StepOutcome::Continue => {}
            sim::StepOutcome::Exit => {
                sim.shutdown().unwrap();
            }
        }

        physics_renderer.sync(&sim.world, engine);

        let drone_pos = sim
            .world
            .bodies
            .get(sim.drone.rb_handle)
            .unwrap()
            .position();

        engine.camera.set_position(drone_pos.translation.vector);

        engine.camera.set_target(
            drone_pos.translation.vector
                + drone_pos
                    .rotation
                    .transform_vector(&na::vector![0.0, 1.0, 0.0]),
        );

        engine.camera.set_up(
            drone_pos
                .rotation
                .transform_vector(&na::vector![0.0, 0.0, 1.0]),
        );
        // engine.camera.set_up(pos);
    })?;

    Ok(())
}
