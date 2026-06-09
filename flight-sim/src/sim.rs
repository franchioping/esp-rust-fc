use std::error::Error;

use flight_control::controller::Input;
use nalgebra as na;

use crate::drone::Drone;
use crate::input::InputRecording;
use crate::logger::{SimLogRow, SimLogger};
use crate::world::World;

pub struct Simulation {
    pub drone: Drone,
    pub world: World,

    input_recording: InputRecording,
    logger: Box<dyn SimLogger>,
    drone_tick_rate: u64,

    sim_data: Vec<SimLogRow>,
}

enum StepOutcome {
    Continue,
    Exit,
}

impl Simulation {
    pub fn new(
        drone: Drone,
        world: World,
        drone_tick_rate: u64,
        logger: Box<dyn SimLogger>,
        input_recording: InputRecording,
    ) -> Self {
        return Self {
            drone,
            world,
            input_recording,
            logger,
            drone_tick_rate,
            sim_data: Default::default(),
        };
    }

    pub fn shutdown(&self) -> Result<(), Box<dyn Error>> {
        self.logger.write_sim_data(&self.sim_data)?;
        Ok(())
    }

    fn step(&mut self) -> Result<StepOutcome, Box<dyn Error>> {
        let step_time = self.world.get_time();
        if self.input_recording.has_ended(step_time) {
            return Ok(StepOutcome::Exit);
        }

        self.world.step();

        let current_input = self.input_recording.get_input(self.world.get_time());
        if self.world.tick
            % ((self.world.integration_parameters.inv_dt() / self.drone_tick_rate as f32) as u64)
            == 0
        {
            self.drone
                .process_controller_tick(&mut self.world, &current_input);
        }
        self.drone.process_tick(&mut self.world);

        Ok(StepOutcome::Continue)
    }

    pub fn run(&mut self) -> Result<(), Box<dyn Error>> {
        loop {
            match self.step()? {
                StepOutcome::Continue => {}
                StepOutcome::Exit => {
                    self.shutdown()?;
                    return Ok(());
                }
            }
        }
    }
}
