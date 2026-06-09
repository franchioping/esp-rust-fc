use std::error::Error;

use nalgebra as na;

use crate::drone::Drone;
use crate::world::World;

pub struct Simulation {
    pub drone: Drone,
    pub world: World,

    drone_tick_rate: u64,
}

enum StepOutcome {
    Continue,
    Exit,
}

impl Simulation {
    pub fn new(drone: Drone, world: World, drone_tick_rate: u64) -> Self {
        return Self {
            drone,
            world,
            drone_tick_rate,
        };
    }

    pub fn shutdown(&self) -> Result<(), Box<dyn Error>> {
        Ok(())
    }

    fn step(&mut self) -> Result<StepOutcome, Box<dyn Error>> {
        self.world.step();
        if self.world.tick
            % ((self.world.integration_parameters.inv_dt() / self.drone_tick_rate as f32) as u64)
            == 0
        {
            self.drone.process_controller_tick(&mut self.world);
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
