use flight_control::controller::*;
use nalgebra::{self as na, Vector3};
use rapier3d::prelude as rp;

use crate::world::World;

const AIR_DENSITY: f32 = 1.23;
const DRAG_CONSTANT: f32 = 1.3;
const DRAG_MAGIC_NUM: f32 = 0.00;

pub struct Drone {
    pub rb_handle: rp::RigidBodyHandle,
    pub controller: Box<dyn DroneController>,
    pub current_throttles: [f32; 4],
    pub last_torque: na::Vector3<f32>,

    motor_characteristics: MotorCharacteristics,
    width: f32,
    height: f32,

    target_throttles: [f32; 4],
    last_time: f32,
    linvel: na::Vector3<f32>,
    accel: na::Vector3<f32>,
}

fn calculate_drag(velocity: f32, area: f32, constant: f32) -> f32 {
    return AIR_DENSITY * (velocity.abs() * velocity) * area * constant * DRAG_MAGIC_NUM;
}

impl Drone {
    pub fn new(
        world: &mut World,
        mut controller: Box<dyn DroneController>,
        motor_characteristics: MotorCharacteristics,
        mass: f32,
    ) -> Drone {
        let drone_rb_handle = world.register_body(
            rp::RigidBodyBuilder::dynamic()
                .translation(rp::vector![0.0, 0.0, 10.0])
                .rotation(rp::vector![0.0, 0.0, 0.0])
                /*
                 * These damping values keep the simulation more realistic,
                 * They act as air resistance
                 *
                 * Values are kind of random for now. Calculating them requires the final model
                 * A Poor Man's fluid simulation :D
                 */
                // .linear_damping(0.2) // Damps velocity slowly
                .angular_damping(0.1) // Damps angular velocity slowly
                .gyroscopic_forces_enabled(false)
                .build(),
        );
        let width = 0.40;
        let height = 0.25;
        world.register_collider(
            rp::ColliderBuilder::cuboid(width / 2.0, height / 2.0, width / 2.0)
                .mass(mass)
                .restitution(0.3)
                .build(),
            drone_rb_handle,
        );

        // Should only need to be called once?
        // No one should be changing motor characteristics during flight time.
        controller.set_motor_characteristics(&motor_characteristics);

        return Drone {
            rb_handle: drone_rb_handle,
            controller: controller,
            motor_characteristics: motor_characteristics,
            width,
            height,
            current_throttles: [0.0; 4],
            target_throttles: [0.0; 4],
            last_time: world.get_time(),
            linvel: na::Vector3::zeros(),
            accel: na::Vector3::zeros(),
            last_torque: na::Vector3::zeros(),
        };
    }

    fn apply_drag(&mut self, world: &mut World) {
        let body = world.bodies.get_mut(self.rb_handle).unwrap();
        let velocity = body.linvel();
        let mut drag = Vector3::<f32>::zeros();

        let side_area = self.width * self.height;
        let up_area = self.width * self.width;

        drag.x = -calculate_drag(velocity.x, side_area, DRAG_CONSTANT);
        drag.y = -calculate_drag(velocity.y, up_area, DRAG_CONSTANT);
        drag.z = -calculate_drag(velocity.z, side_area, DRAG_CONSTANT);

        body.add_force(drag, true);
    }

    fn apply_throttles(&mut self, world: &mut World, dt: f32) {
        let throttles = self.target_throttles;

        let drone_rb = world.bodies.get_mut(self.rb_handle).unwrap();

        /*
         * Clear all the previously applied forces and torques, or theyll add-up every tick
         */
        drone_rb.reset_forces(true);
        drone_rb.reset_torques(true);
        for (i, motor_position) in self
            .motor_characteristics
            .relative_motor_positions
            .iter()
            .enumerate()
        {
            let target_throttle = throttles[i].clamp(0.0, 1.0);

            let alpha = 1.0 - (-dt / self.motor_characteristics.time_constant).exp(); // Exp

            self.current_throttles[i] += (target_throttle - self.current_throttles[i]) * alpha;

            let throttle = self.current_throttles[i];

            let thrust = self.motor_characteristics.max_thrust * throttle;
            let torque = self.motor_characteristics.max_torque * throttle;

            // Thrust is applied upward at motor position.
            //
            // Force calculated with (0.0, thrust, 0.0) points to World Up,
            // Apply RB's rotation to point at RB's Up
            //
            // motor_position is relative, transform it by the RB's position first
            let mut thrust_force: na::Vector3<f32> = nalgebra::Vector3::new(0.0, 0.0, thrust);
            thrust_force = drone_rb.rotation().transform_vector(&thrust_force);
            drone_rb.add_force_at_point(
                thrust_force,
                drone_rb.position().transform_point(motor_position),
                true,
            );

            let torque = if i % 2 == 0 {
                rp::vector![0.0, 0.0, torque]
            } else {
                rp::vector![0.0, 0.0, -torque]
            };
            self.last_torque = torque;
            drone_rb.add_torque(drone_rb.rotation().transform_vector(&torque), true);
        }
    }

    fn update_controller(&mut self, world: &World) {
        let rb = world.bodies.get(self.rb_handle).unwrap();

        self.target_throttles = self.controller.update(&DroneState {
            rotation: *rb.rotation(),
            angular_vel: rb.rotation().inverse().transform_vector(&rb.angvel()),
            time: world.get_time(),
        });
    }

    pub fn get_accel(&self) -> na::Vector3<f32> {
        self.accel
    }

    pub fn get_angvel(&self, world: &World) -> na::Vector3<f32> {
        *world.bodies.get(self.rb_handle).unwrap().angvel()
    }

    pub fn get_rb<'a>(&self, world: &'a World) -> &'a rp::RigidBody {
        world.bodies.get(self.rb_handle).unwrap()
    }
    pub fn get_rot(&self, world: &World) -> na::Unit<na::Quaternion<f32>> {
        *world.bodies.get(self.rb_handle).unwrap().rotation()
    }

    pub fn process_controller_tick(&mut self, world: &mut World, inp: &Input) {
        self.controller.set_input(inp);
        self.update_controller(world);
    }

    pub fn process_tick(&mut self, world: &mut World) {
        self.apply_throttles(world, world.get_time() - self.last_time);
        self.apply_drag(world);

        let cur_linvel = self.get_rb(world).linvel();
        self.accel = (cur_linvel - self.linvel) / (world.get_time() - self.last_time);
        self.linvel = *cur_linvel;
        self.last_time = world.get_time();
    }
}
