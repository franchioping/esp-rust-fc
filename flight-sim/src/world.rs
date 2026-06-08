use std::collections::HashMap;

use nalgebra as na;
use rapier3d::prelude::{self as rp, ColliderHandle};

pub struct World {
    physics_pipeline: rp::PhysicsPipeline,
    gravity: na::Vector3<f32>,
    pub integration_parameters: rp::IntegrationParameters,
    island_manager: rp::IslandManager,
    broad_phase: rp::BroadPhaseBvh,
    narrow_phase: rp::NarrowPhase,
    impulse_joint_set: rp::ImpulseJointSet,
    multibody_joint_set: rp::MultibodyJointSet,
    ccd_solver: rp::CCDSolver,
    hooks: Box<dyn rp::PhysicsHooks>,
    events: Box<dyn rp::EventHandler>,

    pub bodies: rp::RigidBodySet,
    pub colliders: rp::ColliderSet,
    pub tick: u64,
}

impl World {
    pub fn new(dt: f32) -> Self {
        let gravity: na::Vector3<f32> = rp::vector![0.0, 0.0, -9.81];
        let mut integration_parameters = rp::IntegrationParameters::default();
        integration_parameters.set_inv_dt(dt);
        let physics_pipeline = rp::PhysicsPipeline::new();
        let island_manager = rp::IslandManager::new();
        let broad_phase = rp::DefaultBroadPhase::new();
        let narrow_phase = rp::NarrowPhase::new();
        let impulse_joint_set = rp::ImpulseJointSet::new();
        let multibody_joint_set = rp::MultibodyJointSet::new();
        let ccd_solver = rp::CCDSolver::new();
        let physics_hooks = ();
        let event_handler = ();
        return Self {
            physics_pipeline,
            gravity: gravity,
            integration_parameters,
            island_manager,
            broad_phase,
            narrow_phase,
            impulse_joint_set,
            multibody_joint_set,
            ccd_solver,
            hooks: Box::new(physics_hooks),
            events: Box::new(event_handler),
            bodies: rp::RigidBodySet::new(),
            colliders: rp::ColliderSet::new(),
            tick: 0,
        };
    }

    pub fn step(&mut self) {
        self.physics_pipeline.step(
            &self.gravity,
            &self.integration_parameters,
            &mut self.island_manager,
            &mut self.broad_phase,
            &mut self.narrow_phase,
            &mut self.bodies,
            &mut self.colliders,
            &mut self.impulse_joint_set,
            &mut self.multibody_joint_set,
            &mut self.ccd_solver,
            self.hooks.as_ref(),
            self.events.as_ref(),
        );
        self.tick += 1;
    }

    pub fn get_time(&self) -> f32 {
        return self.integration_parameters.dt * (self.tick as f32);
    }

    #[allow(dead_code)]
    pub fn clear_ofb(&mut self) {
        let mut coll_to_del: Vec<ColliderHandle> = Vec::new();
        for (handle, col) in self.colliders.iter() {
            if col.translation().y < -30.0 {
                coll_to_del.push(handle);
            }
        }
        for handle in coll_to_del {
            self.colliders
                .remove(handle, &mut self.island_manager, &mut self.bodies, false);
        }
    }

    pub fn position_of_collider(
        &self,
        collider_handle: rp::ColliderHandle,
    ) -> Option<na::Vector3<f32>> {
        let coll = self.colliders.get(collider_handle)?;
        return Some(*coll.translation());
    }

    pub fn register_body(&mut self, rb: rp::RigidBody) -> rp::RigidBodyHandle {
        return self.bodies.insert(rb);
    }

    pub fn register_collider(
        &mut self,
        collider: rp::Collider,
        rigid_body_handle: rp::RigidBodyHandle,
    ) -> rp::ColliderHandle {
        let handle =
            self.colliders
                .insert_with_parent(collider, rigid_body_handle, &mut self.bodies);
        return handle;
    }

    pub fn register_free_collider(&mut self, collider: rp::Collider) -> rp::ColliderHandle {
        let handle = self.colliders.insert(collider);
        return handle;
    }
}

impl Default for World {
    fn default() -> Self {
        let gravity: na::Vector3<f32> = rp::vector![0.0, 0.0, -9.81];
        let mut integration_parameters = rp::IntegrationParameters::default();
        integration_parameters.set_inv_dt(600.0);
        let physics_pipeline = rp::PhysicsPipeline::new();
        let island_manager = rp::IslandManager::new();
        let broad_phase = rp::DefaultBroadPhase::new();
        let narrow_phase = rp::NarrowPhase::new();
        let impulse_joint_set = rp::ImpulseJointSet::new();
        let multibody_joint_set = rp::MultibodyJointSet::new();
        let ccd_solver = rp::CCDSolver::new();
        let physics_hooks = ();
        let event_handler = ();
        return Self {
            physics_pipeline,
            gravity: gravity,
            integration_parameters,
            island_manager,
            broad_phase,
            narrow_phase,
            impulse_joint_set,
            multibody_joint_set,
            ccd_solver,
            hooks: Box::new(physics_hooks),
            events: Box::new(event_handler),
            bodies: rp::RigidBodySet::new(),
            colliders: rp::ColliderSet::new(),
            tick: 0,
        };
    }
}
