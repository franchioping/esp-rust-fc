use crate::world::World;
use std::collections::HashMap;

use blue_engine::{prelude::Engine, primitive_shapes::cube, ObjectSettings, Vertex};

use nalgebra as na;
use rapier3d::prelude as rp;

pub struct PhysicsRenderer {
    collider_objects: HashMap<rp::ColliderHandle, String>,
}

impl PhysicsRenderer {
    pub fn new() -> Self {
        Self {
            collider_objects: HashMap::new(),
        }
    }

    pub fn build_scene(&mut self, world: &World, engine: &mut Engine) -> anyhow::Result<()> {
        for (handle, collider) in world.colliders.iter() {
            let name = format!("collider_{:?}", handle);

            match collider.shape().as_typed_shape() {
                rp::TypedShape::Cuboid(cuboid) => {
                    cube(
                        &name,
                        ObjectSettings {
                            ..Default::default()
                        },
                        &mut engine.renderer,
                        &mut engine.objects,
                    )?;

                    engine.objects.get_mut(name.as_str()).unwrap().set_scale((
                        cuboid.half_extents.x * 2.0,
                        cuboid.half_extents.y * 2.0,
                        cuboid.half_extents.z * 2.0,
                    ));
                }

                _ => continue,
            }

            self.collider_objects.insert(handle, name);
        }

        Ok(())
    }

    pub fn sync(&self, world: &World, engine: &mut Engine) {
        for (handle, name) in &self.collider_objects {
            let Some(collider) = world.colliders.get(*handle) else {
                continue;
            };

            let pos = collider.position();

            if let Some(obj) = engine.objects.get_mut(name.as_str()) {
                obj.set_position((pos.translation.x, pos.translation.y, pos.translation.z));

                let (rx, ry, rz) = pos.rotation.euler_angles();

                obj.set_rotation((rx, ry, rz));
            }
        }
    }
}
