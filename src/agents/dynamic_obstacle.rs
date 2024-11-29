use rapier2d::prelude::*;

use crate::world;

pub struct DynamicObstacle {
    pub body: RigidBodyHandle,
    pub collider: ColliderHandle,
}

impl DynamicObstacle {
    pub fn new(state: &mut world::PhysicsState, pos: nalgebra::Vector2<f32>, size: f32) -> Self {
        let rigid_body = RigidBodyBuilder::dynamic()
            .translation(pos)
            .angular_damping(world::ANGULAR_DAMPING)
            .linear_damping(world::LINEAR_DAMPING)
            .build();

        let collider = ColliderBuilder::cuboid(size, size).restitution(0.7).build();
        let body_handle = state.bodies.insert(rigid_body);
        let collider_handle =
            state
                .colliders
                .insert_with_parent(collider, body_handle, &mut state.bodies);
        DynamicObstacle {
            body: body_handle,
            collider: collider_handle,
        }
    }
}
