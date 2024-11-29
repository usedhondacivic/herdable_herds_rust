use nalgebra::Vector2;
use rapier2d::prelude::*;

use crate::world;

use crate::agents::sheep::Sheep;

enum ShepherdState {
    DRIVE,
    COLLECT,
}

#[derive(Clone, Copy)]
pub struct ShepherdRules {
    pub chasing_force: f32,
    pub local_force: f32,
    pub collect_radius: f32,
}

pub struct Shepherd {
    pub body: RigidBodyHandle,
    body_collider: ColliderHandle,
    rules: ShepherdRules,
    state: ShepherdState,
}

impl Shepherd {
    pub fn new(
        state: &mut world::PhysicsState,
        pos: nalgebra::Vector2<f32>,
        vel: nalgebra::Vector2<f32>,
        rules: ShepherdRules,
    ) -> Self {
        let rigid_body = RigidBodyBuilder::dynamic()
            .translation(pos)
            .linvel(vel)
            .angular_damping(world::ANGULAR_DAMPING)
            .linear_damping(world::LINEAR_DAMPING)
            .build();

        let collider = ColliderBuilder::ball(world::AGENT_SIZE as f32)
            .restitution(0.7)
            .build();
        let body_handle = state.bodies.insert(rigid_body);
        let collider_handle =
            state
                .colliders
                .insert_with_parent(collider, body_handle, &mut state.bodies);
        Shepherd {
            body: body_handle,
            body_collider: collider_handle,
            rules,
            state: ShepherdState::DRIVE,
        }
    }

    pub fn update(&self, state: &mut world::PhysicsState, sheep: &[Sheep], shepherds: &[Shepherd]) {
        let mut sheep_centroid: Vector2<f32> = sheep
            .iter()
            .map(|s| {
                if let Some(sheep_body) = state.bodies.get(s.body) {
                    return sheep_body.position().translation.vector;
                }
                Vector2::new(0.0, 0.0)
            })
            .sum();
        sheep_centroid = sheep_centroid.scale(1.0 / sheep.len() as f32);

        let mut outlier: Option<Vector2<f32>> = None;
        let mut max_distance = 0.0;
        sheep.iter().for_each(|s| {
            if let Some(sheep_body) = state.bodies.get(s.body) {
                let dist = (sheep_body.position().translation.vector - sheep_centroid).magnitude();
                if dist > max_distance {
                    outlier = Some(sheep_body.position().translation.vector);
                    max_distance = dist;
                }
            }
        });

        let own_body = state.bodies.get_mut(self.body).unwrap();
        own_body.reset_forces(true);

        if max_distance > self.rules.collect_radius {
            own_body.add_force(
                (outlier.unwrap() - own_body.position().translation.vector)
                    .normalize()
                    .scale(self.rules.local_force),
                true,
            );
        } else {
            own_body.add_force(
                (sheep_centroid - own_body.position().translation.vector)
                    .normalize()
                    .scale(self.rules.local_force),
                true,
            );
        }
    }
}
