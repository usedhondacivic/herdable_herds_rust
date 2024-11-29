use nalgebra::Vector2;
use rapier2d::prelude::*;

use crate::world;

use crate::agents::shepherd::Shepherd;

#[derive(Clone, Copy)]
pub struct SheepRules {
    pub protected_radius: f32,
    pub visual_radius: f32,
    pub seperation_strength: f32,
    pub cohesion_strength: f32,
    pub alignment_strength: f32,
    pub predator_strength: f32,
    pub force_cap: f32,
}

pub struct Sheep {
    pub body: RigidBodyHandle,
    body_collider: ColliderHandle,
    rules: SheepRules,
}

impl Sheep {
    pub fn new(
        state: &mut world::PhysicsState,
        pos: nalgebra::Vector2<f32>,
        vel: nalgebra::Vector2<f32>,
        rules: SheepRules,
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
        Sheep {
            body: body_handle,
            body_collider: collider_handle,
            rules,
        }
    }

    pub fn update(&self, state: &mut world::PhysicsState, sheep: &[Sheep], shepherds: &[Shepherd]) {
        let own_pos = state.bodies.get(self.body).unwrap().position();
        let mut force = vector![0.0, 0.0];
        let mut neighbor_vel_sum = vector![0.0, 0.0];
        let mut num_neighbors = 0;

        sheep.iter().for_each(|s| {
            if s.body == self.body {
                return;
            }

            if let Some(other_body) = state.bodies.get(s.body) {
                // Separation
                let diff = own_pos.translation.vector - other_body.position().translation.vector;
                if diff.magnitude() != 0.0 && diff.magnitude() < self.rules.protected_radius {
                    force += diff.normalize()
                        * (1.0 / diff.magnitude())
                        * self.rules.seperation_strength;
                }

                // Cohession
                let diff = own_pos.translation.vector - other_body.position().translation.vector;
                if diff.magnitude() != 0.0 && diff.magnitude() < self.rules.visual_radius {
                    force -=
                        diff.normalize() * (1.0 / diff.magnitude()) * self.rules.cohesion_strength;
                }

                // Allignment
                let diff = own_pos.translation.vector - other_body.position().translation.vector;
                if diff.magnitude() != 0.0 && diff.magnitude() < self.rules.visual_radius {
                    num_neighbors += 1;
                    neighbor_vel_sum +=
                        other_body.linvel() / diff.magnitude() * self.rules.alignment_strength;
                }
            };
        });

        if neighbor_vel_sum.magnitude() != 0.0 && num_neighbors != 0 {
            let neighbor_vel_avg = neighbor_vel_sum / (num_neighbors as f32);
            force += neighbor_vel_avg;
        }

        shepherds.iter().for_each(|sh| {
            if let Some(shepherd_body) = state.bodies.get(sh.body) {
                // Run away from shepherds
                let diff = own_pos.translation.vector - shepherd_body.position().translation.vector;
                if diff.magnitude() != 0.0 && diff.magnitude() < self.rules.visual_radius {
                    force +=
                        diff.normalize() * (1.0 / diff.magnitude()) * self.rules.predator_strength;
                }
            }
        });

        force = force.cap_magnitude(50.0);

        let own_body = state.bodies.get_mut(self.body).unwrap();
        own_body.reset_forces(true);
        own_body.add_force(force, true);
    }
}
