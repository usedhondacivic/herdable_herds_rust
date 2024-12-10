use nalgebra::Vector2;
use rapier2d::prelude::*;

use crate::world;

use crate::agents::sheep::Sheep;

use std::cell::RefCell;

#[derive(Default)]
pub enum ShepherdMode {
    COLLECT {
        outlier_pos: Vector2<f32>,
    },
    #[default]
    DRIVE,
}

#[derive(Default)]
pub struct ShepherdState {
    pub centroid: Vector2<f32>,
    pub movement_target: Vector2<f32>,
    pub mode: ShepherdMode,
}

#[derive(Clone, Copy)]
pub struct ShepherdRules {
    pub chasing_force: f32,
    pub local_force: f32,
    pub collect_radius: f32,
    pub collect_lookahead: f32,
    pub drive_lookahead: f32,
}

pub struct Shepherd {
    pub body: RigidBodyHandle,
    body_collider: ColliderHandle,
    pub rules: ShepherdRules,
    pub state: RefCell<ShepherdState>,
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
            state: RefCell::new(ShepherdState::default()),
        }
    }

    pub fn update(
        &self,
        goal: &Vector2<f32>,
        phys_state: &mut world::PhysicsState,
        sheep: &[Sheep],
        _shepherds: &[Shepherd],
    ) {
        let mut state = self.state.borrow_mut();

        state.centroid = sheep
            .iter()
            .map(|s| {
                if let Some(sheep_body) = phys_state.bodies.get(s.body) {
                    return sheep_body.position().translation.vector;
                }
                Vector2::new(0.0, 0.0)
            })
            .sum();
        state.centroid = state.centroid.scale(1.0 / sheep.len() as f32);

        let mut max_distance = 0.0;
        sheep.iter().for_each(|s| {
            if let Some(sheep_body) = phys_state.bodies.get(s.body) {
                let dist = (sheep_body.position().translation.vector - state.centroid).magnitude();
                if dist > max_distance {
                    max_distance = dist;
                    if dist > self.rules.collect_radius {
                        state.mode = ShepherdMode::COLLECT {
                            outlier_pos: sheep_body.position().translation.vector,
                        }
                    }
                }
            }
        });

        if max_distance < self.rules.collect_radius {
            state.mode = ShepherdMode::DRIVE;
        }

        // Set target
        let own_body = phys_state.bodies.get_mut(self.body).unwrap();

        match state.mode {
            ShepherdMode::COLLECT { outlier_pos } => {
                let dir = outlier_pos - state.centroid;
                state.movement_target =
                    outlier_pos + dir.normalize() * self.rules.collect_lookahead;
            }
            ShepherdMode::DRIVE => {
                let dir = state.centroid - goal;
                state.movement_target =
                    state.centroid + dir.normalize() * self.rules.drive_lookahead;
            }
        };

        // Move towards target
        own_body.reset_forces(true);
        own_body.add_force(
            (state.movement_target - own_body.position().translation.vector)
                .normalize()
                .scale(self.rules.local_force),
            true,
        );
    }
}
