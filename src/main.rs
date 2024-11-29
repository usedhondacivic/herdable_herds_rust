use std::f32::consts::PI;

use macroquad::prelude as mq;
use macroquad::prelude::glam;
use nalgebra::Vector2;
use rapier2d::prelude::*;

const PIXELS_PER_METER: i32 = 4;
const WINDOW_SIZE_PX: (i32, i32) = (750, 750);
const WINDOW_SIZE_METERS: (i32, i32) = (
    WINDOW_SIZE_PX.0 / PIXELS_PER_METER,
    WINDOW_SIZE_PX.1 / PIXELS_PER_METER,
);

const AGENT_SIZE: f32 = 0.5;

const SHEEP_PROTECTED_RADIUS: f32 = 5.;
const SHEEP_VISUAL_RADIUS: f32 = 25.;

const LINEAR_DAMPING: f32 = 2.0;
const ANGULAR_DAMPING: f32 = 2.0;

const DRAW_DEBUG: bool = true;

struct PhysicsState {
    pub islands: IslandManager,
    pub broad_phase: DefaultBroadPhase,
    pub narrow_phase: NarrowPhase,
    pub bodies: RigidBodySet,
    pub colliders: ColliderSet,
    pub impulse_joints: ImpulseJointSet,
    pub multibody_joints: MultibodyJointSet,
    pub ccd_solver: CCDSolver,
    pub query_pipeline: QueryPipeline,
    pub integration_parameters: IntegrationParameters,
    pub gravity: Vector<f32>,
    pub physics_hooks: (),
    pub event_handler: (),
}

struct World {
    physics_state: PhysicsState,
    physics_pipeline: PhysicsPipeline,

    sheep: Vec<Sheep>,
    shepherds: Vec<Shepherd>,
    dynamic_obstacle: Vec<DynamicObstacle>,
}

impl World {
    fn new() -> Self {
        let mut new_world = World {
            sheep: Vec::new(),
            shepherds: Vec::new(),
            dynamic_obstacle: Vec::new(),
            physics_state: PhysicsState {
                islands: IslandManager::new(),
                broad_phase: DefaultBroadPhase::new(),
                narrow_phase: NarrowPhase::new(),
                bodies: RigidBodySet::new(),
                colliders: ColliderSet::new(),
                impulse_joints: ImpulseJointSet::new(),
                multibody_joints: MultibodyJointSet::new(),
                ccd_solver: CCDSolver::new(),
                query_pipeline: QueryPipeline::new(),
                integration_parameters: IntegrationParameters::default(),
                gravity: vector![0.0, 0.0],
                physics_hooks: (),
                event_handler: (),
            },
            physics_pipeline: PhysicsPipeline::new(),
        };

        let boid_rules = BoidRules {
            protected_radius: SHEEP_PROTECTED_RADIUS,
            visual_radius: SHEEP_VISUAL_RADIUS,
            seperation_strength: 25.0,
            cohesion_strength: 5.0,
            alignment_strength: 5.0,
            predator_strength: 100.0,
            force_cap: 50.0,
        };

        (1..10).for_each(|x| {
            (1..10).for_each(|y| {
                new_world.sheep.push(Sheep::new(
                    &mut new_world.physics_state,
                    vector![x as f32 * 2.0 + 20.0, y as f32 * 2.0 + 100.0],
                    vector![0.0, 0.0],
                    boid_rules,
                ))
            });
        });

        let shepherd_rules = ShepherdRules {
            chasing_force: 20.0,
            local_force: 20.0,
            collect_radius: 15.0,
        };

        new_world.shepherds.push(Shepherd::new(
            &mut new_world.physics_state,
            vector![100.0, 100.0],
            vector![0.0, 0.0],
            shepherd_rules,
        ));

        (1..5).for_each(|x| {
            (1..10).for_each(|y| {
                new_world.dynamic_obstacle.push(DynamicObstacle::new(
                    &mut new_world.physics_state,
                    vector![100.0 + x as f32 * 15.0, y as f32 * 15.0],
                    5.0,
                ));
            });
        });

        new_world
    }

    fn draw(&self) {
        self.sheep.iter().for_each(|s| s.draw(&self.physics_state));
        self.shepherds
            .iter()
            .for_each(|s| s.draw(&self.physics_state));
        self.dynamic_obstacle
            .iter()
            .for_each(|s| s.draw(&self.physics_state));
    }

    fn update(&mut self) {
        self.physics_pipeline.step(
            &self.physics_state.gravity,
            &self.physics_state.integration_parameters,
            &mut self.physics_state.islands,
            &mut self.physics_state.broad_phase,
            &mut self.physics_state.narrow_phase,
            &mut self.physics_state.bodies,
            &mut self.physics_state.colliders,
            &mut self.physics_state.impulse_joints,
            &mut self.physics_state.multibody_joints,
            &mut self.physics_state.ccd_solver,
            Some(&mut self.physics_state.query_pipeline),
            &self.physics_state.physics_hooks,
            &self.physics_state.event_handler,
        );

        self.sheep
            .iter()
            .for_each(|s| s.update(&mut self.physics_state, &self.sheep, &self.shepherds));

        self.shepherds
            .iter()
            .for_each(|s| s.update(&mut self.physics_state, &self.sheep, &self.shepherds));
    }
}

#[derive(Clone, Copy)]
struct BoidRules {
    protected_radius: f32,
    visual_radius: f32,
    seperation_strength: f32,
    cohesion_strength: f32,
    alignment_strength: f32,
    predator_strength: f32,
    force_cap: f32,
}

struct Sheep {
    body: RigidBodyHandle,
    body_collider: ColliderHandle,
    rules: BoidRules,
}

impl Sheep {
    fn new(
        state: &mut PhysicsState,
        pos: nalgebra::Vector2<f32>,
        vel: nalgebra::Vector2<f32>,
        rules: BoidRules,
    ) -> Self {
        let rigid_body = RigidBodyBuilder::dynamic()
            .translation(pos)
            .linvel(vel)
            .angular_damping(ANGULAR_DAMPING)
            .linear_damping(LINEAR_DAMPING)
            .build();

        let collider = ColliderBuilder::ball(AGENT_SIZE as f32)
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

    fn draw(&self, state: &PhysicsState) {
        let pos = state.bodies[self.body].position().translation.vector;
        let rot = state.bodies[self.body]
            .linvel()
            .y
            .atan2(state.bodies[self.body].linvel().x);
        draw_agent(pos, rot, mq::BLUE);
    }

    fn update(&self, state: &mut PhysicsState, sheep: &[Sheep], shepherds: &[Shepherd]) {
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

        // let (mouse_x, mouse_y) = mq::mouse_position();
        // let mouse_pos = vector![
        //     mouse_x / (PIXELS_PER_METER as f32),
        //     mouse_y / (PIXELS_PER_METER as f32)
        // ];
        //
        // let diff = own_pos.translation.vector - mouse_pos;
        // if diff.magnitude() != 0.0 {
        //     force += diff.normalize() * (1.0 / diff.magnitude()) * 500.0;
        // }

        // Keep on screen
        if own_pos.translation.x > (WINDOW_SIZE_METERS.0 - 10) as f32 {
            force.x -= 6.0;
        }
        if own_pos.translation.x < 10.0 {
            force.x += 6.0;
        }
        if own_pos.translation.y > (WINDOW_SIZE_METERS.1 - 10) as f32 {
            force.y -= 6.0;
        }
        if own_pos.translation.y < 10.0 {
            force.y += 6.0;
        }

        let own_body = state.bodies.get_mut(self.body).unwrap();
        own_body.reset_forces(true);
        own_body.add_force(force, true);
    }
}

enum ShepherdState {
    DRIVE,
    COLLECT,
}

#[derive(Clone, Copy)]
struct ShepherdRules {
    chasing_force: f32,
    local_force: f32,
    collect_radius: f32,
}

struct Shepherd {
    body: RigidBodyHandle,
    body_collider: ColliderHandle,
    rules: ShepherdRules,
    state: ShepherdState,
}

impl Shepherd {
    fn new(
        state: &mut PhysicsState,
        pos: nalgebra::Vector2<f32>,
        vel: nalgebra::Vector2<f32>,
        rules: ShepherdRules,
    ) -> Self {
        let rigid_body = RigidBodyBuilder::dynamic()
            .translation(pos)
            .linvel(vel)
            .angular_damping(ANGULAR_DAMPING)
            .linear_damping(LINEAR_DAMPING)
            .build();

        let collider = ColliderBuilder::ball(AGENT_SIZE as f32)
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

    fn draw(&self, state: &PhysicsState) {
        let pos = state.bodies[self.body].position().translation.vector;
        let rot = state.bodies[self.body]
            .linvel()
            .y
            .atan2(state.bodies[self.body].linvel().x);
        draw_agent(pos, rot, mq::RED);
    }

    fn update(&self, state: &mut PhysicsState, sheep: &[Sheep], shepherds: &[Shepherd]) {
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
        if DRAW_DEBUG {
            mq::draw_circle_lines(
                sheep_centroid.x * PIXELS_PER_METER as f32,
                sheep_centroid.y * PIXELS_PER_METER as f32,
                self.rules.collect_radius * PIXELS_PER_METER as f32,
                1.0,
                mq::WHITE,
            );
        }

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

        if let Some(outlier) = outlier {
            if DRAW_DEBUG {
                mq::draw_circle_lines(
                    outlier.x * PIXELS_PER_METER as f32,
                    outlier.y * PIXELS_PER_METER as f32,
                    10.0,
                    1.0,
                    mq::RED,
                );
            }
        }

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

struct DynamicObstacle {
    body: RigidBodyHandle,
    body_collider: ColliderHandle,
    size: f32,
}

impl DynamicObstacle {
    fn new(state: &mut PhysicsState, pos: nalgebra::Vector2<f32>, size: f32) -> Self {
        let rigid_body = RigidBodyBuilder::dynamic()
            .translation(pos)
            .angular_damping(ANGULAR_DAMPING)
            .linear_damping(LINEAR_DAMPING)
            .build();

        let collider = ColliderBuilder::cuboid(size, size).restitution(0.7).build();
        let body_handle = state.bodies.insert(rigid_body);
        let collider_handle =
            state
                .colliders
                .insert_with_parent(collider, body_handle, &mut state.bodies);
        DynamicObstacle {
            body: body_handle,
            body_collider: collider_handle,
            size,
        }
    }
    fn draw(&self, state: &PhysicsState) {
        let pos = state.bodies[self.body].position().translation.vector * (PIXELS_PER_METER as f32);
        let rot = state.bodies[self.body].rotation().angle();
        mq::draw_poly_lines(
            pos.x,
            pos.y,
            4,
            f32::sqrt(2.0 * self.size.powf(2.0)) * (PIXELS_PER_METER as f32),
            rot * (180. / PI) + 45.0,
            2.0,
            mq::WHITE,
        );
    }
}

fn draw_agent(pos: nalgebra::Vector2<f32>, rot: f32, color: mq::Color) {
    let agent_size_px = (AGENT_SIZE) * (PIXELS_PER_METER as f32);
    let p = glam::Vec2::new(pos.x, pos.y) * (PIXELS_PER_METER as f32);
    mq::draw_circle(p.x, p.y, agent_size_px, mq::WHITE);
    mq::draw_triangle(
        p,
        p - glam::Vec2::from_angle(rot + (PI / 8.)) * agent_size_px,
        p - glam::Vec2::from_angle(rot - (PI / 8.)) * agent_size_px,
        color,
    );
}

fn conf() -> mq::Conf {
    mq::Conf {
        window_title: String::from("Macroquad"),
        window_width: WINDOW_SIZE_PX.0,
        window_height: WINDOW_SIZE_PX.1,
        fullscreen: false,
        ..Default::default()
    }
}

#[macroquad::main(conf)]
async fn main() {
    let mut world = World::new();

    loop {
        let _delta = mq::get_frame_time();

        if mq::is_key_pressed(mq::KeyCode::Escape) {
            break;
        }

        mq::clear_background(mq::Color {
            r: 0.078,
            g: 0.078,
            b: 0.078,
            a: 1.,
        });

        world.draw();
        world.update();

        mq::next_frame().await
    }
}
