use rapier2d::prelude::*;

use crate::agents::dynamic_obstacle::DynamicObstacle;
use crate::agents::sheep::Sheep;
use crate::agents::shepherd::Shepherd;

pub const LINEAR_DAMPING: f32 = 2.0;
pub const ANGULAR_DAMPING: f32 = 2.0;

pub const AGENT_SIZE: f32 = 0.5;

pub struct PhysicsState {
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

pub struct World {
    pub physics_state: PhysicsState,
    pub physics_pipeline: PhysicsPipeline,

    pub sheep: Vec<Sheep>,
    pub shepherds: Vec<Shepherd>,
    pub dynamic_obstacles: Vec<DynamicObstacle>,
}

impl World {
    pub fn new() -> Self {
        World {
            sheep: Vec::new(),
            shepherds: Vec::new(),
            dynamic_obstacles: Vec::new(),
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
        }
    }

    pub fn update(&mut self) {
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
