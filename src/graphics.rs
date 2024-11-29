use macroquad::prelude as mq;
use macroquad::prelude::glam;

use std::f32::consts::PI;

use crate::{
    agents::{dynamic_obstacle::DynamicObstacle, sheep::Sheep, shepherd::Shepherd},
    world::{PhysicsState, World, AGENT_SIZE},
};

const PIXELS_PER_METER: i32 = 4;
const WINDOW_SIZE_PX: (i32, i32) = (750, 750);
const WINDOW_SIZE_METERS: (i32, i32) = (
    WINDOW_SIZE_PX.0 / PIXELS_PER_METER,
    WINDOW_SIZE_PX.1 / PIXELS_PER_METER,
);

pub fn conf() -> mq::Conf {
    mq::Conf {
        window_title: String::from("Macroquad"),
        window_width: WINDOW_SIZE_PX.0,
        window_height: WINDOW_SIZE_PX.1,
        fullscreen: false,
        ..Default::default()
    }
}

pub fn draw_world(world: &World) {
    draw_sheep(&world.physics_state, &world.sheep);
    draw_shepherds(&world.physics_state, &world.shepherds);
    draw_dynamic_obstacles(&world.physics_state, &world.dynamic_obstacles);
}

fn draw_sheep(state: &PhysicsState, sheep: &[Sheep]) {
    sheep.iter().for_each(|s| {
        let pos = state.bodies[s.body].position().translation.vector;
        let rot = state.bodies[s.body]
            .linvel()
            .y
            .atan2(state.bodies[s.body].linvel().x);
        draw_agent(pos, rot, mq::BLUE);
    })
}

fn draw_shepherds(state: &PhysicsState, shepherd: &[Shepherd]) {
    shepherd.iter().for_each(|s| {
        let pos = state.bodies[s.body].position().translation.vector;
        let rot = state.bodies[s.body]
            .linvel()
            .y
            .atan2(state.bodies[s.body].linvel().x);
        draw_agent(pos, rot, mq::RED);
    })
}

fn draw_dynamic_obstacles(state: &PhysicsState, dynamic_obstacle: &[DynamicObstacle]) {
    dynamic_obstacle.iter().for_each(|s| {
        let pos = state.bodies[s.body].position().translation.vector * (PIXELS_PER_METER as f32);
        let rot = state.bodies[s.body].rotation().angle();
        if let Some(cuboid) = state.colliders[s.collider].shape().as_cuboid() {
            let size = cuboid.half_extents.xy();
            mq::draw_poly_lines(
                pos.x,
                pos.y,
                4,
                f32::sqrt(2.0 * size.x.powf(2.0)) * (PIXELS_PER_METER as f32),
                rot * (180. / PI) + 45.0,
                2.0,
                mq::WHITE,
            );
        }
    });
}

fn draw_agent(pos: nalgebra::Vector2<f32>, rot: f32, color: mq::Color) {
    let agent_size_px = (AGENT_SIZE) * (PIXELS_PER_METER as f32);
    let p = pos * (PIXELS_PER_METER as f32);
    mq::draw_circle(p.x, p.y, agent_size_px, mq::WHITE);
    mq::draw_triangle(
        p.into(),
        glam::Vec2::from(p) - glam::Vec2::from_angle(rot + (PI / 8.)) * agent_size_px,
        glam::Vec2::from(p) - glam::Vec2::from_angle(rot - (PI / 8.)) * agent_size_px,
        color,
    );
}
