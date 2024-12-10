use macroquad::prelude as mq;
use macroquad::prelude::glam;
use nalgebra::Vector2;

use std::f32::consts::PI;

use crate::{
    agents::{
        dynamic_obstacle::DynamicObstacle, sheep::Sheep, shepherd::Shepherd, shepherd::ShepherdMode,
    },
    world::{PhysicsState, World, AGENT_SIZE},
};

const DEBUG_DRAW: bool = true;

pub const PIXELS_PER_METER: i32 = 4;
const WINDOW_SIZE_PX: (i32, i32) = (750, 750);
pub const WINDOW_SIZE_METERS: (i32, i32) = (
    WINDOW_SIZE_PX.0 / PIXELS_PER_METER,
    WINDOW_SIZE_PX.1 / PIXELS_PER_METER,
);

pub fn conf() -> mq::Conf {
    mq::Conf {
        window_title: String::from("Herdable Herds"),
        window_width: WINDOW_SIZE_PX.0,
        window_height: WINDOW_SIZE_PX.1,
        fullscreen: false,
        ..Default::default()
    }
}

pub fn get_init_camera() -> mq::Camera2D {
    mq::Camera2D {
        target: mq::vec2(375.0, 375.0),
        zoom: mq::vec2(
            2.0 / mq::screen_width(),
            (2.0 / mq::screen_width()) * mq::screen_width() / mq::screen_height(),
        ),
        ..Default::default()
    }
}

pub fn update_camera(camera: &mut mq::Camera2D) {
    let mut zoom = camera.zoom.x;
    match mq::mouse_wheel() {
        (_x, y) if y != 0.0 => {
            zoom *= 1.1f32.powf(y);
        }
        _ => (),
    }
    if mq::is_mouse_button_down(mq::MouseButton::Left) {
        camera.target += mq::mouse_delta_position() / zoom;
    }
    camera.zoom = mq::vec2(zoom, zoom * mq::screen_width() / mq::screen_height());
    mq::set_camera(camera);
}

pub fn update_controls(world: &mut World) {
    if mq::is_key_pressed(mq::KeyCode::Left) {
        world.timestep *= 0.5;
    }
    if mq::is_key_pressed(mq::KeyCode::Right) {
        world.timestep *= 2.0;
    }
    if mq::is_key_pressed(mq::KeyCode::Space) {
        world.paused = !world.paused;
    }
}

pub fn draw_world(world: &World, goal: &Vector2<f32>, visual_radius: f32) {
    draw_sheep(&world.physics_state, &world.sheep);
    draw_shepherds(&world.physics_state, &goal, &world.shepherds, visual_radius);
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

fn draw_shepherds(
    state: &PhysicsState,
    goal: &Vector2<f32>,
    shepherd: &[Shepherd],
    visual_radius: f32,
) {
    shepherd.iter().for_each(|s| {
        let pos = state.bodies[s.body].position().translation.vector;
        let rot = state.bodies[s.body]
            .linvel()
            .y
            .atan2(state.bodies[s.body].linvel().x);
        draw_agent(pos, rot, mq::RED);
        if DEBUG_DRAW {
            let state = s.state.borrow();

            let mut annotation_color = mq::BLUE;

            mq::draw_circle_lines(
                state.centroid.x * PIXELS_PER_METER as f32,
                state.centroid.y * PIXELS_PER_METER as f32,
                s.rules.collect_radius * PIXELS_PER_METER as f32,
                1.0,
                mq::GRAY,
            );

            // mq::draw_circle_lines(
            //     pos.x * PIXELS_PER_METER as f32,
            //     pos.y * PIXELS_PER_METER as f32,
            //     visual_radius * PIXELS_PER_METER as f32,
            //     1.0,
            //     mq::GRAY,
            // );

            match state.mode {
                ShepherdMode::COLLECT { outlier_pos } => {
                    annotation_color = mq::RED;
                    mq::draw_circle_lines(
                        outlier_pos.x * PIXELS_PER_METER as f32,
                        outlier_pos.y * PIXELS_PER_METER as f32,
                        10.0,
                        1.0,
                        annotation_color,
                    );
                }
                ShepherdMode::DRIVE => {
                    mq::draw_line(
                        state.centroid.x * PIXELS_PER_METER as f32,
                        state.centroid.y * PIXELS_PER_METER as f32,
                        goal.x * (PIXELS_PER_METER as f32),
                        goal.y * (PIXELS_PER_METER as f32),
                        1.0,
                        mq::GRAY,
                    );
                    annotation_color = mq::BLUE;
                }
            };

            mq::draw_line(
                pos.x * (PIXELS_PER_METER as f32),
                pos.y * (PIXELS_PER_METER as f32),
                state.movement_target.x * (PIXELS_PER_METER as f32),
                state.movement_target.y * (PIXELS_PER_METER as f32),
                1.0,
                annotation_color,
            );
        }
    });
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
