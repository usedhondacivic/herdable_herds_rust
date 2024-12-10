use macroquad::prelude as mq;
use nalgebra::Vector2;
use rapier2d::prelude::*;

use herdable_herds::agents::dynamic_obstacle::*;
use herdable_herds::agents::sheep::*;
use herdable_herds::agents::shepherd::*;
use herdable_herds::graphics;
use herdable_herds::graphics::conf;
use herdable_herds::graphics::WINDOW_SIZE_METERS;
use herdable_herds::world::World;

const SHEEP_PROTECTED_RADIUS: f32 = 4.0;
const SHEEP_VISUAL_RADIUS: f32 = 25.;

fn example_scenario() -> (World, RigidBodyHandle) {
    let mut new_world = World::new(1.0 / 5.0);

    let sheep_rules = SheepRules {
        protected_radius: SHEEP_PROTECTED_RADIUS,
        visual_radius: SHEEP_VISUAL_RADIUS,
        seperation_strength: 10.0,
        cohesion_strength: 1.0,
        alignment_strength: 0.0,
        predator_strength: 100.0,
        force_cap: 20.0,
    };

    (1..20).for_each(|x| {
        (1..15).for_each(|y| {
            new_world.sheep.push(Sheep::new(
                &mut new_world.physics_state,
                vector![
                    x as f32 * 2.0 + WINDOW_SIZE_METERS.0 as f32 * 0.75 - 10.0,
                    y as f32 * 2.0 + WINDOW_SIZE_METERS.1 as f32 * 0.25 - 10.0
                ],
                vector![0.0, 0.0],
                sheep_rules,
            ))
        });
    });

    let shepherd_rules = ShepherdRules {
        chasing_force: 20.0,
        local_force: 20.0,
        collect_radius: 25.0,
        collect_lookahead: 15.0,
        drive_lookahead: 20.0,
    };

    new_world.shepherds.push(Shepherd::new(
        &mut new_world.physics_state,
        vector![
            WINDOW_SIZE_METERS.0 as f32 * 0.25,
            WINDOW_SIZE_METERS.1 as f32 * 0.75
        ],
        vector![0.0, 0.0],
        shepherd_rules,
    ));

    let obstacle = DynamicObstacle::new(
        &mut new_world.physics_state,
        vector![
            WINDOW_SIZE_METERS.0 as f32 * 0.25,
            WINDOW_SIZE_METERS.1 as f32 * 0.25,
        ],
        5.0,
    );
    let handle = obstacle.body.clone();

    new_world.dynamic_obstacles.push(obstacle);

    (new_world, handle)
}

#[macroquad::main(conf)]
async fn main() {
    let (mut world, tracking_obstacle) = example_scenario();

    let goal = Vector2::new(
        WINDOW_SIZE_METERS.0 as f32 * 0.50,
        WINDOW_SIZE_METERS.1 as f32 * 0.50,
    );

    let mut camera = graphics::get_init_camera();

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

        let obstacle_center = world
            .physics_state
            .bodies
            .get(tracking_obstacle)
            .unwrap()
            .position()
            .translation
            .vector;

        let obst_goal = obstacle_center + (obstacle_center - goal).normalize() * 4.0;

        graphics::update_camera(&mut camera);
        graphics::update_controls(&mut world);

        mq::draw_circle_lines(
            goal.x * graphics::PIXELS_PER_METER as f32,
            goal.y * graphics::PIXELS_PER_METER as f32,
            5.0 * graphics::PIXELS_PER_METER as f32,
            1.0,
            mq::GOLD,
        );

        graphics::draw_world(&world, &obst_goal, SHEEP_VISUAL_RADIUS);

        world.update(&obst_goal);

        mq::next_frame().await
    }
}
