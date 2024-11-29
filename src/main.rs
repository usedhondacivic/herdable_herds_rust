use macroquad::prelude as mq;
use rapier2d::prelude::*;

use herdable_herds::agents::dynamic_obstacle::*;
use herdable_herds::agents::sheep::*;
use herdable_herds::agents::shepherd::*;
use herdable_herds::graphics;
use herdable_herds::graphics::conf;
use herdable_herds::world::World;

const SHEEP_PROTECTED_RADIUS: f32 = 5.;
const SHEEP_VISUAL_RADIUS: f32 = 25.;

fn example_scenario() -> World {
    let mut new_world = World::new();

    let sheep_rules = SheepRules {
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
                vector![x as f32 * 2.0 + 80.0, y as f32 * 2.0 + 80.0],
                vector![0.0, 0.0],
                sheep_rules,
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
            new_world.dynamic_obstacles.push(DynamicObstacle::new(
                &mut new_world.physics_state,
                vector![100.0 + x as f32 * 15.0, y as f32 * 15.0],
                5.0,
            ));
        });
    });

    new_world
}

#[macroquad::main(conf)]
async fn main() {
    let mut world = example_scenario();

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

        graphics::draw_world(&world);
        world.update();

        mq::next_frame().await
    }
}
