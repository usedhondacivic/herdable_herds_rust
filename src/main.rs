use macroquad::prelude as mq;
use macroquad::prelude::glam;
use rand::Rng;
use std::f32::consts::PI;
use std::option::Option;

const PIXELS_PER_METER: i32 = 10;
const WINDOW_SIZE_PX: (i32, i32) = (1260, 768);
const WINDOW_SIZE_METERS: (i32, i32) = (
    WINDOW_SIZE_PX.0 / PIXELS_PER_METER,
    WINDOW_SIZE_PX.1 / PIXELS_PER_METER,
);

struct World {
    objects: Vec<GameObject>,
}

impl World {
    fn new() -> World {
        let mut init_world = World {
            objects: Vec::new(),
        };
        let mut rng = rand::thread_rng();
        init_world.objects = (0..100)
            .map(|_| {
                GameObject::Sheep(SheepAgent::new(
                    Some(mq::glam::Vec2 {
                        x: rng.gen_range(0.0..(WINDOW_SIZE_METERS.0 as f32)),
                        y: rng.gen_range(0.0..(WINDOW_SIZE_METERS.1 as f32)),
                    }),
                    Some(rng.gen_range(0.0..(PI * 2.0))),
                    Some(rng.gen_range(0.0..5.0)),
                ))
            })
            .collect();
        init_world
    }

    fn draw(&self) {
        self.objects.iter().for_each(|obj| match obj {
            GameObject::Sheep(agent) => agent.draw(),
            GameObject::Shepherd(agent) => agent.draw(),
        });
    }

    fn update(&mut self, dt: f32) {
        self.objects = self
            .objects
            .iter()
            .map(|obj| match obj {
                GameObject::Sheep(agent) => GameObject::Sheep(agent.update(&self.objects, dt)),
                GameObject::Shepherd(agent) => {
                    GameObject::Shepherd(agent.update(&self.objects, dt))
                }
            })
            .collect();
    }
}

#[derive(Clone, Copy)]
enum GameObject {
    Sheep(SheepAgent),
    Shepherd(ShepherdAgent),
}

#[derive(Clone, Copy)]
struct AgentState {
    position: glam::Vec2,
    heading: f32,
    velocity: f32,
}

#[derive(Clone, Copy)]
struct SheepAgent {
    state: AgentState,
}

impl SheepAgent {
    fn new(pos: Option<glam::Vec2>, heading: Option<f32>, velocity: Option<f32>) -> Self {
        Self {
            state: AgentState {
                position: if let Some(p) = pos {
                    p
                } else {
                    glam::Vec2 { x: 0., y: 0. }
                },
                heading: if let Some(h) = heading { h } else { 0. },
                velocity: if let Some(v) = velocity { v } else { 0. },
            },
        }
    }

    fn draw(&self) {
        mq::draw_rectangle(
            self.state.position.x * (PIXELS_PER_METER as f32),
            self.state.position.y * (PIXELS_PER_METER as f32),
            20.,
            20.,
            mq::BLUE,
        );
    }
    fn update(&self, objects: &Vec<GameObject>, dt: f32) -> Self {
        let mut new = self.clone();
        new.state.position += glam::Vec2 {
            x: f32::cos(self.state.heading),
            y: f32::sin(self.state.heading),
        } * self.state.velocity
            * dt;
        new
    }
}

#[derive(Clone, Copy)]
struct ShepherdAgent {
    state: AgentState,
}

impl ShepherdAgent {
    fn new(pos: Option<glam::Vec2>, heading: Option<f32>, velocity: Option<f32>) -> Self {
        Self {
            state: AgentState {
                position: if let Some(p) = pos {
                    p
                } else {
                    glam::Vec2 { x: 0., y: 0. }
                },
                heading: if let Some(h) = heading { h } else { 0. },
                velocity: if let Some(v) = velocity { v } else { 0. },
            },
        }
    }

    fn draw(&self) {
        mq::draw_rectangle(
            self.state.position.x / (PIXELS_PER_METER as f32),
            self.state.position.y / (PIXELS_PER_METER as f32),
            20.,
            20.,
            mq::RED,
        );
    }
    fn update(self, objects: &Vec<GameObject>, dt: f32) -> Self {
        let mut new = self.clone();
        new.state.position += glam::Vec2 {
            x: f32::cos(self.state.heading),
            y: f32::sin(self.state.heading),
        } * self.state.velocity
            * dt;
        new
    }
}

fn conf() -> mq::Conf {
    mq::Conf {
        window_title: String::from("Macroquad"),
        window_width: 1260,
        window_height: 768,
        fullscreen: false,
        ..Default::default()
    }
}

#[macroquad::main(conf)]
async fn main() {
    let mut world = World::new();

    loop {
        let delta = mq::get_frame_time();

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
        world.update(delta);

        mq::next_frame().await
    }
}
