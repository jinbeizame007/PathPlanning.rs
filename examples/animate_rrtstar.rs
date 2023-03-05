extern crate path_planning as pp;
use path_planning::env::Obstacle;
use path_planning::env::Obstacle::CircleObstacle;
use path_planning::env::Obstacle::RectObstacle;

fn main() {
    let low = [0.0, 0.0];
    let high = [50.0, 30.0];
    let obstacles: Vec<Obstacle<2>> = vec![
        RectObstacle {
            center: [18.0, 13.0],
            size: [8.0, 2.0],
        },
        RectObstacle {
            center: [22.0, 23.5],
            size: [8.0, 3.0],
        },
        RectObstacle {
            center: [27.0, 13.0],
            size: [2.0, 12.0],
        },
        RectObstacle {
            center: [37.0, 15.0],
            size: [10.0, 2.0],
        },
        CircleObstacle {
            center: [7.0, 12.0],
            radius: 3.0,
        },
        CircleObstacle {
            center: [46.0, 20.0],
            radius: 2.0,
        },
        CircleObstacle {
            center: [15.0, 5.0],
            radius: 2.0,
        },
        CircleObstacle {
            center: [37.0, 7.0],
            radius: 3.0,
        },
        CircleObstacle {
            center: [37.0, 23.0],
            radius: 3.0,
        },
    ];
    let env = pp::env::Env::new(low, high, obstacles);

    let low: [f32; 2] = env.low.clone();
    let high: [f32; 2] = env.high.clone();
    let start: [f32; 2] = [1.0, 1.0];
    let goal: [f32; 2] = [48.0, 25.0];

    let _env = env.clone();
    let is_approved = Box::new(move |position: &[f32; 2]| !_env.is_inside_obstacle(position));

    let mut rrt = pp::planner::RRTStar::new(start, goal, low, high, is_approved, 0.2, 2.0, 2000);
    rrt.enable_logging();
    rrt.plan();
    pp::plot::animate_path(&env, &rrt.log).unwrap();
}
