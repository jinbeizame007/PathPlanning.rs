extern crate path_planning as pp;

fn main() {
    let env = pp::env::create_example_2d_env();

    let start: [f32; 2] = [1.0, 1.0];
    let goal: [f32; 2] = [48.0, 25.0];
    let low: [f32; 2] = env.low.clone();
    let high: [f32; 2] = env.high.clone();

    let _env = env.clone();
    let is_approved = Box::new(move |position: &[f32; 2]| !_env.is_inside_obstacle(position));

    let rrt = pp::planner::RRT::new(
        start,
        goal,
        low,
        high,
        is_approved,
        0.2,
        2.0,
        2000,
    );
    let path = rrt.plan();
    pp::plot::plot_path(&env, &path).unwrap();
}
