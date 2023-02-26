extern crate path_planning as pp;

fn main() {
    let env = pp::env::create_example_2d_env();

    let start: [f32; 2] = [1.0, 1.0];
    let goal: [f32; 2] = [48.0, 25.0];

    let rrt = pp::planner::RRT::new(
        start,
        goal,
        env.low.clone(),
        env.high.clone(),
        0.2,
        2.0,
        1000,
    );
    let path = rrt.plan();
    pp::plot::plot_path(&env, &path).unwrap();
}
