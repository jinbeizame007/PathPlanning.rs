extern crate path_planning as pp;

fn main() {
    let env = pp::env::create_example_2d_env();
    pp::plot::plot_env(&env).unwrap();
}
