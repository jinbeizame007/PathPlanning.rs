use path_planning as pp;

#[test]
fn test_env() {
    let env = pp::env::create_example_2d_env();
    assert_eq!(env.obstacles.len(), 9);
}
