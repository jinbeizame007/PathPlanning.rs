use path_planning::planner::RRT;

#[test]
fn test_init_rrt() {
    let start: [f32; 2] = [0.1, 0.1];
    let goal: [f32; 2] = [0.9, 0.9];
    let low: [f32; 2] = [0.0, 0.0];
    let high: [f32; 2] = [1.0, 1.0];
    let rrt = RRT::new(start, goal, low, high);

    assert_eq!(rrt.start_node.position, start);
    assert_eq!(rrt.goal_node.position, goal);
    assert_eq!(rrt.low, low);
    assert_eq!(rrt.high, high);
}
