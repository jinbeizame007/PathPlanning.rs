use path_planning::planner::RRT;
use path_planning::planner::Node;

#[test]
fn test_init() {
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

fn create_example_2d_rrt() -> RRT<2> {
    let start: [f32; 2] = [0.1, 0.1];
    let goal: [f32; 2] = [0.9, 0.9];
    let low: [f32; 2] = [0.0, 0.0];
    let high: [f32; 2] = [1.0, 1.0];
    
    RRT::new(start, goal, low, high)
}

#[test]
fn test_sample() {
    let rrt = create_example_2d_rrt();

    for _ in 0..100 {
        let node = rrt.sample();
        for i in 0..2 {
            assert!(rrt.low[i] <= node.position[i] && node.position[i] <= rrt.high[i]);
        }
    }
}

#[test]
fn test_get_nearest_node_index() {
    let rrt = create_example_2d_rrt();

    let node = Node::new([0.2, 0.2]);
    let nearest_node_index = rrt.get_nearest_node_index(&node);
    assert_eq!(nearest_node_index, 0);
}
