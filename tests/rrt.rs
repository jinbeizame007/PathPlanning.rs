use path_planning::planner::calc_distance;
use path_planning::planner::Node;
use path_planning::planner::RRT;

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

#[test]
fn test_get_extended_node() {
    let rrt = create_example_2d_rrt();

    let new_node = Node::new([2.0, 2.0]);
    let nearest_node_index = rrt.get_nearest_node_index(&new_node);
    let nearest_node = &rrt.nodes[nearest_node_index];

    let extended_node = rrt.get_extended_node(&nearest_node, &new_node);
    assert!((calc_distance(nearest_node, &extended_node) - rrt.step_size).abs() < 1E-10);
}

#[test]
fn test_plan() {
    let low: [f32; 2] = [0.0, 0.0];
    let high: [f32; 2] = [50.0, 30.0];
    let start: [f32; 2] = [1.0, 1.0];
    let goal: [f32; 2] = [48.0, 25.0];

    let rrt = RRT::new(start, goal, low, high);
    let path = rrt.plan();

    assert!(path.len() > 0);
    assert_eq!(path[0], start);
    assert_eq!(path[path.len() - 1], goal);
}
