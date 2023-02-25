use path_planning::planner::Node;
use path_planning::planner::calc_distance;
use path_planning::planner::calc_difference;

#[test]
fn test_node() {
    let position: [f32; 2] = [1.0, 2.0];
    let node = Node::new(position);
    assert_eq!(node.position[0], position[0]);
    assert_eq!(node.position[1], position[1]);

    let node1 = Node::new([0.0, 0.0]);
    let node2 = Node::new([1.0, 1.0]);
    assert_eq!(calc_distance(&node1, &node2), 2.0_f32.powf(0.5));
    assert_eq!(calc_distance(&node2, &node1), 2.0_f32.powf(0.5));

    assert_eq!(calc_difference(&node1, &node2), [1.0, 1.0]);
}
