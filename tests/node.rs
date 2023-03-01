use path_planning::planner::Node;

#[test]
fn test_node() {
    let position: [f32; 2] = [1.0, 2.0];
    let node = Node::new(position);
    assert_eq!(node.position[0], position[0]);
    assert_eq!(node.position[1], position[1]);

    let node1 = Node::new([0.0, 0.0]);
    let node2 = Node::new([1.0, 1.0]);
    assert_eq!(node1.calc_distance(&node2), 2.0_f32.powf(0.5));
    assert_eq!(node2.calc_distance(&node1), 2.0_f32.powf(0.5));

    assert_eq!(node1.calc_difference(&node2), [1.0, 1.0]);
    assert_eq!(node2.calc_difference(&node1), [-1.0, -1.0]);
}
