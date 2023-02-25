use rand::prelude::*;
use crate::planner::node::Node;
use crate::planner::node::calc_distance;

pub struct RRT<const D: usize> {
    pub start_node: Node<D>,
    pub goal_node: Node<D>,
    pub low: [f32; D],
    pub high: [f32; D],
    pub nodes: Vec<Node<D>>,
    goal_sample_rate: f32,
    step_size: f32,
    max_iter: i32,
}

impl<const D: usize> RRT<D> {
    pub fn new(
        start: [f32; D],
        goal: [f32; D],
        low: [f32; D],
        high: [f32; D]
    ) -> Self {
        RRT{
            start_node: Node::new(start),
            goal_node: Node::new(goal),
            low: low,
            high: high,
            nodes: vec![Node::new(start)],
            goal_sample_rate: 0.7,
            step_size: 0.5,
            max_iter: 1000,
        }
    }
}

impl<const D: usize> RRT<D> {
    pub fn sample(&self) -> Node<D> {
        let mut rng = thread_rng();
        let position = (0..D).map(|i| rng.gen_range(self.low[i]..self.high[i])).collect::<Vec<f32>>().try_into().unwrap();

        Node::new(position)
    }

    pub fn get_nearest_node_index(&self, new_node: &Node<D>) -> usize {
        let mut nearest_node_index = 0;
        let mut min_distance = f32::MAX;

        for (i, node) in self.nodes.iter().enumerate() {
            let distance = calc_distance(&node, new_node);
            if distance < min_distance {
                min_distance = distance;
                nearest_node_index = i;
            }
        }
        
        return nearest_node_index;
    }
}
