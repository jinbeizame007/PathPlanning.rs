use crate::planner::node::calc_difference;
use crate::planner::node::calc_distance;
use crate::planner::node::Node;
use rand::prelude::*;

pub struct RRT<const D: usize> {
    pub start_node: Node<D>,
    pub goal_node: Node<D>,
    pub low: [f32; D],
    pub high: [f32; D],
    pub nodes: Vec<Node<D>>,
    pub goal_sample_rate: f32,
    pub step_size: f32,
    pub max_iter: i32,
}

impl<const D: usize> RRT<D> {
    pub fn new(start: [f32; D], goal: [f32; D], low: [f32; D], high: [f32; D]) -> Self {
        RRT {
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
        let position = (0..D)
            .map(|i| rng.gen_range(self.low[i]..self.high[i]))
            .collect::<Vec<f32>>()
            .try_into()
            .unwrap();

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

        nearest_node_index
    }

    pub fn get_extended_node(&self, nearest_node: &Node<D>, new_node: &Node<D>) -> Node<D> {
        let difference = calc_difference(&nearest_node, &new_node);
        let distance = calc_distance(&nearest_node, &new_node);

        let mut new_position = nearest_node.position.clone();
        for i in 0..D {
            new_position[i] += difference[i] * (self.step_size / distance);
        }

        Node::new(new_position)
    }

    pub fn is_near_goal(&self, node: &Node<D>) -> bool {
        let distance_from_goal = calc_distance(&self.goal_node, &node);
        return distance_from_goal <= self.step_size;
    }

    pub fn extract_path(&self) -> Vec<[f32; D]> {
        let mut reverse_path: Vec<[f32; D]> = Vec::new();

        let mut node = &self.goal_node;
        loop {
            reverse_path.push(node.position.clone());

            match node.parent {
                Some(parent_node_index) => node = &self.nodes[parent_node_index],
                None => break,
            }
        }

        return reverse_path
            .iter()
            .rev()
            .map(|&x| x)
            .collect::<Vec<[f32; D]>>();
    }

    pub fn plan(mut self) -> Vec<[f32; D]> {
        //self.nodes = vec![Node::new(self.start_node.position.clone())];

        for _ in 0..self.max_iter {
            // Sample a node
            let mut new_node;
            if thread_rng().gen::<f32>() < self.goal_sample_rate {
                new_node = Node::new(self.goal_node.position);
            } else {
                new_node = self.sample();
            }

            // Get the nearest node
            let nearest_node_index = self.get_nearest_node_index(&new_node);
            let nearest_node = &self.nodes[nearest_node_index];

            let distance_from_nearest_node = calc_distance(nearest_node, &new_node);
            if self.step_size < distance_from_nearest_node {
                new_node = self.get_extended_node(nearest_node, &new_node);
            }

            // Add the new node to the tree
            new_node.parent = Some(nearest_node_index);
            if self.is_near_goal(&new_node) {
                self.nodes.push(new_node);

                let new_node_index = self.nodes.len();
                let mut goal_node = Node::new(self.goal_node.position);
                goal_node.parent = Some(new_node_index);
                self.nodes.push(goal_node);
                return self.extract_path();
            } else {
                self.nodes.push(new_node);
            }
        }

        return Vec::new();
    }
}
