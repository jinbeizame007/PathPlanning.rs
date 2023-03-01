use crate::planner::node::Node;
use rand::prelude::*;

pub struct RRT<const D: usize> {
    pub start: [f32; D],
    pub goal: [f32; D],
    pub low: [f32; D],
    pub high: [f32; D],
    pub nodes: Vec<Node<D>>,
    is_approved: Box<dyn Fn(&[f32; D]) -> bool>,
    pub goal_sample_rate: f32,
    pub step_size: f32,
    pub max_iter: usize,
}

impl<const D: usize> RRT<D> {
    pub fn new(
        start: [f32; D],
        goal: [f32; D],
        low: [f32; D],
        high: [f32; D],
        is_approved: Box<dyn Fn(&[f32; D]) -> bool>,
        goal_sample_rate: f32,
        step_size: f32,
        max_iter: usize,
    ) -> Self {
        RRT {
            start: start,
            goal: goal,
            low: low,
            high: high,
            is_approved: is_approved,
            nodes: vec![Node::new(start)],
            goal_sample_rate: goal_sample_rate,
            step_size: step_size,
            max_iter: max_iter,
        }
    }
}

impl<const D: usize> RRT<D> {
    pub fn sample(&self) -> Node<D> {
        let mut rng = thread_rng();
        let mut position: [f32; D] = [0.0; D];
        for i in 0..D {
            position[i] = rng.gen_range(self.low[i]..self.high[i]);
        }

        Node::new(position)
    }

    pub fn get_nearest_node_index(&self, new_node: &Node<D>) -> usize {
        let mut nearest_node_index = 0;
        let mut min_distance = f32::MAX;

        for (i, node) in self.nodes.iter().enumerate() {
            let distance = node.calc_distance(new_node);
            if distance < min_distance {
                min_distance = distance;
                nearest_node_index = i;
            }
        }

        nearest_node_index
    }

    pub fn get_extended_node(&self, nearest_node: &Node<D>, new_node: &Node<D>) -> Node<D> {
        let difference = nearest_node.calc_difference(&new_node);
        let distance = nearest_node.calc_distance(&new_node);

        let mut new_position = nearest_node.position.clone();
        for i in 0..D {
            new_position[i] += difference[i] * (self.step_size / distance);
        }

        Node::new(new_position)
    }

    pub fn is_near_goal(&self, node: &Node<D>) -> bool {
        let distance_from_goal = node.calc_distance(&Node::new(self.goal));
        return distance_from_goal <= self.step_size;
    }

    pub fn extract_path(&self) -> Vec<[f32; D]> {
        let mut reverse_path: Vec<[f32; D]> = Vec::new();

        let mut node = &self.nodes[self.nodes.len() - 1];
        loop {
            reverse_path.push(node.position.clone());

            match node.parent {
                Some(parent_node_index) => node = &self.nodes[parent_node_index],
                None => break,
            }
        }

        return reverse_path.iter().rev().map(|&x| x).collect();
    }

    pub fn plan(mut self) -> Vec<[f32; D]> {
        for _ in 0..self.max_iter {
            // Sample a node
            let mut new_node;
            if thread_rng().gen::<f32>() < self.goal_sample_rate {
                new_node = Node::new(self.goal);
            } else {
                new_node = self.sample();
            }

            // Get the nearest node
            let nearest_node_index = self.get_nearest_node_index(&new_node);
            let nearest_node = &self.nodes[nearest_node_index];

            let distance_from_nearest_node = nearest_node.calc_distance(&new_node);
            if self.step_size < distance_from_nearest_node {
                new_node = self.get_extended_node(nearest_node, &new_node);
            }

            if !(self.is_approved)(&new_node.position) {
                continue;
            }

            // Add the new node to the tree
            new_node.parent = Some(nearest_node_index);
            if self.is_near_goal(&new_node) {
                self.nodes.push(new_node);

                let new_node_index = self.nodes.len() - 1;
                let mut goal_node = Node::new(self.goal);
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
