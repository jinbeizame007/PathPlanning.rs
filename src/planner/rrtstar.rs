use crate::planner::AbstractRRT;
use crate::planner::Node;
use rand::prelude::*;

pub struct RRTStar<const D: usize> {
    pub start: [f32; D],
    pub goal: [f32; D],
    pub low: [f32; D],
    pub high: [f32; D],
    pub nodes: Vec<Node<D>>,
    is_approved: Box<dyn Fn(&[f32; D]) -> bool>,
    pub goal_sample_rate: f32,
    pub step_size: f32,
    pub max_iter: usize,
    goal_node_index: usize,
    is_logginge_enabled: bool,
    pub log: Vec<Vec<Node<D>>>,
}

impl<const D: usize> RRTStar<D> {
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
        RRTStar {
            start,
            goal,
            low,
            high,
            is_approved,
            nodes: vec![Node::new(start)],
            goal_sample_rate,
            step_size,
            max_iter,
            goal_node_index: 0,
            is_logginge_enabled: false,
            log: Vec::new(),
        }
    }
}

impl<const D: usize> RRTStar<D> {
    pub fn enable_logging(&mut self) {
        self.is_logginge_enabled = true;
    }

    pub fn add_node(&mut self, mut new_node: Node<D>, parent_node_index: usize) {
        // Add new_node_index to children of the parent node
        let new_node_index = self.nodes.len();
        self.nodes[parent_node_index]
            .children
            .insert(new_node_index);

        // Add new_node to nodes
        let parent_node = &self.nodes[parent_node_index];
        new_node.parent = Some(parent_node_index);
        new_node.cost = parent_node.cost + parent_node.calc_distance(&new_node);
        self.nodes.push(new_node);
    }

    pub fn get_parent_node_index_minimize_cost(&self, new_node: &Node<D>) -> usize {
        let _tol = 1E-8;
        let mut parent_node_index: usize = 0;
        let mut minimum_cost = f32::MAX;

        for i in 0..self.nodes.len() {
            let node = &self.nodes[i];
            let new_cost = node.cost + node.calc_distance(&new_node);
            if new_cost < minimum_cost {
                minimum_cost = new_cost;
                parent_node_index = i;
            }
        }

        return parent_node_index;
    }

    pub fn get_near_node_indices(&self, node: &Node<D>) -> Vec<usize> {
        let mut near_node_indices: Vec<usize> = Vec::new();
        for i in 0..self.nodes.len() {
            if self.nodes[i].calc_distance(&node) <= self.step_size {
                near_node_indices.push(i);
            }
        }
        return near_node_indices;
    }

    fn update_costs(&mut self, node_index: usize, diff_cost: f32) {
        self.nodes[node_index].cost += diff_cost;
        for child_node_index in self.nodes[node_index].children.clone().iter() {
            self.update_costs(*child_node_index, diff_cost);
        }
    }

    pub fn rewire_near_nodes(&mut self, near_node_indices: Vec<usize>, new_node_index: usize) {
        for near_node_index in near_node_indices {
            let new_cost = self.nodes[new_node_index].cost
                + self.nodes[new_node_index].calc_distance(&self.nodes[near_node_index]);
            if new_cost < self.nodes[near_node_index].cost {
                // Delete near_node from children of the current parent node of the near node
                let near_node_parent = self.nodes[near_node_index].parent.unwrap();
                self.nodes[near_node_parent]
                    .children
                    .remove(&near_node_index);

                // Set new_node as parent of near_node
                self.nodes[near_node_index].parent = Some(new_node_index);

                // Add near_node as children of new_node
                self.nodes[new_node_index].children.insert(near_node_index);

                let diff_cost = new_cost - self.nodes[near_node_index].cost;
                self.update_costs(near_node_index, diff_cost);
            }
        }
    }

    pub fn extract_path(&self) -> Vec<[f32; D]> {
        let mut reverse_path: Vec<[f32; D]> = Vec::new();

        let mut node = &self.nodes[self.goal_node_index];
        loop {
            reverse_path.push(node.position.clone());

            match node.parent {
                Some(parent_node_index) => node = &self.nodes[parent_node_index],
                None => break,
            }
        }

        return reverse_path.iter().rev().map(|&x| x).collect();
    }

    pub fn plan(&mut self) -> Vec<[f32; D]> {
        let mut is_goaled = false;

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
            //let parent_node_index = self.get_parent_node_index_minimize_cost(&new_node);
            let parent_node_index = self.get_nearest_node_index(&new_node);
            self.add_node(new_node, parent_node_index);

            // Add the new node to the tree
            if !is_goaled && self.is_near_goal(&self.nodes[self.nodes.len() - 1]) {
                let new_node_index = self.nodes.len() - 1;
                let goal_node = Node::new(self.goal);
                self.add_node(goal_node, new_node_index);
                self.goal_node_index = self.nodes.len() - 1;
                is_goaled = true
            }

            // Rewire near nodes
            let new_node_index = self.nodes.len() - 1;
            let new_node = &self.nodes[new_node_index];
            let near_node_indices = self.get_near_node_indices(new_node);
            self.rewire_near_nodes(near_node_indices, new_node_index);

            if self.is_logginge_enabled {
                self.log.push(self.nodes.clone());
            }
        }

        return self.extract_path();
    }
}

impl<const D: usize> AbstractRRT<D> for RRTStar<D> {
    fn get_low(&self) -> &[f32; D] {
        &self.low
    }
    fn get_high(&self) -> &[f32; D] {
        &self.high
    }
    fn get_goal(&self) -> &[f32; D] {
        &self.goal
    }
    fn get_step_size(&self) -> f32 {
        self.step_size
    }
    fn get_nodes(&self) -> &Vec<Node<D>> {
        &self.nodes
    }
}
