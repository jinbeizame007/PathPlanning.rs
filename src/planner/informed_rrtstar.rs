use rand::prelude::*;
use ndarray::prelude::*;
use ndarray_linalg::Determinant;
use ndarray_linalg::SVD;
use crate::planner::AbstractRRT;
use crate::planner::Node;

pub struct InformedRRTStar<const D: usize> {
    pub start: [f32; D],
    pub goal: [f32; D],
    center: [f32; D],
    pub low: [f32; D],
    pub high: [f32; D],
    cost_min: f32,
    cost_max: f32,
    rotation_matrix: Array2<f32>,
    pub nodes: Vec<Node<D>>,
    is_approved: Box<dyn Fn(&[f32; D]) -> bool>,
    pub goal_sample_rate: f32,
    pub step_size: f32,
    pub max_iter: usize,
    goal_node_index: usize,
    is_logginge_enabled: bool,
    pub log: Vec<Vec<Node<D>>>,
}

impl<const D: usize> InformedRRTStar<D> {
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
        let mut center: [f32; D] = [0.0; D];
        for i in 0..D {
            center[i] = (high[i] + low[i]) / 2.0;
        }

        let cost_min = (0..D).map(|i| (goal[i] - start[i]).powf(2.0)).sum::<f32>().powf(0.5);
        let cost_max = f32::MAX;
        let rotation_matrix = get_rotation_to_main_frame(start, goal);

        InformedRRTStar {
            start,
            goal,
            center,
            low,
            high,
            is_approved,
            cost_min,
            cost_max,
            rotation_matrix,
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

fn get_rotation_to_main_frame<const D: usize>(start: [f32; D], goal: [f32; D]) -> Array2<f32> {
    let start_node = Node::new(start);
    let goal_node = Node::new(goal);
    let difference = start_node.calc_difference(&goal_node);
    let distance = start_node.calc_distance(&goal_node);

    let a_1 = Array1::from_iter((0..D).map(|i| difference[i] / distance)).into_shape((D,1)).unwrap();
    
    let mut e_1 = Array::zeros((1,D));
    e_1[[0,0]] = 1.0;
    let m: Array2<f32> = a_1.dot(&e_1);

    let (u, _s, vt) = m.svd(true, true).unwrap();
    let mut d: Array2<f32> = Array2::eye(D);
    d[(D-2, D-2)] = u.clone().unwrap().det().unwrap();
    d[(D-1, D-1)] = vt.clone().unwrap().t().det().unwrap();

    let c = u.clone().unwrap().dot(&d).dot(&vt.clone().unwrap());
    return c;
}

impl<const D: usize> InformedRRTStar<D> {
    pub fn enable_logging(&mut self) {
        self.is_logginge_enabled = true;
    }

    fn sample_from_unit_ball(&self) -> [f32; D] {
        let mut rng = thread_rng();

        loop {
            let mut position: [f32; D] = [0.0; D];
            for i in 0..D {
                position[i] = rng.gen::<f32>() * 2.0 - 1.0;
            }
            let distance_from_origin = position.iter().map(|x| x.powf(2.0)).sum::<f32>().powf(0.5);
            if distance_from_origin <= 1.0 {
                return position;
            }
        }
    }

    fn sample_from_informed_elipse(&self) -> Node<D> {
        let mut radiuses = Array::zeros(D);
        radiuses[0] = self.cost_max / 2.0;
        for i in 1..D {
            radiuses[i] = (self.cost_max.powf(2.0) - self.cost_min.powf(2.0)).powf(0.5) / 2.0;
        }
        let l = Array::from_diag(&radiuses);

        let _center = arr1(&self.center);
        loop {
            let mut position = arr1(&self.sample_from_unit_ball());
            position = self.rotation_matrix.dot(&l).dot(&position) + _center;

            let mut _position: [f32; D] = [0.0; D];
            for i in 0..D {
                _position[i] = position[i];
            }
            return Node::new(_position);
        }
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
        let tol = 1E-5;
        let mut parent_node_index: usize = 0;
        let mut minimum_cost = f32::MAX;

        for i in 0..self.nodes.len() {
            let node = &self.nodes[i];
            let distance = node.calc_distance(&new_node);
            let new_cost = node.cost + distance;
            if distance <= self.step_size + tol && new_cost < minimum_cost {
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

    pub fn plan(&mut self) -> Vec<[f32; D]> {
        let mut is_goaled = false;

        for _ in 0..self.max_iter {
            // Sample a node
            let mut new_node;
            if is_goaled {
                new_node = self.sample_from_informed_elipse();
            } else {
                if thread_rng().gen::<f32>() < self.goal_sample_rate {
                    new_node = Node::new(self.goal);
                } else {
                    new_node = self.sample();
                }
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
            let parent_node_index = self.get_parent_node_index_minimize_cost(&new_node);
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

            if is_goaled {
                self.cost_max = self.nodes[self.goal_node_index].cost;
            }

            if self.is_logginge_enabled {
                self.log.push(self.nodes.clone());
            }
        }

        return self.extract_path();
    }
}

impl<const D: usize> AbstractRRT<D> for InformedRRTStar<D> {
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
    fn get_goal_node_index(&self) -> usize {
        self.goal_node_index
    }
    fn get_nodes(&self) -> &Vec<Node<D>> {
        &self.nodes
    }
}
