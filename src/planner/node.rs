use std::collections::HashSet;

pub struct Node<const D: usize> {
    pub position: [f32; D],
    pub parent: Option<usize>,
    pub children: HashSet<usize>,
    pub cost: f32,
}

impl<const D: usize> Node<D> {
    pub fn new(position: [f32; D]) -> Self {
        Node {
            position: position,
            parent: None,
            children: HashSet::new(),
            cost: 0.0,
        }
    }

    pub fn calc_difference(&self, node: &Node<D>) -> [f32; D] {
        let mut difference: [f32; D] = [0.0; D];
        for i in 0..D {
            difference[i] = node.position[i] - self.position[i];
        }
        return difference;
    }

    pub fn calc_distance(&self, node: &Node<D>) -> f32 {
        return self
            .calc_difference(node)
            .iter()
            .map(|x| x.powf(2.0))
            .sum::<f32>()
            .powf(0.5);
    }
}

impl<const D: usize> Clone for Node<D> {
    fn clone(&self) -> Node<D> {
        Node {
            position: self.position.clone(),
            parent: self.parent,
            children: self.children.clone(),
            cost: self.cost,
        }
    }
}
