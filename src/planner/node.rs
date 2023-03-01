pub struct Node<const D: usize> {
    pub position: [f32; D],
    pub parent: Option<usize>,
}

impl<const D: usize> Node<D> {
    pub fn new(position: [f32; D]) -> Self {
        Node {
            position: position,
            parent: None,
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
