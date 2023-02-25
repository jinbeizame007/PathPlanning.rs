pub struct Node<const D: usize> {
    pub position: [f32; D],
    pub parent: Option<usize>,
}

impl<const D: usize> Node<D> {
    pub fn new(position: [f32; D]) -> Self {
        Node{position: position, parent: None}
    }
}

pub fn calc_distance<const D: usize>(node1: &Node<D>, node2: &Node<D>) -> f32 {
    let mut distance = 0.0;
    for i in 0..D {
        distance += (node1.position[i] - node2.position[i]).powf(2.0);
    }
    return distance.powf(0.5);
}

pub fn calc_difference<const D: usize>(node1: &Node<D>, node2: &Node<D>) -> [f32; D] {
    let mut difference: [f32; D] = [0.0; D];
    for i in 0..D {
        difference[i] = node2.position[i] - node1.position[i];
    }
    return difference;
}
