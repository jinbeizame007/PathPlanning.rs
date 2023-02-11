pub trait Obstacle<const D: usize> {
    fn is_inside(&self, position: &[f32]) -> bool;
}

pub struct RectObstacle<const D: usize> {
    pub center: [f32; D],
    pub size: [f32; D],
}

impl<const D: usize> Obstacle<D> for RectObstacle<D> {
    fn is_inside(&self, position: &[f32]) -> bool {
        for i in 0..position.len() {
            if position[i] < self.center[i] - self.size[i] ||
                self.center[i] + self.size[i] < position[i] {
                return false;
            }
        }
        return true;
    }
}
