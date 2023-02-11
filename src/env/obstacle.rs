pub trait Obstacle<const D: usize> {
    fn is_inside(&self, position: &[f32]) -> bool;
}

pub struct RectObstacle<const D: usize> {
    pub center: [f32; D],
    pub size: [f32; D],
}

impl<const D: usize> Obstacle<D> for RectObstacle<D> {
    fn is_inside(&self, position: &[f32]) -> bool {
        for i in 0..self.center.len() {
            if position[i] < self.center[i] - self.size[i] ||
                self.center[i] + self.size[i] < position[i] {
                return false;
            }
        }
        return true;
    }
}

pub struct CircleObstacle<const D: usize> {
    pub center: [f32; D],
    pub radius: f32,
}

impl<const D: usize> Obstacle<D> for CircleObstacle<D> {
    fn is_inside(&self, position: &[f32]) -> bool {
        let mut distance = 0.0;
        for i in 0..self.center.len() {
            distance += (self.center[i] - position[i]).powf(2.0);
        }
        distance = distance.powf(0.5);
        return distance <= self.radius;
    }
}
