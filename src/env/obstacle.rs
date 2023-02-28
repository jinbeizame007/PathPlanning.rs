pub enum Obstacle<const D: usize> {
    RectObstacle { center: [f32; D], size: [f32; D] },
    CircleObstacle { center: [f32; D], radius: f32 },
}

impl<const D: usize> Obstacle<D> {
    pub fn is_inside(&self, position: &[f32]) -> bool {
        match self {
            Obstacle::RectObstacle { center, size } => {
                for i in 0..D {
                    if position[i] < center[i] - size[i] / 2.0
                        || center[i] + size[i] / 2.0 < position[i]
                    {
                        return false;
                    }
                }
                return true;
            }
            Obstacle::CircleObstacle { center, radius } => {
                let mut distance = 0.0;
                for i in 0..D {
                    distance += (center[i] - position[i]).powf(2.0);
                }
                distance = distance.powf(0.5);
                return distance <= *radius;
            }
        }
    }
}

impl<const D: usize> Clone for Obstacle<D> {
    fn clone(&self) -> Obstacle<D> {
        match self {
            Obstacle::RectObstacle {center, size} => {
                return Obstacle::RectObstacle{center: center.clone(), size: size.clone()};
            }
            Obstacle::CircleObstacle {center, radius} => {
                return Obstacle::CircleObstacle{center: center.clone(), radius: *radius};
            }
        }
    }
}
