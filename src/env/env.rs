use crate::env::Obstacle;
use super::obstacle::RectObstacle;
use super::obstacle::CircleObstacle;

pub struct Env<const D: usize> {
    pub obstacles: Vec<Box<dyn Obstacle<D>>>,
}

pub fn create_example_2d_env() -> Env<2> {
    let obstacles: Vec<Box<dyn Obstacle<2>>> = vec![
        Box::new(RectObstacle{center: [18.0, 13.0], size: [8.0, 2.0]}),
        Box::new(RectObstacle{center: [22.0, 23.5], size: [8.0, 3.0]}),
        Box::new(RectObstacle{center: [27.0, 13.0], size: [2.0, 12.0]}),
        Box::new(RectObstacle{center: [37.0, 15.0], size: [10.0, 2.0]}),
        Box::new(CircleObstacle{center: [7.0, 12.0], radius: 3.0}),
        Box::new(CircleObstacle{center: [46.0, 20.0], radius: 2.0}),
        Box::new(CircleObstacle{center: [15.0, 5.0], radius: 2.0}),
        Box::new(CircleObstacle{center: [37.0, 7.0], radius: 3.0}),
        Box::new(CircleObstacle{center: [37.0, 23.0], radius: 3.0}),
    ];
    return Env{obstacles};
}
