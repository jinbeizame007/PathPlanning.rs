use crate::env::Obstacle;
use crate::env::Obstacle::RectObstacle;
use crate::env::Obstacle::CircleObstacle;

pub struct Env<const D: usize> {
    pub low: [f32; D],
    pub high: [f32; D],
    pub obstacles: Vec<Obstacle<D>>,
}

pub fn create_example_2d_env() -> Env<2> {
    let low = [0.0, 0.0];
    let high = [50.0, 30.0];

    let obstacles: Vec<Obstacle<2>> = vec![
        RectObstacle{center: [18.0, 13.0], size: [8.0, 2.0]},
        RectObstacle{center: [22.0, 23.5], size: [8.0, 3.0]},
        RectObstacle{center: [27.0, 13.0], size: [2.0, 12.0]},
        RectObstacle{center: [37.0, 15.0], size: [10.0, 2.0]},
        CircleObstacle{center: [7.0, 12.0], radius: 3.0},
        CircleObstacle{center: [46.0, 20.0], radius: 2.0},
        CircleObstacle{center: [15.0, 5.0], radius: 2.0},
        CircleObstacle{center: [37.0, 7.0], radius: 3.0},
        CircleObstacle{center: [37.0, 23.0], radius: 3.0},
    ];
    return Env{low, high, obstacles};
}
