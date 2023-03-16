# PathPlanning.rs

# Example

```Rust
extern crate path_planning as pp;
use path_planning::env::Obstacle;
use path_planning::env::Obstacle::RectObstacle;
use path_planning::env::Obstacle::CircleObstacle;

fn main() {
    let low = [0.0, 0.0];
    let high = [50.0, 30.0];
    let obstacles: Vec<Obstacle<2>> = vec![
        RectObstacle {center: [18.0, 13.0], size: [8.0, 2.0]},
        RectObstacle {center: [22.0, 23.5], size: [8.0, 3.0]},
        RectObstacle {center: [27.0, 13.0], size: [2.0, 12.0]},
        RectObstacle {center: [37.0, 15.0], size: [10.0, 2.0]},
        CircleObstacle {center: [7.0, 12.0], radius: 3.0},
        CircleObstacle {center: [46.0, 20.0], radius: 2.0},
        CircleObstacle {center: [15.0, 5.0], radius: 2.0},
        CircleObstacle {center: [37.0, 7.0], radius: 3.0},
        CircleObstacle {center: [37.0, 23.0], radius: 3.0},
    ];
    let env = pp::env::Env::new(low, high, obstacles);

    let low: [f32; 2] = env.low.clone();
    let high: [f32; 2] = env.high.clone();
    let start: [f32; 2] = [1.0, 1.0];
    let goal: [f32; 2] = [48.0, 25.0];

    let _env = env.clone();
    let is_approved = Box::new(move |position: &[f32; 2]| !_env.is_inside_obstacle(position));

    let rrt = pp::planner::RRT::new(start, goal, low, high, is_approved, 0.2, 2.0, 2000);
    let path = rrt.plan();
    pp::plot::plot_path(&env, &path).unwrap();
}
```

# Figures

## RRT
![rrt](https://user-images.githubusercontent.com/16977484/222941352-ecb72780-e93b-4877-9b9f-be33cd0f196d.png)

## RRT*
![rrtstar](https://user-images.githubusercontent.com/16977484/222941354-02c2a4d3-761b-4d52-9160-09ba8962b473.png)

## Informed RRT*
![informed_rrtstar](https://user-images.githubusercontent.com/16977484/225611271-51dbcafa-aacd-4cf3-b3ba-902b116295c8.png)

