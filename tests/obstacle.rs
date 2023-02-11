use path_planning as pp;
use path_planning::env::Obstacle;

#[test]
fn test_rect_obstacle() {
    let center: [f32; 2] = [0.0, 0.0];
    let size: [f32; 2] = [2.0, 2.0];
    let rect_obstacle = pp::env::RectObstacle{center, size};

    let position: [f32; 2] = [0.5,0.5];
    assert!(rect_obstacle.is_inside(&position));

    let position: [f32; 2] = [2.1,2.1];
    assert!(!rect_obstacle.is_inside(&position));
}

#[test]
fn test_circle_obstacle() {
    let center: [f32; 2] = [0.0, 0.0];
    let radius = 2.0;
    let circle_obstacle = pp::env::CircleObstacle{center, radius};
    
    let position: [f32; 2] = [1.2, 1.2];
    assert!(circle_obstacle.is_inside(&position));
    
    let position: [f32; 2] = [1.8, 1.8];
    assert!(!circle_obstacle.is_inside(&position));
}
