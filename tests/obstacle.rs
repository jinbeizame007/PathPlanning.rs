use path_planning as pp;
use path_planning::env::Obstacle;

#[test]
fn test_rect_obstacle() {
    let center: [f32; 2] = [0.0, 0.0];
    let size: [f32; 2] = [2.0, 2.0];
    let rect_obstacle = pp::env::RectObstacle{center, size};
    assert_eq!(rect_obstacle.center[0], 0.0);

    let position: [f32; 2] = [0.5,0.5];
    assert!(rect_obstacle.is_inside(&position));

    let position: [f32; 2] = [2.1,2.1];
    assert!(!rect_obstacle.is_inside(&position));
}
