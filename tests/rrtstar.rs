use path_planning::planner::RRTStar;

#[test]
fn test_plan() {
    let low: [f32; 2] = [0.0, 0.0];
    let high: [f32; 2] = [50.0, 30.0];
    let start: [f32; 2] = [1.0, 1.0];
    let goal: [f32; 2] = [48.0, 25.0];
    let is_approved = Box::new(|_position: &[f32; 2]| true);
    let goal_sample_rate = 0.2;
    let step_size = 2.0;
    let max_iter = 2000;

    let mut rrt = RRTStar::new(
        start,
        goal,
        low,
        high,
        is_approved,
        goal_sample_rate,
        step_size,
        max_iter,
    );
    let path = rrt.plan();

    assert!(path.len() > 0);
    assert_eq!(path[0], start);
    assert_eq!(path[path.len() - 1], goal);
}
