use crate::env::Env;
use crate::env::Obstacle;
use crate::planner::Node;
use plotters::coord::types::RangedCoordf32;
use plotters::prelude::*;

const MARGIN: i32 = 20;
const X_LABEL_AREA_SIZE: i32 = 30;
const Y_LABEL_AREA_SIZE: i32 = 30;

fn draw_env(
    chart: &mut ChartContext<BitMapBackend, Cartesian2d<RangedCoordf32, RangedCoordf32>>,
    env: &Env<2>,
) {
    for (i, obs) in env.obstacles.iter().enumerate() {
        match obs {
            Obstacle::RectObstacle { center, size } => {
                let left_upper_corner = (center[0] - size[0] / 2.0, center[1] - size[1] / 2.0);
                let right_lower_corner = (center[0] + size[0] / 2.0, center[1] + size[1] / 2.0);
                let rect = Rectangle::new(
                    [left_upper_corner, right_lower_corner],
                    Palette99::pick(i + 1).filled(),
                );
                chart.draw_series([rect]).unwrap();
            }
            Obstacle::CircleObstacle { center, radius } => {
                let circle = Circle::new(
                    (center[0], center[1]),
                    radius * 10.0,
                    Palette99::pick(i + 1).filled(),
                );
                chart.draw_series([circle]).unwrap();
            }
        };
    }
}

fn draw_path(
    chart: &mut ChartContext<BitMapBackend, Cartesian2d<RangedCoordf32, RangedCoordf32>>,
    path: &Vec<[f32; 2]>,
) {
    let path_tuple: Vec<(f32, f32)> = (0..path.len()).map(|i| (path[i][0], path[i][1])).collect();
    chart
        .draw_series(LineSeries::new(path_tuple.clone(), &RED))
        .unwrap();

    for i in 0..path.len() {
        let circle = Circle::new(path_tuple[i].clone(), 3.0, Palette99::pick(0).filled());
        chart.draw_series([circle]).unwrap();
    }
}

fn draw_all_paths(
    chart: &mut ChartContext<BitMapBackend, Cartesian2d<RangedCoordf32, RangedCoordf32>>,
    nodes: &Vec<Node<2>>,
) {
    let circles = nodes.iter().map(|node| Circle::new((node.position[0], node.position[1]), 3.0, Palette99::pick(0).filled()));
    chart.draw_series(circles).unwrap();

    for node in nodes.iter() {
        match node.parent {
            Some(parent_index) => {
                let parent_position = &nodes[parent_index].position;
                let line = [
                    (node.position[0], node.position[1]),
                    (parent_position[0], parent_position[1]),
                ];
                chart
                    .draw_series(LineSeries::new(line, &Palette99::pick(0)))
                    .unwrap();
            }
            None => {}
        }
    }
}

pub fn plot_env(env: &Env<2>) -> Result<(), Box<dyn std::error::Error>> {
    let root = BitMapBackend::new("env.png", (1000, 600)).into_drawing_area();
    root.fill(&WHITE)?;

    let mut chart = ChartBuilder::on(&root)
        .margin(MARGIN)
        .x_label_area_size(X_LABEL_AREA_SIZE)
        .y_label_area_size(Y_LABEL_AREA_SIZE)
        .build_cartesian_2d(env.low[0]..env.high[0], env.low[1]..env.high[1])
        .unwrap();
    chart.configure_mesh().draw().unwrap();

    draw_env(&mut chart, &env);

    root.present()?;
    Ok(())
}

pub fn plot_path(env: &Env<2>, path: &Vec<[f32; 2]>) -> Result<(), Box<dyn std::error::Error>> {
    let root = BitMapBackend::new("path.png", (1000, 600)).into_drawing_area();
    root.fill(&WHITE)?;

    let mut chart = ChartBuilder::on(&root)
        .margin(MARGIN)
        .x_label_area_size(X_LABEL_AREA_SIZE)
        .y_label_area_size(Y_LABEL_AREA_SIZE)
        .build_cartesian_2d(env.low[0]..env.high[0], env.low[1]..env.high[1])
        .unwrap();
    chart.configure_mesh().draw().unwrap();

    draw_env(&mut chart, &env);
    draw_path(&mut chart, &path);

    root.present()?;
    Ok(())
}

pub fn animate_path(
    env: &Env<2>,
    log: &Vec<Vec<Node<2>>>,
) -> Result<(), Box<dyn std::error::Error>> {
    let root = BitMapBackend::gif("log.gif", (1000, 600), 50)?.into_drawing_area();
    let mut chart = ChartBuilder::on(&root)
        .margin(MARGIN)
        .x_label_area_size(X_LABEL_AREA_SIZE)
        .y_label_area_size(Y_LABEL_AREA_SIZE)
        .build_cartesian_2d(env.low[0]..env.high[0], env.low[1]..env.high[1])
        .unwrap();

    for nodes in log.iter() {
        root.fill(&WHITE)?;
        chart.configure_mesh().draw().unwrap();

        draw_env(&mut chart, &env);
        draw_all_paths(&mut chart, &nodes);

        root.present()?;
    }
    Ok(())
}
