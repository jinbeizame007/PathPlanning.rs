use crate::env::Env;
use crate::env::Obstacle;
use plotters::prelude::*;
use plotters::coord::types::RangedCoordf32;

const MARGIN: i32 = 20;
const X_LABEL_AREA_SIZE: i32 = 30;
const Y_LABEL_AREA_SIZE: i32 = 30;

pub fn draw_env(chart: &mut ChartContext<BitMapBackend, Cartesian2d<RangedCoordf32, RangedCoordf32>>, env: &Env<2>) {
    for i in 0..env.obstacles.len() {
        let obs = &env.obstacles[i];
        match obs {
            Obstacle::RectObstacle { center, size } => {
                let left_upper_corner = (center[0] - size[0] / 2.0, center[1] - size[1] / 2.0);
                let right_lower_corner = (center[0] + size[0] / 2.0, center[1] + size[1] / 2.0);
                let rect = Rectangle::new(
                    [left_upper_corner, right_lower_corner],
                    *&Palette99::pick(i).filled(),
                );
                chart.draw_series([rect]).unwrap();
            }
            Obstacle::CircleObstacle { center, radius } => {
                let circle = Circle::new(
                    (center[0], center[1]),
                    radius * 10.0,
                    *&Palette99::pick(i).filled(),
                );
                chart.draw_series([circle]).unwrap();
            }
        };
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
