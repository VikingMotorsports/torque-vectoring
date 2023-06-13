use plotters::prelude::*;

const OUT_FILE_NAME: &'static str = "yawrates.png";

pub fn yaw(current: Vec<f32>, desired: Vec<f32>) -> Result<(), Box<dyn std::error::Error>>
{
    let root = BitMapBackend::new(OUT_FILE_NAME, (1024, 768)).into_drawing_area();
    root.fill(&WHITE)?;

    let mut chart = ChartBuilder::on(&root)
        .x_label_area_size(35)
        .y_label_area_size(45)
        .margin(5)
        .caption("Yaw Rates", ("sans-serif", 50.0).into_font())
        .build_cartesian_2d(0f32..10f32, 0_f32..100_f32)?
        .set_secondary_coord(0f32..10f32, 0_f32..100_f32);

    chart
        .configure_mesh()
        .disable_x_mesh()
        .disable_y_mesh()
        .x_desc("Time (seconds)")
        .y_desc("Yaw Rate (radians/s)")
        .draw()?;

    chart
        .draw_series(LineSeries::new((0..=current.len() - 1).map(|x| (x as f32, current[x as usize])), &BLUE,))?
        .label("Actual Yaw Rate")
        .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], &BLUE));

    chart
        .draw_secondary_series(LineSeries::new((0..=desired.len() - 1).map(|x| (x as f32, desired[x as usize])), &RED,))?
        .label("Desired Yaw Rate")
        .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], &RED));

    chart
        .configure_series_labels()
        .background_style(&RGBColor(128, 128, 128))
        .draw()?;

    // To avoid the IO failure being ignored silently, we manually call the present function
    root.present().expect("Unable to write result to file");
    println!("Result has been saved to {}", OUT_FILE_NAME);

    Ok(())
}
