use plotters::prelude::*;

pub fn yaw(current: Vec<f32>, desired: Vec<f32>) -> Result<(), Box<dyn std::error::Error>>
{
    const OUT_FILE_NAME: &'static str = "yawrates.png";
    let root = BitMapBackend::new(OUT_FILE_NAME, (1024, 768)).into_drawing_area();
    root.fill(&WHITE)?;

    let cur_min = bounds(current.clone(), true);
    let des_min = bounds(desired.clone(), true);
    let cur_max = bounds(current.clone(), false);
    let des_max = bounds(desired.clone(), false);
    let cur_len = current.len() - 1;
    let des_len = desired.len() - 1;

    let min: f32 = min_bounds(cur_min, des_min);
    let max: f32 = max_bounds(cur_max, des_max);
    let time: f32 = max_bounds(cur_len as f32, des_len as f32);

    let mut chart = ChartBuilder::on(&root)
        .x_label_area_size(35)
        .y_label_area_size(45)
        .margin(5)
        .caption("Yaw Rates", ("sans-serif", 50.0).into_font())
        .build_cartesian_2d(0f32..time, min..max)?
        .set_secondary_coord(0f32..time, min..max);

    chart
        .configure_mesh()
        .disable_x_mesh()
        .disable_y_mesh()
        .x_desc("Time (seconds)")
        .y_desc("Yaw Rate (radians/s)")
        .draw()?;

    chart
        .draw_series(LineSeries::new((0 ..= cur_len)
        .map(|x| (x as f32, current[x as usize])), &BLUE,))?
        .label("Actual Yaw Rate")
        .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], &BLUE));

    chart
        .draw_secondary_series(LineSeries::new((0 ..= des_len)
        .map(|x| (x as f32, desired[x as usize])), &RED,))?
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

fn min_bounds(cur: f32, des: f32) -> f32
{
    // determine minimum value for graph bounds
    if cur < des 
    { 
        return cur; 
    }
    else 
    { 
        return des; 
    }
}

fn max_bounds(cur: f32, des: f32) -> f32
{
    // determine maximum value for graph bounds
    if cur > des 
    { 
        return cur; 
    }
    else 
    { 
        return des; 
    }
}

fn bounds(v: Vec<f32>, min: bool) -> f32
{
    // language doesn't seem to have a built-in solution for this with f32 vectors
    let mut val: f32 = v[0];

    if min
    {
        for i in &v
        {
            if val > *i
            {
                val = *i
            }
        }
    }
    else
    {
        for i in &v
        {
            if val < *i
            {
                val = *i
            }
        }
    }
    val
}
