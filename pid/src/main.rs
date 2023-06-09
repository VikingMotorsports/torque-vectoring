mod calc;

fn main() 
{
    let mut a_x: Vec<f32> = Vec::new();
    let mut a_y: Vec<f32> = Vec::new();
    let mut wheel_velocity: Vec<f32> = Vec::new();
    let mut curr_yaw_rate: Vec<f32> = Vec::new();
    let mut des_yaw_rate: Vec<f32> = Vec::new();

    (a_x, a_y, wheel_velocity, curr_yaw_rate, des_yaw_rate) = calc::simulate(1.0, 1.0, 1.0, 1.0, -20.0);

    let mut i = 1;
    let mut time: Vec<usize> = Vec::new();
    time.push(0);
    let mut total_slip: Vec<f32> = Vec::new();
    total_slip.push(((wheel_velocity[i] / a_x[i]) - 1.0) * 100.0);

    while i < a_x.len() {
        time.push(i / 100);
        total_slip.push(((wheel_velocity[i] / a_x[i]) - 1.0) * 100.0);
        i += 1;
    }

    // Display Graphs
}
