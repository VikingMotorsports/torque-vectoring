use std::f32::consts;

// Gear ratio
const GR: f32 = 4.1;

// Wheel radius in meters
const WHEEL_RADIUS: f32 = 0.4;

// Inertia Moment around z axis
const IZZ: f32 = 120.0;

// Half track of rear axle in meters
const TR: f32 = 0.6604;

// Radius of wheel
const RW: f32 = 0.2286;

// lf (float): Distance between the center of gravity and the front axle in meters
const LF: f32 = 0.885;

// lr (float): Distance between the center of gravity and the rear axle in meters
const LR: f32 = 0.770;

// m (float): Mass of the vehicle in kilograms
const M: f32 = 222.26;

// Cy_f (float): Cornering stiffness of the front tires in N/rad
const CY_F: f32 = 7085.0;

// Cy_r (float): Cornering stiffness of the rear tires in N/rad
const CY_R: f32 = 7302.0;

const WEIGHT: f32 = M * 9.81;

const B: f32 = 10.0;

const C: f32 = 1.9;
const D: f32 = 1.0; 
const E: f32 = 0.97;

fn radians_to_ms(radians: f32) -> f32 {
    return radians * WHEEL_RADIUS;
}

fn degrees_to_radians(degrees: f32) -> f32 {
    return degrees * (consts::PI / 180.0);
}

fn calculate_desired_yaw_rate(v_cg: f32, delta: f32) -> f32 {
    let des_yaw_rate: f32 = (v_cg / ((LF + LR) + (0.0012965 * v_cg.powf(2.0)))) * delta;
    des_yaw_rate
}

fn calculate_yaw_rate(t: f32, mut ku: f32, vx: f32, st_a: f32, trr: f32, trl: f32) -> f32 {
    ku += ((((-LF * CY_F + LR * CY_R) / (IZZ * vx)) - ((LF.powf(2.0) * CY_F + LR.powf(2.0) * CY_R) / (IZZ * vx))) * ku + ((LF * CY_F) / IZZ) * st_a + (1.0 / (0.05 * IZZ)) * calculate_delta_torque(trr, trl)) / t;
    ku
}

fn calculate_yaw_moment(trr: f32, trl: f32) -> f32 {
    let m_v: f32 = (trr - trl) * TR;
    m_v
}

fn calculate_lateral_velocity(vx: f32, st_a: f32) -> f32 {
    let vy: f32 = ((-(CY_F + CY_R)/(M * vx)) + (((-LF * CY_F + LR + CY_R)/(M * vx)) - vx)) + (CY_F / (M * vx)) * st_a;
    vy
}

fn calculate_delta_torque(trr: f32, trl: f32) -> f32 {
    let delt_t: f32 = (RW / (2.0 * TR * GR)) * calculate_yaw_moment(trr, trl);
    delt_t
}

fn magic_formula(w: f32, mut p: f32) -> f32 {

    p = p - 1.0;
    let f: f32 = w * D * f32::sin(C * f32::atan(B * p * (1.0 - E) + E * B * p - f32::atan(B * p)));
    f
}

fn steering_wheel_angle_to_steering_angle(steering_wheel_angle: f32) -> f32 {
    let mut x: f32 = 8.355 * 10_f32.powf(-5.0) * steering_wheel_angle.powi(2) + 0.139 * steering_wheel_angle - 0.03133;
    x = degrees_to_radians(x);
    x
}

pub fn simulate(mut v_cg: f32, w_velocity: f32, rl_torque_wheel: f32, rr_torque_wheel: f32, mut steering_a: f32) -> (Vec<f32>, Vec<f32>, Vec<f32>, Vec<f32>, Vec<f32>) {
    steering_a = steering_wheel_angle_to_steering_angle(steering_a);
    let mut a_x: Vec<f32> = Vec::new();
    a_x.push(v_cg);
    let mut a_y: Vec<f32> = Vec::new();
    a_y.push(0.0);
    let mut wheel_velocity: Vec<f32> = Vec::new();
    wheel_velocity.push(w_velocity);
    let mut curr_yaw_rate: Vec<f32> = Vec::new();
    curr_yaw_rate.push(0.0);
    let mut des_yaw_rate: Vec<f32> = Vec::new();
    des_yaw_rate.push(calculate_desired_yaw_rate(v_cg, steering_a));

    let mut rl_wheel_angular_accel: f32;
    let mut rr_wheel_angular_accel: f32;
    let mut rl_wheel_angular_v: f32;
    let mut rr_wheel_angular_v: f32;
    let mut rl_slippage: f32;
    let mut rr_slippage: f32;
    let mut f: f32;
    let mut long_accel: f32;

    for i in 0..1000 {
        rl_wheel_angular_accel = radians_to_ms((2.0 *rl_torque_wheel) / (9.0  * WHEEL_RADIUS));
        rr_wheel_angular_accel = radians_to_ms((2.0 *rr_torque_wheel) / (9.0  * WHEEL_RADIUS));
        rl_slippage = (v_cg + rl_wheel_angular_accel) / v_cg;
        rr_slippage = (v_cg + rr_wheel_angular_accel) / v_cg;
        if rl_slippage > 1.0 {
            rl_slippage = 1.0;
        }
        if rr_slippage > 1.0 {
            rr_slippage = 1.0;
        }
        f = magic_formula(WEIGHT / 2.0, (rl_slippage + rr_slippage) / 2.0);
        long_accel = (f / M) / 10.0;
        v_cg += long_accel;
        rl_wheel_angular_v = rl_slippage * v_cg;
        rr_wheel_angular_v = rr_slippage * v_cg;
        a_x.push(a_x[i] + long_accel);
        wheel_velocity.push((rl_wheel_angular_v + rr_wheel_angular_v) / 2.0);
        a_y.push(calculate_lateral_velocity(v_cg, steering_a));
        curr_yaw_rate.push(calculate_yaw_rate(i as f32, curr_yaw_rate[i], v_cg, steering_a, rr_torque_wheel, rl_torque_wheel));
        des_yaw_rate.push(calculate_desired_yaw_rate(v_cg, steering_a));
        // still working on code
    }
    (a_x, a_y, wheel_velocity, curr_yaw_rate, des_yaw_rate)
    // return vector
}