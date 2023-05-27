use std::f32::consts;

// Wheel radius in meters
const wheelRadius: f32 = 0.4;

// Inertia Moment around z axis
const Izz: f32 = 120.0;

// Half track of rear axle in meters
const Tr: f32 = 0.6604;

// Radius of wheel
const Rw: f32 = 0.2286;

// lf (float): Distance between the center of gravity and the front axle in meters
const lf: f32 = 0.885;

// lr (float): Distance between the center of gravity and the rear axle in meters
const lr: f32 = 0.770;

// m (float): Mass of the vehicle in kilograms
const m: f32 = 222.26;

// Cy_f (float): Cornering stiffness of the front tires in N/rad
const Cy_f: f32 = 7085.0;

// Cy_r (float): Cornering stiffness of the rear tires in N/rad
const Cy_r: f32 = 7302.0;

const b: f32 = 10.0;

const c: f32 = 1.9;
const d: f32 = 1.0; 
const e: f32 = 0.97;

fn radians_to_ms(radians: f32) -> f32 {
    return radians * wheelRadius;
}

fn degrees_to_radians(degrees: f32) -> f32 {
    return degrees * (consts::PI / 180.0);
}

fn calculate_desired_yaw_rate(vel: f32, delta: f32) -> f32 {
    let mut des_yaw_rate: f32 = 0.0;
    des_yaw_rate = (vel / (lf + lr) - 0.00168 * vel.powi(2) * delta);
    des_yaw_rate
}

fn calculate_yaw_rate(Vx: f32, st_a: f32, Trr: f32, Trl: f32) -> f32 {
    let mut Ku: f32 = 0.0;
    Ku = ((-lf * Cy_f + lr * Cy_r) / (Izz * Vx));
    Ku
}

fn calculate_yaw_moment(Trr: f32, Trl: f32) -> f32 {
    let m_v: f32 = (Trr - Trl) * Tr;
    m_v
}

fn calculate_lateral_velocity(Vx: f32, St_a: f32) -> f32 {
    let vy: f32 = ((-(Cy_f + Cy_r)/(m * Vx)) + (((-lf * Cy_f + lr + Cy_r)/(m * Vx)) -Vx)) + (Cy_f / (m * Vx)) * St_a;
    vy
}

fn magic_formula(w: f32, mut p: f32) -> f32 {

    p = p - 1.0;
    let f: f32 = w * d * f32::sin(c * f32::atan(b * p * (1.0 - e) + e * b * p - f32::atan(b * p)));
    f
}

fn steering_wheel_angle_to_steering_angle(steering_wheel_angle: f32) -> f32 {
    let x: f32 = (8.355 * 10_f32.powf(-5.0) * steering_wheel_angle.powi(2) + 0.139 * steering_wheel_angle - 0.03133) * consts::PI/180.0;
    x
}

fn simulate(v_cg: f32, w_velocity: f32, rl_torqueWheel: f32, rr_torqueWheel: f32, steering_a: f32) -> f32 {
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

    let mut rl_wheelAngularAccel: f32 = 0.0;
    let mut rr_wheelAngularAccel: f32 = 0.0;
    let mut rl_slippage: f32 = 0.0;
    let mut rr_slippage: f32 = 0.0;

    for i in 0..1000 {
        rl_wheelAngularAccel = radians_to_ms((2.0 *rl_torqueWheel) / (9.0  * wheelRadius));
        rr_wheelAngularAccel = radians_to_ms((2.0 *rr_torqueWheel) / (9.0  * wheelRadius));
        rl_slippage = (v_cg + rl_wheelAngularAccel) / v_cg;
        rr_slippage = (v_cg + rr_wheelAngularAccel) / v_cg;
        if rl_slippage > 1.0 {
            rl_slippage = 1.0;
        }
        if rr_slippage > 1.0 {
            rr_slippage = 1.0;
        }
        // still working on code
    }
    // return vector
}