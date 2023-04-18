import math
import random
import array as arr
from graph import display_graphs

# Used Terms:
"""
Yaw Rate: rad/s
Longitudinal Velocity: Vx, m/s
Cornering stiffness at rear wheel: Cy_r, N/rad
Cornering stiffness at front wheel: Cy_f, N/rad
Inertia Moment: Izz, Kg/m^2
Mass: m, Kg
Front wheelbase: lf, m
Rear wheelbase: lr, m
Steering Angle, St_a, degrees (Convert from radians)
Yaw Moment: M_V, Nm
Yaw Moment around vertical axis: Mz
Lateral Velocity: Vy, m/s
Gear Ratio: Gr, -
Half the track of the car: Tr, m
Radius of the wheel: Rw, m (0.2286)
"""

# radians/s (wheel speed)
wheelAngularVelocity = 0 

# radians/s^2 (wheel torque basically)
wheelAngularAccel = 28.2744 

# max angular acceleration in radians/s^2
maxAngularAccel = 1 

# Wheel radius in meters
wheelRadius = 0.4

#Slippage in percentage (1.0 as 0%)
slippage = 1.0

# Gear ratio
Gr = 4.1

# Inertia Moment around z axis
Izz = 120

# Half track of rear axle in meters
Tr = 0.6604 

# Radius of wheel
Rw = 0.2286

# lf (float): Distance between the center of gravity and the front axle in meters
lf = 0.885

# lr (float): Distance between the center of gravity and the rear axle in meters
lr = 0.770

# m (float): Mass of the vehicle in kilograms
m = 222.26

# Cy_f (float): Cornering stiffness of the front tires in N/rad
Cy_f = 7085

# Cy_r (float): Cornering stiffness of the rear tires in N/rad
Cy_r = 7302

# v_cg (float): Velocity of the center of gravity in m/s
v_cg = 1

weight = m * 9.81

# PID Controller class for controlling the car's yaw rate
class PIDController:
    def __init__(self, Kp, Ki, Kd, dt):
        self.Kp = Kp  # Proportional gain
        self.Ki = Ki  # Integral gain
        self.Kd = Kd  # Derivative gain
        self.dt = dt  # Time step
        self.previous_error = 0  # Previous error for calculating derivative term
        self.integral = 0  # Integral of errors for calculating integral term

    # Control method for calculating the control output (throttle adjustment)
    def control(self, error):
        """
        Calculate the control output based on the PID controller.

        Parameters:
        error (float): The difference between the desired and current values.

        Returns:
        float: The control output.
        """
        self
        self.integral += error * self.dt  # Calculate integral term
        derivative = (error - self.previous_error) / self.dt  # Calculate derivative term
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative  # Calculate control output
        self.previous_error = error  # Store current error as previous error for next iteration
        return output  # Return control output
    
def radians_to_ms(radians):
    ms = radians * wheelRadius
    return ms

# Function for converting degrees to radians
def degrees_to_radians(degrees):
    """
        Converts an angle from degrees to radians.

        Parameters:
            degrees (float): Angle in degrees.

        Returns:
            float: Angle in radians.
    """
    return degrees * (math.pi / 180)

def calculate_steer_gradient():
    """
    Calculates the characteristics of the steering gradient
    of the car.

    Args:
   
    Returns:
    float: Value that describes whether or not the car understeers
    or oversteers.
    """
    Ku = ((lr * m) / (Cy_f * (lf + lr))) - ((lf * m) / (Cy_r * (lf + lr)))

    return Ku

def calculate_desired_yaw_rate(v_cg, delta):
    """
    Calculates the desired yaw rate of a vehicle given the velocity of the center
    of gravity and the steering angle.

    Args:
    v_cg (float): Velocity of the center of gravity in m/s
    delta (float): Steering angle in radians in rad

    Returns:
    float: Desired yaw rate of the vehicle in radians per second
    """
    # Calculate Ku using the existing function
    # Ku = calculate_yaw_rate(Cy_f, Cy_r)

    # Calculate the desired yaw rate using the given formula
    desired_yaw_rate = (v_cg / (lf + lr) - 0.00168 * math.pow(v_cg, 2)) * delta

    # Return the calculated desired yaw rate
    return desired_yaw_rate

def calculate_yaw_rate(Cy_f, Cy_r, Vx, St_a, Trr, Trl):
    """
    Calculates the yaw rate of a vehicle given the distance between the center
    of gravity and the front and rear axles, the mass of the vehicle, and the
    cornering stiffness of the front and rear tires.

    Args:
    Cy_f (float): Cornering stiffness of the front tires in N/rad
    Cy_r (float): Cornering stiffness of the rear tires in N/rad
    Vx, (float): Current velocity of car in m/s
    St_a, (float): Current steering angle in radians
    Trr, (float): Current power from rear right wheel in Nm
    Trl, (float): Current power from rear left wheel in Nm

    Returns:
    float: Yaw rate of the vehicle in radians per second
    """
    # Calculate the yaw rate using the given formula
    Ku = ((-lf * Cy_f + lr * Cy_r) / (Izz * Vx)) - ((lf**2 * Cy_f + lr**2 * Cy_r) / (Izz * Vx)) + ((lf * Cy_f) / Izz) * St_a + (1 / 0.05 * Izz) * calculate_delta_torque(Trr, Trl)

    # Return the calculated yaw rate
    return Ku

def calculate_yaw_moment(Trr, Trl):
    """
    Calculates the momentum of how the power ratio of each wheel
    influences the yaw rate of the car

    Args:
    Trr, (float): Current power from rear right wheel in Nm
    Trl, (float): Current power from rear left wheel in Nm

    Returns:
    float: Yaw moment ratio
    """
    # Calculate the yaw moment using the given formula
    M_V = (Trr - Trl) * Tr

    # Return the calculated yaw moment
    return M_V

def calculate_lateral_velocity(Vx, St_a):
    """
    Calculates the lateral velocity of the car during turns.

    Args:
    Vx, (float): Current velocity of the car in m/s
    St_a (float): Steering angle in radians

    Returns:
    float: Lateral velocity (also known as lateral force)
    """
    # Calculate the lateral velocity using the given formula
    Vy = ((-(Cy_f + Cy_r)/(m * Vx)) + (((-lf * Cy_f + lr * Cy_r)/(m * Vx)) - Vx)) + (Cy_f / (m * Vx)) * St_a

    # Return the calculated lateral velocity
    return Vy

def calculate_delta_torque(Trr, Trl):
    """
    Calculates the difference in torque between each wheel

    Args:
    Trr, (float): Current power from rear right wheel in Nm
    Trl, (float): Current power from rear left wheel in Nm

    Returns:
    float: Difference in torque in Nm
    """
    # Calculate the delta torque using the given formula
    delt_T = (Rw / ((2 * Tr) * Gr)) * calculate_yaw_moment(Trr, Trl)

    #return delt_T
    return delt_T

# Function for calculating the throttle adjustment required to reach the desired yaw rate
def calculate_throttle_adjustment(pid_controller, current_yaw_rate, desired_yaw_rate):
    """
    Calculates the throttle adjustment required to reach the desired yaw rate.

    Parameters:
        pid_controller (PIDController): A PIDController object for controlling the car's yaw rate.
        current_yaw_rate (float): The car's current yaw rate.
        desired_yaw_rate (float): The desired yaw rate.

    Returns:
        float: Throttle adjustment required to reach the desired yaw rate.
    """
    error = desired_yaw_rate - current_yaw_rate  # Calculate error
    throttle_adjustment = pid_controller.control(error)  # Calculate throttle adjustment using PID controller
    return throttle_adjustment  # Return throttle adjustment required to reach the desired yaw rate

def magic_formula(w, p, b=10, c=1.9, d=1, e=.97):
    """

    The magic formula [1] computes the longitudinal force between wheel and tarmac
    in function of the longitudinal tire slip.

    [1] Pacejka, H. B. Tire and Vehicle Dynamics. Elsevier Science, 2005

    Parameters
    ----------
    num : int, optional
        Number of samples to generate. Default is 50. Must be non-negative.
    p : float
        Weight applied on the tire [N].
    b : float
        Stiffness factor. Default value corresponds to dry tarmac.
    c : float
        Shape factor. Default value corresponds to dry tarmac.
    d : float
        Peak factor. Default value corresponds to dry tarmac.
    e : float
        Curvature factor. Default value corresponds to dry tarmac.

    Returns
    -------
    F : float or array
        If x is a slip ratio, Func is the longitudinal force between tire and tarmac [N], if x is a wheel slip angle, Func
        is the lateral force.
    alpha : float, array_like
        Tire slip ratio or slip angle.
    """
    p = p - 1
    F = w * d * math.sin(c * math.atan(b * p * (1 - e) + e * b * p - math.atan(b * p)))
    return F

def simulate(v_cg, w_Velocity, rl_torqueWheel, rr_torqueWheel, steering_a):
    """
    Simulate the vehicle dynamics.

    Parameters:
    v_cg (float): The center of gravity velocity in m/s.
    w_Velocity (float): Total rear longitudinal velocity of wheels in m/s.
    rl_torqueWheel (float): Rear left wheel torque in Nm.
    rr_torqueWheel (float): Rear right wheel torque in Nm.
    steering_a (float): The steering angle in radians.

    Returns:
    tuple: A tuple containing velocity (m/s), lateral velocity (m/s), wheel velocity (m/s), and yaw rate (radians/s)
    """
    a_x = arr.array('f', [v_cg])
    a_y = arr.array('f', [0])
    wheel_velocity = arr.array('f', [w_Velocity])
    curr_yaw_rate = arr.array('f', [calculate_yaw_rate(Cy_f, Cy_r, v_cg, steering_a, rr_torqueWheel, rl_torqueWheel)])
    des_yaw_rate = arr.array('f', [calculate_desired_yaw_rate(v_cg, steering_a)])
    rl_wheelAngularAccel = radians_to_ms((2*rl_torqueWheel) / (9 * wheelRadius))
    rr_wheelAngularAccel = radians_to_ms((2*rr_torqueWheel) / (9 * wheelRadius))
    rl_slippage = 1.0
    rr_slippage = 1.0
    i = 0
    while i < 1000:
            
        rl_wheelAngularAccel = radians_to_ms((2*rl_torqueWheel) / (9 * wheelRadius))
        rr_wheelAngularAccel = radians_to_ms((2*rr_torqueWheel) / (9 * wheelRadius))
        rl_slippage = (v_cg + rl_wheelAngularAccel) / v_cg
        rr_slippage = (v_cg + rr_wheelAngularAccel) / v_cg
        if rl_slippage < 1:
            rl_slippage = 1.0
        if rr_slippage < 1:
            rr_slippage = 1.0
        F = magic_formula(weight / 2, (rl_slippage + rr_slippage) / 2)
        LongAccel = (F / m) / 10
        v_cg = v_cg + LongAccel
        rl_wheelAngularVelocity = rl_slippage * v_cg
        rr_wheelAngularVelocity = rr_slippage * v_cg
        a_x.append(a_x[i] + LongAccel)
        wheel_velocity.append((rl_wheelAngularVelocity + rr_wheelAngularVelocity) / 2)
        a_y.append(calculate_lateral_velocity(v_cg, steering_a))
        curr_yaw_rate.append(calculate_yaw_rate(Cy_f, Cy_r, v_cg, steering_a, rr_torqueWheel, rl_torqueWheel))
        des_yaw_rate.append(calculate_desired_yaw_rate(v_cg, steering_a))
        i = i + 1

    return a_x, a_y, wheel_velocity, curr_yaw_rate, des_yaw_rate

if __name__ == '__main__':
    
    # Simulate vehicle dynamics with given velocities, torque applied to each wheel, and steering wheel angle
    ax, ay, wheel_velocity, yaw_rate, des_rate = simulate(1, 1, 4.75, 5, 0.4)

    # desired_yaw_rate = calculate_desired_yaw_rate(random_velocity, random_steering_angle)

    # Calculate time
    i = 1
    time = arr.array('f', [0])
    total_slip = arr.array('f', [((wheel_velocity[i] / ax[i]) - 1) * 100])
    while i < len(ax):
        time.append(i / 10)
        total_slip.append(((wheel_velocity[i] / ax[i]) - 1) * 100)
        i += 1
    
    # Display results
    display_graphs(time, ax, ay, wheel_velocity, total_slip, yaw_rate, des_rate)
    """
    # Print the simulation results
    print(f"Random steering angle (degrees): {random_steering_angle}")
    print(f"Random velocity (m/s): {random_velocity}")
    print(f"Longitudinal acceleration (m/s^2): {ax}")
    print(f"Lateral acceleration (m/s^2): {ay}")
    print(f"Yaw rate (rad/s): {yaw_rate}")
    print(f"Desired Yaw rate (rad/s): {desired_yaw_rate}")

    # Define PID controller constants
    Kp = 2
    Ki = 0.3
    Kd = 20
    dt = 0.2

    # Instantiate PID controller
    pid_controller = PIDController(Kp, Ki, Kd, dt)

    # Calculate throttle adjustment based on yaw rate error
    throttle_adjustment = calculate_throttle_adjustment(pid_controller, yaw_rate, desired_yaw_rate)
    print(f"Throttle adjustment: {throttle_adjustment}")

    # Define initial throttle percentages for left and right wheels
    left_wheel_throttle = 50
    right_wheel_throttle = 50

    # Adjust throttle percentages based on calculated throttle adjustment
    left_wheel_throttle += throttle_adjustment / 2
    right_wheel_throttle -= throttle_adjustment / 2

    # Print the final throttle percentages for left and right wheels
    print(f"Left wheel throttle percentage: {left_wheel_throttle}")
    print(f"Right wheel throttle percentage: {right_wheel_throttle}")
    """