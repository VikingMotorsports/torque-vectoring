import math
import random

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
    Ku = calculate_yaw_rate(Cy_f, Cy_r)

    # Calculate the desired yaw rate using the given formula
    desired_yaw_rate = v_cg / ((lf + lr) + Ku * v_cg**2) * delta

    # Return the calculated desired yaw rate
    return desired_yaw_rate

def calculate_yaw_rate(Cy_f, Cy_r):
    """
    Calculates the yaw rate of a vehicle given the distance between the center
    of gravity and the front and rear axles, the mass of the vehicle, and the
    cornering stiffness of the front and rear tires.

    Args:
    Cy_f (float): Cornering stiffness of the front tires in N/rad
    Cy_r (float): Cornering stiffness of the rear tires in N/rad

    Returns:
    float: Yaw rate of the vehicle in radians per second
    """
    # Calculate the yaw rate using the given formula
    Ku = (lr * m) / (Cy_f * (lf + lr)) - (lf * m) / (Cy_r * (lf + lr))

    # Return the calculated yaw rate
    return Ku

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
def simulate(v_cg, delta_degrees):
    """
    Simulate the vehicle dynamics.

    Parameters:
    v_cg (float): The center of gravity velocity.
    delta_degrees (float): The steering angle in degrees.

    Returns:
    tuple: A tuple containing longitudinal acceleration (m/s^2), lateral acceleration (m/s^2), and yaw rate (rad/s).
    """
    delta = degrees_to_radians(delta_degrees)
    yaw_rate = calculate_yaw_rate(Cy_f, Cy_r)
    ay = v_cg * yaw_rate
    ax = v_cg ** 2 / (lf + lr) * delta
    return ax, ay, yaw_rate

if __name__ == '__main__':
    # Generate random steering angle and velocity for simulation
    random_steering_angle = random.uniform(-230, 230)
    random_velocity = random.uniform(0, 50)

    # Simulate vehicle dynamics with given steering angle and velocity
    ax, ay, yaw_rate = simulate(random_velocity, random_steering_angle)
    desired_yaw_rate = calculate_desired_yaw_rate(random_velocity, random_steering_angle)

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


