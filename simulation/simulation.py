import math

# lf (float): Distance between the center of gravity and the front axle in meters
lf = 0.717

# lr (float): Distance between the center of gravity and the rear axle in meters
lr = 2.5

# m (float): Mass of the vehicle in kilograms
m = 356

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

Cy_f = 15714
Cy_r = 21429
v_cg = 20
delta = math.radians(10)

desired_yaw_rate = calculate_desired_yaw_rate(v_cg, delta)

print("Desired yaw rate:", desired_yaw_rate, "rad/s")