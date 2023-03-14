import math
import matplotlib.pyplot as plt

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
v_cg = 100

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


if __name__ == '__main__':
    # Calculate the desired yaw rate for each degree in the range of -230 to 230
    delta_deg = range(-230, 231)
    delta_rad = [math.radians(deg) for deg in delta_deg]
    desired_yaw_rate = [calculate_desired_yaw_rate(v_cg, delta) for delta in delta_rad]

    # Identify the range of angles where the desired yaw rate is between -0.5 and 0.5 rad/s
    low_threshold = -0.5
    high_threshold = 0.5
    close_to_zero = [(delta_deg[i], delta_deg[i + 1]) for i in range(len(desired_yaw_rate) - 1)
                     if low_threshold < desired_yaw_rate[i] < high_threshold and
                     low_threshold < desired_yaw_rate[i + 1] < high_threshold]

    # Plot the desired yaw rate as a function of the steering angle
    plt.plot(delta_deg, desired_yaw_rate)

    # Highlight the range of angles where the desired yaw rate is between -0.5 and 0.5 rad/s
    for start, end in close_to_zero:
        plt.axvspan(start, end, color='green', alpha=0.5)

    # Add a caption to the plot
    caption = "Desired Steering angle range"
    plt.text(-100, 0.4, caption, bbox=dict(facecolor='green', alpha=0.5))

    plt.xlabel("Steering angle (deg)")
    plt.ylabel("Desired yaw rate (rad/s)")
    plt.title

    plt.show()