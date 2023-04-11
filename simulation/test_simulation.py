from simulation import *
import pytest

random_steering_angle = random.uniform(-230, 230)

def test_degrees_to_radians():
    degrees = random.uniform(0, 360)
    assert degrees_to_radians(degrees) == degrees * (math.pi / 180)
    assert degrees_to_radians(90) == 90 * (math.pi / 180)
    assert degrees_to_radians(90) == pytest.approx(1.5708, 0.0001)

def test_calculate_desired_yaw_rate():
    delta = random.uniform(-230, 230)
    Ku = calculate_yaw_rate(Cy_f, Cy_r)
    assert calculate_desired_yaw_rate(v_cg, delta) == v_cg / ((lf + lr) + Ku * v_cg**2) * delta

def test_calculate_yaw_rate():
    assert calculate_yaw_rate(Cy_f, Cy_r) == (lr * m) / (Cy_f * (lf + lr)) - (lf * m) / (Cy_r * (lf + lr))

def test_calculate_throttle_adjustment():
    random_velocity = random.uniform(0, 50)
    desired_yaw_rate = calculate_desired_yaw_rate(random_velocity, random_steering_angle)
    current_yaw_rate = simulate(random_velocity, random_steering_angle)[2]
    pid_controller = PIDController(2, 0.3, 20, 0.2)

    error = desired_yaw_rate - current_yaw_rate
    throttle_adjustment = pid_controller.control(error)
    assert calculate_throttle_adjustment(PIDController(2, 0.3, 20, 0.2), current_yaw_rate, desired_yaw_rate) == throttle_adjustment

def test_simulate():
    delta = degrees_to_radians(random_steering_angle)
    yaw_rate = calculate_yaw_rate(Cy_f, Cy_r)
    ay = v_cg * yaw_rate
    ax = v_cg ** 2 / (lf + lr) * delta
    assert simulate(v_cg, random_steering_angle) == (ax, ay, yaw_rate)